#!/usr/bin/env python3
"""gripper_node.py (rh_p12_rna_controller)
— ROS 2 Action 기반 + 순수 파이썬 소켓 TCP 서버(DRL) 통합 아키텍처 —

Doosan 내장 server_socket_write/read API는 str도 bytes도 거부하는 버그가 있음.
따라서 DRL 안에서 표준 파이썬 'import socket' 모듈을 사용하여 구현.
(이전 테스트에서 DRL 내부에서 import socket + socket.accept() 가 정상 동작함을 확인)
"""
from __future__ import annotations

import json
import struct
import socket
import threading
import time
import queue

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data

from rh_p12_rna_controller.action import GripperCommand
from dsr_msgs2.srv import DrlStart
from sensor_msgs.msg import JointState
from std_msgs.msg import String


TCP_MAGIC = b"GP"
TCP_VERSION = 1
TCP_HDR = struct.Struct(">2sBBHH")  # magic, ver, type, seq, payload_len

# Binary message types
TCP_T_PING = 1
TCP_T_PONG = 2
TCP_T_CMD = 3
TCP_T_ACK = 4
TCP_T_STATE = 5
TCP_T_STOP = 6


class ModbusRTU:
    @staticmethod
    def crc16(data: bytes) -> bytes:
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
        return struct.pack("<H", crc)

    @classmethod
    def fc06(cls, slave_id: int, start: int, value: int) -> bytes:
        body = bytes([slave_id, 0x06]) + struct.pack(">HH", start, value)
        return body + cls.crc16(body)

    @classmethod
    def fc16(cls, slave_id: int, start: int, num_regs: int, values: list) -> bytes:
        body = bytes([slave_id, 0x10]) + struct.pack(">HHB", start, num_regs, len(values) * 2)
        for val in values:
            body += struct.pack(">H", val)
        return body + cls.crc16(body)

    @classmethod
    def fc03(cls, slave_id: int, start: int, count: int) -> bytes:
        body = bytes([slave_id, 0x03]) + struct.pack(">HH", start, count)
        return body + cls.crc16(body)


def build_drl_write_packets(packets: list[bytes], wait_between: float = 0.1, tail_wait: float = 0.2) -> str:
    """`gripper_controller` 방식: DRL에 패킷을 직접 써서 실행시키는 짧은 코드 생성."""
    lines = [
        "flange_serial_open(baudrate=57600, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)",
        "wait(0.1)",
        "def _flush():",
        "    flange_serial_read(0.05)",
        "_flush()",
    ]
    for i, pkt in enumerate(packets):
        byte_str = "b'" + "".join([f"\\x{x:02x}" for x in pkt]) + "'"
        lines.append(f"flange_serial_write({byte_str})")
        lines.append(f"wait({wait_between})")
        lines.append("_flush()")
    if tail_wait > 0:
        lines.append(f"wait({tail_wait})")
    lines.append("flange_serial_close()")
    return "\n".join(lines) + "\n"


def build_drl_move_and_poll(
    slave_id: int,
    target_pulse: int,
    target_current: int,
    grip_current_threshold: int,
    pos_tolerance: int = 20,
    max_loops: int = 50,
) -> str:
    """`gripper_controller`에서 동작했던 방식: DRL이 move + readback polling까지 수행."""
    cur_move_pkt = "b'" + "".join([f"\\x{x:02x}" for x in ModbusRTU.fc06(slave_id, 275, target_current)]) + "'"
    pos_move_pkt = "b'" + "".join([f"\\x{x:02x}" for x in ModbusRTU.fc16(slave_id, 282, 2, [target_pulse, 0])]) + "'"
    pos_read_pkt = "b'" + "".join([f"\\x{x:02x}" for x in ModbusRTU.fc03(slave_id, 284, 2)]) + "'"
    cur_read_pkt = "b'" + "".join([f"\\x{x:02x}" for x in ModbusRTU.fc03(slave_id, 287, 1)]) + "'"

    # NOTE: DRL에서 readback은 (size, data) = flange_serial_read() 형태.
    code = (
        "flange_serial_open(baudrate=57600, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)\n"
        "wait(0.1)\n"
        "\n"
        "def _flush():\n"
        "    flange_serial_read(0.05)\n"
        "\n"
        "def _read_cur():\n"
        "    for _i in range(3):\n"
        "        _flush()\n"
        f"        flange_serial_write({cur_read_pkt})\n"
        "        wait(0.05)\n"
        "        _sz, _val = flange_serial_read(0.3)\n"
        "        if _sz >= 7 and _val[1] == 3:\n"
        "            _v = (_val[3] << 8) | _val[4]\n"
        "            if _v > 32767:\n"
        "                _v = _v - 65536\n"
        "            return _v\n"
        "    return -99999\n"
        "\n"
        "def _read_pos():\n"
        "    for _i in range(3):\n"
        "        _flush()\n"
        f"        flange_serial_write({pos_read_pkt})\n"
        "        wait(0.05)\n"
        "        _sz, _val = flange_serial_read(0.3)\n"
        "        if _sz >= 9 and _val[1] == 3:\n"
        "            _hi = (_val[3] << 8) | _val[4]\n"
        "            _lo = (_val[5] << 8) | _val[6]\n"
        "            return (_hi << 16) | _lo\n"
        "    return -99999\n"
        "\n"
        "_flush()\n"
        f"flange_serial_write({cur_move_pkt})\n"
        "wait(0.3)\n"
        "_flush()\n"
        "\n"
        "_flush()\n"
        f"flange_serial_write({pos_move_pkt})\n"
        "wait(0.5)\n"
        "_flush()\n"
        "\n"
        "__done = False\n"
        f"__loop = {max_loops}\n"
        "while not __done and __loop > 0:\n"
        "    __loop = __loop - 1\n"
        "    __cur = _read_cur()\n"
        "    if __cur != -99999:\n"
        f"        if abs(__cur) > {int(grip_current_threshold)}:\n"
        "            __done = True\n"
        "            break\n"
        "    __pos = _read_pos()\n"
        "    if __pos != -99999:\n"
        f"        if __pos >= {int(target_pulse - pos_tolerance)} and __pos <= {int(target_pulse + pos_tolerance)}:\n"
        "            __done = True\n"
        "            break\n"
        "    wait(0.15)\n"
        "\n"
        "flange_serial_close()\n"
    )
    return code


# ─── DRL 코드 (로봇 내장 환경에서 실행됨) ───
# 핵심: Doosan 내장 server_socket_* API 대신 표준 파이썬 socket 모듈 사용
# NOTE: __SLAVE_ID__, __TCP_PORT__, __PRESENT_CURRENT_REG__, __PRESENT_POSITION_REG__, __PRESENT_POSITION_REGS__,
#       __GOAL_POS_REG__, __GOAL_CUR_REG__,
#       __SNAP_ENABLED__, __PLC_FEEDBACK_ENABLED__, __PLC_ADDR_POS__, __PLC_ADDR_CUR__, __PLC_ADDR_CODE__
#       는 노트북에서 replace 해서 주입한다.
DRL_SERVER_CODE = """\
import socket
import json
import select
import time
import struct

SLAVE_ID = __SLAVE_ID__
TCP_PORT = __TCP_PORT__
PROTOCOL = "__TCP_PROTOCOL__"   # "json" | "binary"
STATE_PERIOD_SEC = __STATE_PERIOD_SEC__
TCP_STATE_STREAM_ENABLED = __TCP_STATE_STREAM_ENABLED__
TCP_CMD_FRAME_WAIT_SEC = __TCP_CMD_FRAME_WAIT_SEC__
PRESENT_CURRENT_REG = __PRESENT_CURRENT_REG__
PRESENT_POSITION_REG = __PRESENT_POSITION_REG__
PRESENT_POSITION_REGS = __PRESENT_POSITION_REGS__
GOAL_POS_REG = __GOAL_POS_REG__
GOAL_CUR_REG = __GOAL_CUR_REG__
SNAP_ENABLED = __SNAP_ENABLED__
PLC_FEEDBACK_ENABLED = __PLC_FEEDBACK_ENABLED__
PLC_ADDR_POS = __PLC_ADDR_POS__
PLC_ADDR_CUR = __PLC_ADDR_CUR__
PLC_ADDR_CODE = __PLC_ADDR_CODE__

flange_serial_open(baudrate=57600, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)
wait(0.1)

def _flush():
    # flange_serial_read()는 남아있는 바이트를 '한 번'에 다 못 비울 수 있음.
    # 짧게 여러 번 드레인해서 이전 응답/ACK 찌꺼기가 다음 읽기를 오염시키지 않게 한다.
    for _k in range(10):
        _sz, _val = flange_serial_read(0.01)
        if _sz <= 0:
            break

def _crc16(_data):
    _crc = 0xFFFF
    for _b in _data:
        _crc = _crc ^ _b
        for _i in range(8):
            if (_crc & 1) != 0:
                _crc = (_crc >> 1) ^ 0xA001
            else:
                _crc = (_crc >> 1)
    return _crc

def _fc03(_addr, _count):
    _pkt = bytes([SLAVE_ID, 0x03, (_addr >> 8) & 0xff, _addr & 0xff, (_count >> 8) & 0xff, _count & 0xff])
    _c = _crc16(_pkt)
    return _pkt + bytes([_c & 0xff, (_c >> 8) & 0xff])

def _fc06(_addr, _val):
    _pkt = bytes([SLAVE_ID, 0x06, (_addr >> 8) & 0xff, _addr & 0xff, (_val >> 8) & 0xff, _val & 0xff])
    _c = _crc16(_pkt)
    return _pkt + bytes([_c & 0xff, (_c >> 8) & 0xff])

def _read_reg16(_addr):
    for _i in range(3):
        _flush()
        flange_serial_write(_fc03(_addr, 1))
        wait(0.03)
        _sz, _val = flange_serial_read(0.2)
        if _sz >= 7 and _val[1] == 3 and _val[2] == 2:
            return ((_val[3] << 8) | _val[4])
    return -99999

def _read_cur():
    for _i in range(3):
        _flush()
        flange_serial_write(_fc03(PRESENT_CURRENT_REG, 1))   # PRESENT_CURRENT (configurable)
        wait(0.05)
        _sz, _val = flange_serial_read(0.3)
        # 1 reg read response: [id, 0x03, 0x02, hi, lo, crcLo, crcHi]
        if _sz >= 7 and _val[1] == 3 and _val[2] == 2:
            _v = (_val[3] << 8) | _val[4]
            if _v > 32767:
                _v = _v - 65536
            return _v
    return -99999

def _read_reg16(_addr):
    for _i in range(2):
        _flush()
        flange_serial_write(_fc03(_addr, 1))
        wait(0.03)
        _sz, _val = flange_serial_read(0.2)
        if _sz >= 7 and _val[1] == 3 and _val[2] == 2:
            return ((_val[3] << 8) | _val[4])
    return -99999

def _read_pos():
    for _i in range(3):
        _flush()
        flange_serial_write(_fc03(PRESENT_POSITION_REG, PRESENT_POSITION_REGS))
        wait(0.05)
        _sz, _val = flange_serial_read(0.3)
        # 1 reg read response: [id, 0x03, 0x02, hi, lo, crcLo, crcHi]
        if PRESENT_POSITION_REGS == 1:
            if _sz >= 7 and _val[1] == 3 and _val[2] == 2:
                return ((_val[3] << 8) | _val[4])
        # 2 reg read response: [id, 0x03, 0x04, hi1, lo1, hi2, lo2, crcLo, crcHi]
        else:
            if _sz >= 9 and _val[1] == 3 and _val[2] == 4:
                # IMPORTANT: Modbus returns registers in increasing address order.
                # Bridge implementation treats first word as LOW, second as HIGH (i32 = low + (high<<16)).
                _low = ((_val[3] << 8) | _val[4])
                _high = ((_val[5] << 8) | _val[6])
                _v = _low + (_high << 16)
                if _v >= 2147483648:
                    _v = _v - 4294967296
                return _v
    return -99999

def _read_cur_pos_bulk():
    # Bulk read if current/position regs are contiguous (reduces Modbus round trips).
    try:
        _pos_regs = int(PRESENT_POSITION_REGS)
        if _pos_regs < 1:
            _pos_regs = 1
        _cur_reg = int(PRESENT_CURRENT_REG)
        _pos_reg = int(PRESENT_POSITION_REG)
        _start = _cur_reg if _cur_reg < _pos_reg else _pos_reg
        _end = _cur_reg if _cur_reg > (_pos_reg + _pos_regs - 1) else (_pos_reg + _pos_regs - 1)
        _count = (_end - _start) + 1
        # keep request size small; fallback if too wide
        if _count <= 0 or _count > 16:
            return _read_cur(), _read_pos()
        for _i in range(3):
            _flush()
            flange_serial_write(_fc03(_start, _count))
            wait(0.05)
            _sz, _val = flange_serial_read(0.3)
            # response: [id, 0x03, bytecount, data..., crcLo, crcHi]
            if _sz < (5 + 2*_count) or _val[1] != 3:
                continue
            _data = _val[3:3+2*_count]
            # helpers
            def _reg_u16(_addr):
                _idx = _addr - _start
                if _idx < 0 or _idx >= _count:
                    return 0
                return (_data[2*_idx] << 8) | _data[2*_idx + 1]
            # current (signed 16-bit)
            _cur_u = _reg_u16(_cur_reg)
            _cur = _cur_u - 65536 if _cur_u > 32767 else _cur_u
            # position (1 reg or 2 regs)
            if _pos_regs == 1:
                _pos = _reg_u16(_pos_reg)
            else:
                _low = _reg_u16(_pos_reg)
                _high = _reg_u16(_pos_reg + 1)
                _pos = _low + (_high << 16)
                if _pos >= 2147483648:
                    _pos = _pos - 4294967296
            return _cur, _pos
    except:
        pass
    return _read_cur(), _read_pos()

# PLC Output Int GPR write helper
# Doosan manual 9.5.x: Output Int GPR address range is 0~23
def _clamp_out_int_addr(_a):
    try:
        _a = int(_a)
    except:
        return -1
    if _a < 0:
        return 0
    if _a > 23:
        return 23
    return _a

def _plc_write_int(_addr, _val):
    try:
        if not PLC_FEEDBACK_ENABLED:
            return
        _a = _clamp_out_int_addr(_addr)
        if _a < 0:
            return
        set_output_register_int(_a, int(_val))
    except:
        # never break gripper loop due to PLC write issues
        pass

# 토크 ON
_flush()
flange_serial_write(_fc06(256, 1))
wait(0.2)
_flush()

# 순수 파이썬 소켓으로 TCP 서버 열기 (Doosan 내장 API 사용 안 함)
MAGIC = b"GP"
VERSION = 1
HDR = struct.Struct(">2sBBHH")
T_PING = 1
T_PONG = 2
T_CMD  = 3
T_ACK  = 4
T_STATE= 5
T_STOP = 6

_srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
_srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    _srv.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
except Exception:
    pass
_srv.bind(('0.0.0.0', TCP_PORT))
_srv.listen(1)
_srv.settimeout(30.0)
tp_log("Pure Python TCP Server on port " + str(TCP_PORT) + ", waiting...")

_conn = None
try:
    _conn, _addr = _srv.accept()
    _conn.settimeout(0.05)
    try:
        _conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except Exception:
        pass
except Exception as _e:
    pass

if _conn:
    # --- Framed JSON protocol ---
    # Frame format: [len_hi][len_lo] + JSON(utf-8)
    def _sendall(_b):
        try:
            _off = 0
            while _off < len(_b):
                _sent = _conn.send(_b[_off:])
                if _sent <= 0:
                    break
                _off = _off + _sent
        except:
            pass

    def _send_obj(_obj):
        try:
            _payload = json.dumps(_obj).encode('utf-8')
            _sendall(bytes([(len(_payload) >> 8) & 0xff, len(_payload) & 0xff]) + _payload)
        except:
            pass

    _send_obj({"type": "hello", "info": "connected", "slave_id": SLAVE_ID, "tcp_port": TCP_PORT})
    # Initial PLC code: 0=boot/idle
    _plc_write_int(PLC_ADDR_CODE, 0)

    # event-driven command receive + rate-limited state stream
    try:
        _conn.setblocking(False)
    except Exception:
        pass
    _rxbuf = b""
    _last_state_t = 0.0

    def _send_bin(_type, _seq, _payload):
        try:
            _hdr = HDR.pack(MAGIC, VERSION, int(_type), int(_seq) & 0xFFFF, len(_payload))
            _conn.sendall(_hdr + _payload)
        except:
            pass

    def _recv_bin_packets():
        global _rxbuf
        packets = []
        while True:
            try:
                r, _, _ = select.select([_conn], [], [], 0)
                if not r:
                    break
                _raw = _conn.recv(2048)
                if _raw == b"":
                    return None
                if _raw:
                    if b"STOP" in _raw:
                        return "STOP"
                    _rxbuf = _rxbuf + _raw
            except Exception:
                break

        # Decode packets
        while True:
            if len(_rxbuf) < HDR.size:
                break
            try:
                _magic, _ver, _type, _seq, _plen = HDR.unpack(_rxbuf[:HDR.size])
            except:
                _rxbuf = _rxbuf[1:]
                continue
            if _magic != MAGIC or _ver != VERSION or _plen < 0 or _plen > 65535:
                _rxbuf = _rxbuf[1:]
                continue
            if len(_rxbuf) < HDR.size + _plen:
                break
            _payload = _rxbuf[HDR.size:HDR.size+_plen]
            _rxbuf = _rxbuf[HDR.size+_plen:]
            packets.append((_type, _seq, _payload))
        return packets

    def _drain_rx_json():
        global _rxbuf
        msgs = []
        # Read everything currently available (non-blocking)
        while True:
            try:
                r, _, _ = select.select([_conn], [], [], 0)
                if not r:
                    break
                _raw = _conn.recv(2048)
                if _raw == b"":
                    return None  # peer closed
                if _raw:
                    if b"STOP" in _raw:
                        return "STOP"
                    _rxbuf = _rxbuf + _raw
            except Exception:
                break

        # Decode framed JSON messages
        while len(_rxbuf) >= 2:
            _n = (_rxbuf[0] << 8) | _rxbuf[1]
            if len(_rxbuf) < 2 + _n:
                break
            _payload = _rxbuf[2:2+_n]
            _rxbuf = _rxbuf[2+_n:]
            try:
                msgs.append(json.loads(_payload.decode('utf-8', errors='ignore')))
            except:
                pass
        return msgs

    while True:
        # 1) Command receive (event-driven)
        if PROTOCOL == "binary":
            _pkts = _recv_bin_packets()
            if _pkts is None or _pkts == "STOP":
                break
            for _type, _seq, _payload in (_pkts or []):
                if _type == T_PING:
                    _send_bin(T_PONG, _seq, b"")
                    continue
                if _type == T_STOP:
                    break
                if _type != T_CMD:
                    continue
                # payload: [num_frames u16] + (len u16 + frame bytes)...
                _ok = True
                _err = ""
                try:
                    if len(_payload) < 2:
                        _ok = False
                    else:
                        _n = (_payload[0] << 8) | _payload[1]
                        _off = 2
                        _flush()
                        for _k in range(_n):
                            if _off + 2 > len(_payload):
                                _ok = False
                                break
                            _flen = (_payload[_off] << 8) | _payload[_off+1]
                            _off = _off + 2
                            if _off + _flen > len(_payload):
                                _ok = False
                                break
                            _pkt = _payload[_off:_off+_flen]
                            _off = _off + _flen
                            flange_serial_write(_pkt)
                            wait(TCP_CMD_FRAME_WAIT_SEC)
                            _flush()
                except Exception as _e:
                    _ok = False
                    _err = str(_e)
                # ack payload: [ok u8][err_len u16][err bytes]
                try:
                    _eb = (_err or "").encode("utf-8") if (not _ok and _err) else b""
                    if len(_eb) > 200:
                        _eb = _eb[:200]
                    _ap = bytes([1 if _ok else 0]) + bytes([(len(_eb) >> 8) & 0xFF, len(_eb) & 0xFF]) + _eb
                    _send_bin(T_ACK, _seq, _ap)
                except:
                    pass
                _plc_write_int(PLC_ADDR_CODE, 1 if _ok else -1)
        else:
            _cmd_msgs = _drain_rx_json()
            if _cmd_msgs is None or _cmd_msgs == "STOP":
                break
            if _cmd_msgs:
                for _m in _cmd_msgs:
                    # ping/pong for protocol sanity check
                    if _m.get("type", "") == "ping":
                        _send_obj({"type": "pong", "t": 0})
                        continue
                    _cmd_id = _m.get("id", 0)
                    _frames = _m.get("frames", [])
                    _flush()
                    _ok = True
                    _err = ""
                    try:
                        for _hx in _frames:
                            _pkt = bytes.fromhex(_hx)
                            flange_serial_write(_pkt)
                            # keep waits minimal; flush consumes ACK residues
                            wait(TCP_CMD_FRAME_WAIT_SEC)
                            _flush()
                    except Exception as _e:
                        _ok = False
                        _err = str(_e)
                    # ACK ASAP
                    _send_obj({"type": "ack", "id": _cmd_id, "ok": _ok, "err": _err})
                    _plc_write_int(PLC_ADDR_CODE, 1 if _ok else -1)

                    if SNAP_ENABLED:
                        try:
                            _snap = {}
                            for _addr in range(270, 291):
                                _snap[str(_addr)] = _read_reg16(_addr)
                            _send_obj({"type": "snap", "id": _cmd_id, "snap": _snap})
                        except:
                            pass

        # 2) State stream (rate-limited; disable during sim2real to free flange_serial)
        _now = time.time()
        if TCP_STATE_STREAM_ENABLED and (_now - _last_state_t) >= STATE_PERIOD_SEC:
            _last_state_t = _now
            cur_val, pos_val = _read_cur_pos_bulk()
            if cur_val != -99999 and pos_val != -99999:
                _plc_write_int(PLC_ADDR_CUR, cur_val)
                _plc_write_int(PLC_ADDR_POS, pos_val)
                gcur = _read_reg16(GOAL_CUR_REG)
                gpos_lo = _read_reg16(GOAL_POS_REG)
                gpos_hi = _read_reg16(GOAL_POS_REG + 1)
                if PROTOCOL == "binary":
                    # payload: cur(i16), pos(i32), gcur(i16), gpos_lo(u16), gpos_hi(u16)
                    try:
                        _p = struct.pack(">hi hHH", int(cur_val), int(pos_val), int(gcur), int(gpos_lo) & 0xFFFF, int(gpos_hi) & 0xFFFF)
                    except Exception:
                        _p = b""
                    _send_bin(T_STATE, 0, _p)
                else:
                    _send_obj({
                        "type": "state",
                        "cur": cur_val,
                        "pos": pos_val,
                        "pcur_reg": PRESENT_CURRENT_REG,
                        "gcur_reg": GOAL_CUR_REG,
                        "gpos_reg": GOAL_POS_REG,
                        "gcur": gcur,
                        "gpos_lo": gpos_lo,
                        "gpos_hi": gpos_hi,
                    })

        # 3) tiny yield to avoid busy-loop
        wait(0.005)

    _conn.close()

_srv.close()
flange_serial_close()
tp_log("TCP Server closed.")
"""


class GripperNode(Node):
    def __init__(self):
        super().__init__("gripper_node")
        self._cb = ReentrantCallbackGroup()

        # ---- 파라미터 ----
        self.declare_parameter("robot_ns", "dsr01")
        self.declare_parameter("robot_ip", "110.120.1.40")
        self.declare_parameter("robot_port", 9000)
        # 로봇 없이 TCP 프로토콜만 테스트할 때 True:
        # - /drl/drl_start 주입 생략
        # - 지정된 robot_ip:robot_port의 외부 TCP 서버로 바로 접속
        self.declare_parameter("tcp_external_server", False)
        self.declare_parameter("state_hz", 10.0)
        self.declare_parameter("grip_current_threshold", 50)
        # Default: do not scale raw pos. Some firmware reports pulse*64; in that case set position_scale:=64.
        self.declare_parameter("position_scale", 1.0)
        # position 파싱 옵션 (펌웨어/레지스터 맵 차이 대응)
        # - use_low_word: 32-bit 값 중 low 16-bit만 사용 (많이 쓰는 pulse*64가 여기에 있음)
        # - word_order: "hi_lo"(기본) 또는 "lo_hi" (32-bit 워드 조합 순서)
        self.declare_parameter("position_use_low_word", False)
        self.declare_parameter("position_word_order", "hi_lo")
        # e0509 + RH-P12 계열에서 레지스터 값이 pulse*64 스케일로 관측됨
        self.declare_parameter("goal_position_scale", 1.0)
        self.declare_parameter("command_transport", "drl")  # "drl" | "tcp"
        self.declare_parameter("slave_id", 1)
        self.declare_parameter("present_current_reg", 287)
        # RH-P12-RN(A) default map (matches bridge_compare):
        # - present_velocity: 288~289
        # - present_position: 290~291 (32-bit, low word then high word)
        self.declare_parameter("present_position_reg", 290)
        self.declare_parameter("present_position_regs", 2)  # 1 or 2
        # RH-P12-RN(A) default map (matches bridge_compare):
        # - goal_current: 275
        # - goal_position: 282~283 (32-bit, low word then high word)
        self.declare_parameter("goal_current_reg", 275)
        self.declare_parameter("goal_position_reg", 282)
        # Default to fc16 for robustness on RH-P12-RN(A) (some controllers don't reliably reflect fc06 writes).
        self.declare_parameter("goal_position_write_mode", "fc16")  # auto|fc06|fc16
        self.declare_parameter("goal_position_regs", 2)  # 1 or 2
        self.declare_parameter("drl_snap_enabled", False)
        # PLC feedback via Industrial Ethernet Output Int GPR (Doosan manual 9.5.x)
        # - enable: DRL TCP server writes current/position/code to set_output_register_int()
        # - addr range: 0~23
        self.declare_parameter("plc_feedback_enabled", False)
        self.declare_parameter("plc_addr_pos", 0)
        self.declare_parameter("plc_addr_cur", 2)
        self.declare_parameter("plc_addr_code", 1)
        # TCP robustness / latency tuning
        # On real controller, first ACK after reinject can be slower.
        self.declare_parameter("tcp_ack_timeout_sec", 3.0)
        # DRL TCP server: state streaming rate (controller -> PC); 0 = off
        self.declare_parameter("tcp_state_hz", 20.0)
        self.declare_parameter("tcp_state_stream_enabled", True)
        self.declare_parameter("tcp_cmd_frame_wait_sec", 0.02)
        # TCP protocol between PC and DRL TCP server
        self.declare_parameter("tcp_protocol", "json")  # json|binary
        self.declare_parameter("tcp_watchdog_enabled", True)
        self.declare_parameter("tcp_watchdog_period_sec", 1.0)
        self.declare_parameter("tcp_watchdog_stale_sec", 2.5)
        self.declare_parameter("tcp_rx_buf_max_bytes", 65536)
        self.declare_parameter("tcp_rx_buf_keep_bytes", 8192)
        # 완료 판정 튜닝
        self.declare_parameter("done_pos_tolerance", 20)
        self.declare_parameter("done_min_motion", 10)
        self.declare_parameter("done_require_reached", True)
        self.declare_parameter("grip_detect_enabled", True)
        self.declare_parameter("action_max_wait_sec", 20.0)
        self.declare_parameter("pulse_open", 100)
        self.declare_parameter("pulse_cube", 420)
        self.declare_parameter("pulse_rotate", 400)
        self.declare_parameter("pulse_repose", 410)
        self.declare_parameter("init_current", 400)
        self.declare_parameter("cube_current", 300)
        # action name (so CLI path matches consistently)
        self.declare_parameter("action_name", "/rh_p12_rna_controller/gripper_command")
        self.declare_parameter("gripper_command_action_enabled", True)
        # direct (topic-based) immediate command interface
        self.declare_parameter("direct_cmd_topic_enabled", False)
        self.declare_parameter("direct_cmd_topic", "/gripper/cmd_direct")
        # sim2real streaming: keep only latest target, skip stale queue + redundant TCP
        self.declare_parameter("direct_cmd_latest_only", True)
        self.declare_parameter("direct_cmd_streaming", True)
        self.declare_parameter("direct_cmd_streaming_ack_timeout_sec", 0.5)
        self.declare_parameter("direct_cmd_streaming_no_ack", True)

        ns = str(self.get_parameter("robot_ns").value).strip()
        self._prefix = f"/{ns}" if ns else ""
        self._robot_ip = str(self.get_parameter("robot_ip").value).strip()
        self._tcp_port = int(self.get_parameter("robot_port").value)
        self._tcp_external = bool(self.get_parameter("tcp_external_server").value)
        self._state_hz = float(self.get_parameter("state_hz").value)
        self._grip_threshold = float(self.get_parameter("grip_current_threshold").value)
        self._pos_scale = float(self.get_parameter("position_scale").value)
        self._pos_use_low = bool(self.get_parameter("position_use_low_word").value)
        self._pos_word_order = str(self.get_parameter("position_word_order").value).strip().lower()
        self._goal_pos_scale = float(self.get_parameter("goal_position_scale").value)
        self._cmd_transport = str(self.get_parameter("command_transport").value).strip().lower()
        self._slave_id = int(self.get_parameter("slave_id").value)
        self._present_current_reg = int(self.get_parameter("present_current_reg").value)
        self._present_position_reg = int(self.get_parameter("present_position_reg").value)
        self._present_position_regs = int(self.get_parameter("present_position_regs").value)
        self._goal_cur_reg = int(self.get_parameter("goal_current_reg").value)
        self._goal_pos_reg = int(self.get_parameter("goal_position_reg").value)
        self._goal_pos_write_mode = str(self.get_parameter("goal_position_write_mode").value).strip().lower()
        self._goal_pos_regs = int(self.get_parameter("goal_position_regs").value)
        self._drl_snap_enabled = bool(self.get_parameter("drl_snap_enabled").value)
        self._plc_feedback_enabled = bool(self.get_parameter("plc_feedback_enabled").value)
        self._plc_addr_pos = int(self.get_parameter("plc_addr_pos").value)
        self._plc_addr_cur = int(self.get_parameter("plc_addr_cur").value)
        self._plc_addr_code = int(self.get_parameter("plc_addr_code").value)
        self._tcp_ack_timeout = float(self.get_parameter("tcp_ack_timeout_sec").value)
        self._tcp_state_hz = float(self.get_parameter("tcp_state_hz").value)
        self._tcp_state_stream_enabled = bool(
            self.get_parameter("tcp_state_stream_enabled").value
        )
        self._tcp_cmd_frame_wait_sec = max(
            0.0, float(self.get_parameter("tcp_cmd_frame_wait_sec").value)
        )
        self._tcp_protocol = str(self.get_parameter("tcp_protocol").value).strip().lower()
        if self._tcp_protocol not in ("json", "binary"):
            self._tcp_protocol = "json"
        self._tcp_wd_enabled = bool(self.get_parameter("tcp_watchdog_enabled").value)
        self._tcp_wd_period = float(self.get_parameter("tcp_watchdog_period_sec").value)
        self._tcp_wd_stale = float(self.get_parameter("tcp_watchdog_stale_sec").value)
        self._tcp_rx_max = int(self.get_parameter("tcp_rx_buf_max_bytes").value)
        self._tcp_rx_keep = int(self.get_parameter("tcp_rx_buf_keep_bytes").value)
        self._done_tol = int(self.get_parameter("done_pos_tolerance").value)
        self._done_min_motion = int(self.get_parameter("done_min_motion").value)
        self._done_require_reached = bool(self.get_parameter("done_require_reached").value)
        self._grip_enabled = bool(self.get_parameter("grip_detect_enabled").value)
        self._action_max_wait = float(self.get_parameter("action_max_wait_sec").value)
        self._pulse_open = int(self.get_parameter("pulse_open").value)
        self._pulse_cube = int(self.get_parameter("pulse_cube").value)
        self._pulse_rotate = int(self.get_parameter("pulse_rotate").value)
        self._pulse_repose = int(self.get_parameter("pulse_repose").value)
        self._cur_init = int(self.get_parameter("init_current").value)
        self._cur_cube = int(self.get_parameter("cube_current").value)
        self._action_name = str(self.get_parameter("action_name").value).strip()
        self._gripper_action_enabled = bool(self.get_parameter("gripper_command_action_enabled").value)
        self._direct_cmd_enabled = bool(self.get_parameter("direct_cmd_topic_enabled").value)
        self._direct_cmd_topic = str(self.get_parameter("direct_cmd_topic").value).strip() or "/gripper/cmd_direct"
        self._direct_cmd_latest_only = bool(self.get_parameter("direct_cmd_latest_only").value)
        self._direct_cmd_streaming = bool(self.get_parameter("direct_cmd_streaming").value)
        self._direct_cmd_streaming_ack_timeout = float(
            self.get_parameter("direct_cmd_streaming_ack_timeout_sec").value
        )
        self._direct_cmd_streaming_no_ack = bool(
            self.get_parameter("direct_cmd_streaming_no_ack").value
        )
        self._last_sent_direct_pulse: int | None = None
        self._last_sent_direct_cur: int | None = None
        self._direct_stream_lock = threading.Lock()

        # Validate PLC Output Int GPR addresses (0~23). We clamp only if enabled.
        if self._plc_feedback_enabled:
            self._plc_addr_pos = self._clamp_plc_out_int_addr(self._plc_addr_pos, "plc_addr_pos")
            self._plc_addr_cur = self._clamp_plc_out_int_addr(self._plc_addr_cur, "plc_addr_cur")
            self._plc_addr_code = self._clamp_plc_out_int_addr(self._plc_addr_code, "plc_addr_code")

        # ---- 상태 변수 ----
        self._current_hz_pos = 0
        self._current_hz_cur = 0
        self._last_state_rx_t = 0.0
        self._sock = None
        self._socket_active = False
        self._tcp_rx_buf = b""
        self._ack_lock = threading.Lock()
        self._ack_waiters: dict[int, threading.Event] = {}
        self._ack_results: dict[int, dict] = {}
        self._next_cmd_id = 1
        self._tcp_hello_seen = False
        self._tcp_state_seen = False
        self._tcp_ack_seen = False
        self._tcp_sniff_logged = False
        self._tcp_pong_seen = False
        self._recv_thread = None
        self._last_gpos_lo = None
        self._last_pong_rx_t = 0.0
        self._watchdog_thread = None
        # Prevent reconnect/reinject races (only one at a time)
        self._tcp_reconnect_lock = threading.Lock()
        self._tcp_reconnect_inflight = False

        # ---- ROS 인터페이스 ----
        self._cli_drl = self.create_client(
            DrlStart, f"{self._prefix}/drl/drl_start", callback_group=self._cb
        )
        # High-rate telemetry: prefer sensor QoS (best-effort, small depth)
        self._state_pub = self.create_publisher(JointState, "/gripper/state", qos_profile_sensor_data)
        self.create_timer(1.0 / self._state_hz, self._publish_state, callback_group=self._cb)

        self._direct_sub = None
        self._direct_cmd_q: queue.Queue[tuple[str, int, int]] | None = None
        self._direct_cmd_worker_thread: threading.Thread | None = None
        if self._direct_cmd_enabled:
            self._direct_cmd_q = queue.Queue(maxsize=20)
            self._direct_cmd_worker_thread = threading.Thread(
                target=self._direct_cmd_worker_loop,
                daemon=True,
            )
            self._direct_cmd_worker_thread.start()
            self._direct_sub = self.create_subscription(
                String,
                self._direct_cmd_topic,
                self._on_direct_cmd,
                10,
                callback_group=self._cb,
            )
            self.get_logger().info(f"direct cmd topic enabled: {self._direct_cmd_topic} (std_msgs/String)")

        self._executing = False
        self._action_server = None
        if self._gripper_action_enabled:
            self._action_server = ActionServer(
                self,
                GripperCommand,
                self._action_name,
                execute_callback=self._execute_callback,
                goal_callback=self._goal_callback,
                cancel_callback=self._cancel_callback,
                callback_group=self._cb,
            )

        self.get_logger().info("초기화 대기 중 (DRL 서버 주입)...")
        self._init_timer = self.create_timer(1.0, self._init_drl_server, callback_group=self._cb)
        self.add_on_set_parameters_callback(self._on_param_set)
        if self._tcp_wd_enabled:
            self._watchdog_thread = threading.Thread(target=self._tcp_watchdog_loop, daemon=True)
            self._watchdog_thread.start()

    def _tcp_state_period_sec(self) -> float:
        if not self._tcp_state_stream_enabled:
            return 999999.0
        hz = float(self._tcp_state_hz) if self._tcp_state_hz else 0.0
        if hz <= 0.0:
            return 999999.0
        if hz > 50.0:
            hz = 50.0
        return 1.0 / hz

    def _build_drl_server_code(self) -> str:
        return (
            DRL_SERVER_CODE.replace("__SLAVE_ID__", str(self._slave_id))
            .replace("__TCP_PORT__", str(self._tcp_port))
            .replace("__TCP_PROTOCOL__", str(self._tcp_protocol))
            .replace("__STATE_PERIOD_SEC__", str(float(self._tcp_state_period_sec())))
            .replace(
                "__TCP_STATE_STREAM_ENABLED__",
                "True" if self._tcp_state_stream_enabled else "False",
            )
            .replace("__TCP_CMD_FRAME_WAIT_SEC__", str(float(self._tcp_cmd_frame_wait_sec)))
            .replace("__PRESENT_CURRENT_REG__", str(self._present_current_reg))
            .replace("__PRESENT_POSITION_REG__", str(self._present_position_reg))
            .replace("__PRESENT_POSITION_REGS__", str(self._present_position_regs))
            .replace("__GOAL_POS_REG__", str(self._goal_pos_reg))
            .replace("__GOAL_CUR_REG__", str(self._goal_cur_reg))
            .replace("__SNAP_ENABLED__", "True" if self._drl_snap_enabled else "False")
            .replace(
                "__PLC_FEEDBACK_ENABLED__",
                "True" if self._plc_feedback_enabled else "False",
            )
            .replace("__PLC_ADDR_POS__", str(int(self._plc_addr_pos)))
            .replace("__PLC_ADDR_CUR__", str(int(self._plc_addr_cur)))
            .replace("__PLC_ADDR_CODE__", str(int(self._plc_addr_code)))
        )

    def _configure_tcp_socket(self, sock: socket.socket) -> None:
        # Low-latency + keepalive (best-effort; some options may not exist).
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except Exception:
            pass
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        except Exception:
            pass
        # Linux-specific TCP keepalive tuning (ignore if unavailable)
        for opt_name, val in (("TCP_KEEPIDLE", 10), ("TCP_KEEPINTVL", 3), ("TCP_KEEPCNT", 3)):
            try:
                opt = getattr(socket, opt_name, None)
                if opt is not None:
                    sock.setsockopt(socket.IPPROTO_TCP, opt, int(val))
            except Exception:
                pass

    def _send_bin(self, mtype: int, seq: int, payload: bytes = b"") -> None:
        hdr = TCP_HDR.pack(TCP_MAGIC, TCP_VERSION, int(mtype) & 0xFF, int(seq) & 0xFFFF, len(payload))
        self._sock.sendall(hdr + (payload or b""))

    def _bin_try_parse_one(self) -> tuple[int, int, bytes] | None:
        """Parse one GP binary packet from rx buffer. Returns (type, seq, payload) or None."""
        buf = self._tcp_rx_buf
        if len(buf) < TCP_HDR.size:
            return None
        try:
            magic, ver, mtype, seq, plen = TCP_HDR.unpack(buf[: TCP_HDR.size])
        except Exception:
            self._tcp_rx_buf = buf[1:]
            return None
        if magic != TCP_MAGIC or ver != TCP_VERSION or plen < 0 or plen > 65535:
            self._tcp_rx_buf = buf[1:]
            return None
        if len(buf) < TCP_HDR.size + plen:
            return None
        payload = buf[TCP_HDR.size : TCP_HDR.size + plen]
        self._tcp_rx_buf = buf[TCP_HDR.size + plen :]
        return int(mtype), int(seq), payload

    def _resolve_preset_action(self, action: str, pulse: int = 0, current: int = 0) -> tuple[int, int]:
        a = (action or "").strip()
        if a in ("open", "release"):
            return self._pulse_open, self._cur_init
        if a == "grab_cube":
            return self._pulse_cube, self._cur_cube
        if a == "grab_rotate":
            return self._pulse_rotate, self._cur_cube
        if a == "grab_repose":
            return self._pulse_repose, self._cur_cube
        if a == "custom":
            p = int(pulse) if int(pulse) >= 0 else self._pulse_cube
            c = int(current) if int(current) > 0 else self._cur_init
            return p, c
        return -1, -1

    def _on_direct_cmd(self, msg: String) -> None:
        """Immediate, best-effort command path.

        Payload examples:
        - "open"
        - "grab_cube"
        - "custom 420 300"  (action pulse current)
        """
        raw = (msg.data or "").strip()
        if not raw:
            return
        parts = raw.split()
        action = parts[0]
        pulse = int(parts[1]) if len(parts) >= 2 else 0
        current = int(parts[2]) if len(parts) >= 3 else 0
        t_pulse, t_cur = self._resolve_preset_action(action, pulse=pulse, current=current)
        if t_pulse == -1:
            self.get_logger().warn(f"direct cmd rejected: {raw!r}")
            return

        if (
            self._direct_cmd_streaming
            and self._direct_cmd_streaming_no_ack
            and self._cmd_transport == "tcp"
        ):
            self._direct_cmd_streaming_inline(action, int(t_pulse), int(t_cur))
            return

        # Enqueue and return immediately (avoid per-message thread spawn).
        if not self._direct_cmd_q:
            return
        item = (action, int(t_pulse), int(t_cur))
        if self._direct_cmd_latest_only:
            while True:
                try:
                    self._direct_cmd_q.get_nowait()
                except queue.Empty:
                    break
        try:
            self._direct_cmd_q.put_nowait(item)
        except queue.Full:
            try:
                _ = self._direct_cmd_q.get_nowait()
            except Exception:
                pass
            try:
                self._direct_cmd_q.put_nowait(item)
            except Exception:
                pass

    def _direct_cmd_streaming_inline(
        self, action: str, t_pulse: int, t_cur: int
    ) -> None:
        with self._direct_stream_lock:
            if (
                self._last_sent_direct_pulse == int(t_pulse)
                and self._last_sent_direct_cur == int(t_cur)
            ):
                return
            if not self._socket_active:
                return

            pos_frames = self._direct_cmd_pos_frames(int(t_pulse))
            need_cur = self._last_sent_direct_cur != int(t_cur)
            frames: list[bytes] = []
            if need_cur:
                frames.append(
                    ModbusRTU.fc06(self._slave_id, self._goal_cur_reg, int(t_cur))
                )
            frames.extend(pos_frames)

            ok, err = self._send_cmd_fire_and_forget(frames)
            if not ok:
                self.get_logger().warning(
                    f'direct inline send failed: {err}',
                    throttle_duration_sec=2.0,
                )
                return

            self._last_sent_direct_pulse = int(t_pulse)
            self._last_sent_direct_cur = int(t_cur)
            self.get_logger().debug(
                f'direct inline: action={action} pos={t_pulse} cur={t_cur}'
            )

    def _drain_direct_cmd_latest(self) -> tuple[str, int, int]:
        assert self._direct_cmd_q is not None
        action, t_pulse, t_cur = self._direct_cmd_q.get()
        while True:
            try:
                action, t_pulse, t_cur = self._direct_cmd_q.get_nowait()
            except queue.Empty:
                break
        return action, t_pulse, t_cur

    def _direct_cmd_worker_loop(self) -> None:
        assert self._direct_cmd_q is not None
        while rclpy.ok():
            try:
                if self._direct_cmd_latest_only:
                    action, t_pulse, t_cur = self._drain_direct_cmd_latest()
                else:
                    action, t_pulse, t_cur = self._direct_cmd_q.get()
                self._direct_cmd_worker(action, t_pulse, t_cur)
            except Exception:
                continue

    def _direct_cmd_pos_frames(self, t_pulse: int) -> list[bytes]:
        goal_val = int(round(float(t_pulse) * (self._goal_pos_scale if self._goal_pos_scale else 1.0)))
        pos_pkt_fc06 = ModbusRTU.fc06(self._slave_id, self._goal_pos_reg, int(goal_val) & 0xFFFF)
        pos_pkt_fc16 = ModbusRTU.fc16(self._slave_id, self._goal_pos_reg, 2, [int(goal_val) & 0xFFFF, 0])
        regs = int(self._goal_pos_regs)
        mode = str(self._goal_pos_write_mode)
        if regs <= 1:
            return [pos_pkt_fc06]
        if mode == "fc16":
            return [pos_pkt_fc16]
        return [pos_pkt_fc06]

    def _direct_cmd_worker(self, action: str, t_pulse: int, t_cur: int) -> None:
        try:
            if (
                self._direct_cmd_streaming
                and self._last_sent_direct_pulse == int(t_pulse)
                and self._last_sent_direct_cur == int(t_cur)
            ):
                return

            self.get_logger().info(f"direct cmd: action={action} pos={t_pulse} cur={t_cur}")
            cur_pkt = ModbusRTU.fc06(self._slave_id, self._goal_cur_reg, int(t_cur))

            if self._cmd_transport == "drl":
                drl_code = build_drl_move_and_poll(
                    slave_id=self._slave_id,
                    target_pulse=int(t_pulse),
                    target_current=int(t_cur),
                    grip_current_threshold=int(self._grip_threshold),
                    pos_tolerance=int(self._done_tol),
                    max_loops=60,
                )
                ok = self._call_drl(drl_code, timeout_sec=15.0)
                self.get_logger().info(f"direct cmd done(drl): ok={ok}")
                return

            # TCP mode
            if not self._socket_active:
                self.get_logger().warn("direct cmd failed: TCP offline")
                return

            pos_frames = self._direct_cmd_pos_frames(int(t_pulse))
            need_cur = (
                not self._direct_cmd_streaming
                or self._last_sent_direct_cur != int(t_cur)
            )
            frames: list[bytes] = []
            if need_cur:
                frames.append(cur_pkt)
            frames.extend(pos_frames)

            use_no_ack = bool(
                self._direct_cmd_streaming and self._direct_cmd_streaming_no_ack
            )
            send_fn = self._send_cmd_fire_and_forget if use_no_ack else self._send_cmd_and_wait_ack
            ack_timeout = (
                self._direct_cmd_streaming_ack_timeout
                if self._direct_cmd_streaming and not use_no_ack
                else None
            )

            sent = False
            if self._direct_cmd_streaming and len(frames) > 1:
                ok, err = send_fn(frames, timeout_sec=ack_timeout)
                if ok:
                    sent = True
                elif need_cur and len(pos_frames) == 1 and not use_no_ack:
                    ok2, err2 = send_fn([cur_pkt], timeout_sec=ack_timeout)
                    if not ok2:
                        self.get_logger().error(f"direct cmd cur ack failed: {err2}")
                        return
                    ok3, err3 = send_fn(pos_frames, timeout_sec=ack_timeout)
                    if not ok3:
                        self.get_logger().error(f"direct cmd pos ack failed: {err3}")
                        return
                    sent = True
                elif not ok:
                    self.get_logger().error(f"direct cmd send failed: {err}")
                    return
            elif len(pos_frames) == 1 and not need_cur:
                ok, err = send_fn(pos_frames, timeout_sec=ack_timeout)
                if not ok:
                    self.get_logger().error(f"direct cmd pos send failed: {err}")
                    return
                sent = True
            else:
                if need_cur:
                    ok1, err1 = send_fn([cur_pkt], timeout_sec=ack_timeout)
                    if not ok1:
                        self.get_logger().error(f"direct cmd cur send failed: {err1}")
                        return
                    if not use_no_ack:
                        time.sleep(0.02)
                mode = str(self._goal_pos_write_mode)
                if mode in ("fc06", "auto"):
                    ok2, err2 = send_fn([pos_frames[0]], timeout_sec=ack_timeout)
                    if ok2:
                        sent = True
                    elif mode == "fc06":
                        self.get_logger().error(f"direct cmd pos(fc06) send failed: {err2}")
                        return
                if (not sent) and mode in ("fc16", "auto"):
                    fc16_pkt = pos_frames[-1]
                    ok3, err3 = send_fn([fc16_pkt], timeout_sec=ack_timeout)
                    if not ok3:
                        self.get_logger().error(f"direct cmd pos(fc16) send failed: {err3}")
                        return
                    sent = True

            if sent:
                self._last_sent_direct_pulse = int(t_pulse)
                self._last_sent_direct_cur = int(t_cur)
            tag = "fire-and-forget" if use_no_ack else "tcp"
            self.get_logger().info(f"direct cmd sent({tag}): ok={sent}")
        except Exception as e:
            self.get_logger().error(f"direct cmd exception: {e}")

    def _clamp_plc_out_int_addr(self, addr: int, name: str) -> int:
        a = int(addr)
        if a < 0 or a > 23:
            clamped = 0 if a < 0 else 23
            self.get_logger().warn(
                f"{name}={a} is out of range (0~23, Output Int GPR). Clamping to {clamped}."
            )
            return clamped
        return a

    def _on_param_set(self, params):
        """Allow runtime tuning via `ros2 param set`.

        Important: some values affect DRL-injected code (server-side reads). Those still
        require reinjection/restart to fully reflect on the DRL side, but client-side
        command addressing should update immediately.
        """
        try:
            need_reinject_tcp = False
            for p in params:
                if p.name == "present_current_reg":
                    self._present_current_reg = int(p.value)
                    need_reinject_tcp = True
                elif p.name == "present_position_reg":
                    self._present_position_reg = int(p.value)
                    need_reinject_tcp = True
                elif p.name == "present_position_regs":
                    self._present_position_regs = int(p.value)
                    need_reinject_tcp = True
                elif p.name == "goal_current_reg":
                    self._goal_cur_reg = int(p.value)
                    need_reinject_tcp = True
                elif p.name == "goal_position_reg":
                    self._goal_pos_reg = int(p.value)
                    need_reinject_tcp = True
                elif p.name == "goal_position_write_mode":
                    self._goal_pos_write_mode = str(p.value).strip().lower()
                elif p.name == "goal_position_regs":
                    self._goal_pos_regs = int(p.value)
                elif p.name == "goal_position_scale":
                    self._goal_pos_scale = float(p.value)
                elif p.name == "position_scale":
                    self._pos_scale = float(p.value)
                elif p.name == "grip_current_threshold":
                    self._grip_threshold = float(p.value)
                elif p.name == "done_pos_tolerance":
                    self._done_tol = int(p.value)
                elif p.name == "done_min_motion":
                    self._done_min_motion = int(p.value)
                elif p.name == "done_require_reached":
                    self._done_require_reached = bool(p.value)
                elif p.name == "grip_detect_enabled":
                    self._grip_enabled = bool(p.value)
                elif p.name == "action_max_wait_sec":
                    self._action_max_wait = float(p.value)
                elif p.name == "drl_snap_enabled":
                    self._drl_snap_enabled = bool(p.value)
                    need_reinject_tcp = True
                elif p.name == "plc_feedback_enabled":
                    self._plc_feedback_enabled = bool(p.value)
                    need_reinject_tcp = True
                elif p.name == "plc_addr_pos":
                    self._plc_addr_pos = int(p.value)
                    need_reinject_tcp = True
                elif p.name == "plc_addr_cur":
                    self._plc_addr_cur = int(p.value)
                    need_reinject_tcp = True
                elif p.name == "plc_addr_code":
                    self._plc_addr_code = int(p.value)
                    need_reinject_tcp = True
                elif p.name == "tcp_ack_timeout_sec":
                    self._tcp_ack_timeout = float(p.value)
                elif p.name == "tcp_watchdog_enabled":
                    self._tcp_wd_enabled = bool(p.value)
                elif p.name == "tcp_watchdog_period_sec":
                    self._tcp_wd_period = float(p.value)
                elif p.name == "tcp_watchdog_stale_sec":
                    self._tcp_wd_stale = float(p.value)
                elif p.name == "tcp_rx_buf_max_bytes":
                    self._tcp_rx_max = int(p.value)
                elif p.name == "tcp_rx_buf_keep_bytes":
                    self._tcp_rx_keep = int(p.value)

            # Single concise line so we can confirm runtime update worked.
            self.get_logger().info(
                f"param updated: gcur_reg={self._goal_cur_reg} gpos_reg={self._goal_pos_reg} "
                f"gpos_regs={self._goal_pos_regs} gpos_write={self._goal_pos_write_mode} "
                f"pcur_reg={self._present_current_reg} "
                f"ppos_reg={self._present_position_reg} ppos_regs={self._present_position_regs} "
                f"snap={self._drl_snap_enabled} "
                f"plc_fb={self._plc_feedback_enabled} plc(pos/cur/code)={self._plc_addr_pos}/{self._plc_addr_cur}/{self._plc_addr_code} "
                f"ack_to={self._tcp_ack_timeout}s wd={self._tcp_wd_enabled}"
            )

            # If TCP server reads depend on these params, reinject server so DRL-side regs match.
            if need_reinject_tcp and self._cmd_transport == "tcp" and (not self._tcp_external):
                self.get_logger().warn("TCP regs changed -> DRL TCP 서버 재주입/재접속을 수행합니다.")
                threading.Thread(target=self._reinject_tcp_server, daemon=True).start()
            return SetParametersResult(successful=True)
        except Exception as e:
            return SetParametersResult(successful=False, reason=str(e))

    def _tcp_watchdog_loop(self):
        """Background watchdog:
        - periodically sends ping (if connected)
        - if state/pong is stale, triggers reconnect (no reinject)
        """
        while rclpy.ok():
            try:
                time.sleep(max(0.2, self._tcp_wd_period))
                if not self._tcp_wd_enabled:
                    continue
                if self._cmd_transport != "tcp":
                    continue
                if self._tcp_external:
                    # external server: don't try to reinject/stop
                    pass
                if not self._socket_active or not self._sock:
                    continue
                # Avoid piling reconnect threads while a reconnect/reinject is in progress
                if self._tcp_reconnect_inflight:
                    continue

                # ping (best-effort)
                try:
                    self._send_frame({"type": "ping"})
                except Exception:
                    # connection likely dead
                    self._socket_active = False
                    continue

                now = time.time()
                last_rx = self._last_pong_rx_t or 0.0
                if self._tcp_state_stream_enabled:
                    last_rx = max(self._last_state_rx_t or 0.0, last_rx)
                if last_rx and (now - last_rx) > self._tcp_wd_stale:
                    self.get_logger().warn("TCP watchdog: state/pong stale -> reconnect 시도")
                    threading.Thread(target=self._reconnect_tcp_only, daemon=True).start()
            except Exception:
                continue

    def _reconnect_tcp_only(self):
        """Reconnect TCP client without reinjecting DRL code."""
        # single-flight guard
        if not self._tcp_reconnect_lock.acquire(blocking=False):
            return
        self._tcp_reconnect_inflight = True
        try:
            try:
                self._socket_active = False
                if self._sock:
                    try:
                        self._sock.close()
                    except Exception:
                        pass
                self._sock = None
            except Exception:
                pass
            # reset timestamps so watchdog doesn't immediately re-trigger
            self._last_state_rx_t = 0.0
            self._last_pong_rx_t = 0.0

            for attempt in range(10):
                try:
                    time.sleep(0.2 if attempt == 0 else 0.5)
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.settimeout(3.0)
                    sock.connect((self._robot_ip, self._tcp_port))
                    self._configure_tcp_socket(sock)
                    self._sock = sock
                    self._socket_active = True
                    self._tcp_rx_buf = b""
                    self._tcp_hello_seen = False
                    self._tcp_state_seen = False
                    self._tcp_ack_seen = False
                    self._tcp_sniff_logged = False
                    self._tcp_pong_seen = False
                    self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
                    self._recv_thread.start()
                    if self._tcp_handshake(timeout_sec=2.5):
                        self._last_pong_rx_t = time.time()
                        self.get_logger().info("TCP watchdog reconnect 성공 (handshake OK)")
                        return
                    # handshake failed -> close and retry
                    self.get_logger().warn(
                        f"TCP reconnect: handshake 실패 -> 재시도 ({attempt+1}/10)"
                    )
                    try:
                        self._socket_active = False
                        self._sock.close()
                    except Exception:
                        pass
                    self._sock = None
                except Exception:
                    continue
        finally:
            self._tcp_reconnect_inflight = False
            try:
                self._tcp_reconnect_lock.release()
            except Exception:
                pass

    def _reinject_tcp_server(self):
        """Reinject DRL TCP server with updated params and reconnect.

        This is needed because DRL_SERVER_CODE placeholders are resolved at injection time.
        """
        # single-flight guard (prevents overlap with watchdog reconnect)
        if not self._tcp_reconnect_lock.acquire(blocking=False):
            return
        self._tcp_reconnect_inflight = True
        # 1) stop client socket + recv loop
        try:
            # Best-effort: ask current server to stop via existing connection
            try:
                if self._sock and self._socket_active:
                    self._sock.sendall(struct.pack(">H", 4) + b"STOP")
            except Exception:
                pass
            self._socket_active = False
            if self._sock:
                try:
                    self._sock.close()
                except Exception:
                    pass
            self._sock = None
        except Exception:
            pass

        # reset timestamps so watchdog doesn't immediately re-trigger
        self._last_state_rx_t = 0.0
        self._last_pong_rx_t = 0.0

        # 2) try stop existing DRL tcp server
        try:
            killer = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            killer.settimeout(0.7)
            killer.connect((self._robot_ip, self._tcp_port))
            killer.sendall(b"STOP")
            killer.close()
            time.sleep(0.5)
        except Exception:
            pass

        # 3) reinject with latest params
        try:
            if not self._cli_drl.service_is_ready():
                return
            req = DrlStart.Request()
            req.robot_system = 0
            req.code = self._build_drl_server_code()
            fut = self._cli_drl.call_async(req)

            def _inj_done(_f):
                try:
                    res = _f.result()
                    ok = bool(res and res.success)
                except Exception as e:
                    ok = False
                    self.get_logger().error(f"DRL 서버 재주입 응답 예외: {e}")
                if ok:
                    self.get_logger().info("DRL 서버 재주입 응답: success=True")
                else:
                    self.get_logger().error("DRL 서버 재주입 응답: success=False")

            fut.add_done_callback(_inj_done)
        except Exception:
            return

        # 4) reconnect
        try:
            for attempt in range(10):
                try:
                    # Give controller some time to start the newly injected DRL server.
                    time.sleep(2.0 if attempt == 0 else 0.8)
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.settimeout(3.0)
                    sock.connect((self._robot_ip, self._tcp_port))
                    self._configure_tcp_socket(sock)
                    self._sock = sock
                    self._socket_active = True
                    self._tcp_rx_buf = b""
                    self._tcp_hello_seen = False
                    self._tcp_state_seen = False
                    self._tcp_ack_seen = False
                    self._tcp_sniff_logged = False
                    self._tcp_pong_seen = False
                    self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
                    self._recv_thread.start()
                    if self._tcp_handshake(timeout_sec=5.0):
                        self._last_pong_rx_t = time.time()
                        self.get_logger().info(
                            "TCP 재접속 성공 (DRL 서버 재주입 반영, handshake OK)"
                        )
                        return
                    # handshake failed -> close and retry
                    self.get_logger().warn(
                        f"TCP 재접속: handshake 실패 -> 재시도 ({attempt+1}/10)"
                    )
                    try:
                        self._socket_active = False
                        self._sock.close()
                    except Exception:
                        pass
                    self._sock = None
                except Exception:
                    continue
        finally:
            self._tcp_reconnect_inflight = False
            try:
                self._tcp_reconnect_lock.release()
            except Exception:
                pass

    def _call_drl(self, code: str, timeout_sec: float = 5.0) -> bool:
        if not self._cli_drl.service_is_ready():
            return False
        req = DrlStart.Request()
        req.robot_system = 0
        req.code = code
        event = threading.Event()
        ok = {"success": False}

        fut = self._cli_drl.call_async(req)

        def _done(_f):
            try:
                res = _f.result()
                ok["success"] = bool(res and res.success)
            except Exception:
                ok["success"] = False
            event.set()

        fut.add_done_callback(_done)
        event.wait(timeout=timeout_sec)
        return ok["success"]

    def _init_drl_server(self):
        self._init_timer.cancel()

        # 로봇 없이 TCP 서버만 테스트하는 경우
        if self._cmd_transport == "tcp" and self._tcp_external:
            self.get_logger().info("tcp_external_server=true: drl_start 주입 생략, 외부 TCP 서버로 바로 접속")
            for attempt in range(15):
                try:
                    time.sleep(0.2 if attempt == 0 else 0.5)
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.settimeout(3.0)
                    sock.connect((self._robot_ip, self._tcp_port))
                    self._configure_tcp_socket(sock)
                    self._sock = sock
                    self._socket_active = True
                    self.get_logger().info(f"TCP({self._tcp_port}) 접속 성공! (external server)")

                    self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
                    self._recv_thread.start()
                    return
                except Exception as e:
                    self.get_logger().warn(f"TCP 대기중 ({attempt+1}/15): {e}")

            self.get_logger().error("외부 TCP 서버 접속 실패")
            return

        if not self._cli_drl.service_is_ready():
            self._init_attempt = getattr(self, "_init_attempt", 0) + 1
            if self._init_attempt <= 60:
                self.get_logger().warn(
                    f"DRL 서비스 미준비 ({self._prefix}/drl/drl_start) "
                    f"— {self._init_attempt}/60, 2초 후 재시도"
                )
                self._init_timer = self.create_timer(
                    2.0, self._init_drl_server, callback_group=self._cb
                )
                return
            self.get_logger().error(f"DRL 서비스 연결 실패 ({self._prefix}/drl/drl_start)")
            return
        self._init_attempt = 0

        # tcp 모드: 이전에 남아있는 TCP DRL 서버가 있으면 프로토콜이 꼬일 수 있어 먼저 종료 시도
        if self._cmd_transport == "tcp":
            try:
                killer = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                killer.settimeout(0.7)
                killer.connect((self._robot_ip, self._tcp_port))
                # 구형/신형 DRL 모두 raw STOP을 인식하도록 되어 있음
                killer.sendall(b"STOP")
                killer.close()
                time.sleep(0.5)
                self.get_logger().info("기존 TCP 서버 종료(STOP) 시도 완료")
            except Exception:
                pass

        # command_transport=drl 인 경우: 토크 ON + 기본 전류 설정을 먼저 1회 수행
        if self._cmd_transport == "drl":
            try:
                init_pkts = [
                    ModbusRTU.fc06(self._slave_id, 256, 1),  # TORQUE_ENABLE
                    ModbusRTU.fc06(self._slave_id, 275, self._cur_init),  # GOAL_CURRENT
                ]
                if not self._call_drl(build_drl_write_packets(init_pkts), timeout_sec=5.0):
                    self.get_logger().error("DRL 직접 초기화 실패(토크ON/기본전류)")
                else:
                    self.get_logger().info("DRL 직접 초기화 완료(토크ON/기본전류)")
            except Exception as e:
                self.get_logger().error(f"DRL 직접 초기화 예외: {e}")

        # command_transport=drl 인 경우엔 굳이 무한루프 TCP 서버를 DRL에 올릴 필요가 없음
        if self._cmd_transport == "tcp":
            req = DrlStart.Request()
            req.robot_system = 0
            req.code = self._build_drl_server_code()
            self._cli_drl.call_async(req)  # 무한루프 DRL이라 결과를 기다리지 않음
            self.get_logger().info("DRL 서버 코드 배포 완료! 소켓 접속 대기...")
        else:
            self.get_logger().info("command_transport=drl: DRL 서버 주입 생략 (명령은 drl_start로 직접 실행)")

        if self._cmd_transport == "tcp":
            for attempt in range(15):
                try:
                    time.sleep(1.5 if attempt == 0 else 1.0)  # 첫 시도는 DRL 부팅 시간 대기
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.settimeout(3.0)
                    sock.connect((self._robot_ip, self._tcp_port))
                    self._configure_tcp_socket(sock)
                    self._sock = sock
                    self._socket_active = True
                    self._tcp_rx_buf = b""
                    self._tcp_hello_seen = False
                    self._tcp_state_seen = False
                    self._tcp_ack_seen = False
                    self._tcp_sniff_logged = False
                    self._tcp_pong_seen = False
                    self.get_logger().info(f"TCP({self._tcp_port}) 접속 성공! 실시간 피드백 활성화!")

                    self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
                    self._recv_thread.start()

                    # Protocol sanity check (ping/pong)
                    if not self._tcp_handshake(timeout_sec=5.0):
                        self.get_logger().error("TCP 핸드셰이크 실패(ping/pong). 서버가 응답하지 않아 재시도합니다.")
                        self._socket_active = False
                        try:
                            self._sock.close()
                        except Exception:
                            pass
                        self._sock = None
                        continue
                    return
                except Exception as e:
                    self.get_logger().warn(f"TCP 대기중 ({attempt+1}/15): {e}")

            self.get_logger().error("TCP 소켓 확립 실패! 로봇 네트워크 확인 요망")

    def _recv_loop(self):
        """실시간 피드백 수신 루프"""
        while self._socket_active and rclpy.ok():
            try:
                chunk = self._sock.recv(512)
                if not chunk:
                    self.get_logger().warn("소켓 연결 끊김 (빈 수신)")
                    break
                self._tcp_rx_buf += chunk
                # If controller spams state and our loop can't keep up, drop old bytes.
                if self._tcp_rx_max > 0 and len(self._tcp_rx_buf) > self._tcp_rx_max:
                    keep = max(1024, self._tcp_rx_keep)
                    self._tcp_rx_buf = self._tcp_rx_buf[-keep:]
                    # framing may be broken in the kept tail; wait for next good frame
                    self.get_logger().warn("TCP rx buffer overflow -> keeping only latest tail")
                if not self._tcp_sniff_logged and len(self._tcp_rx_buf) > 0:
                    # 서버가 framed JSON을 보내는지 확인하기 위한 1회 스니핑 로그
                    sniff = self._tcp_rx_buf[:32]
                    self.get_logger().info(
                        f"TCP 초기 수신 스니핑({len(sniff)}B): hex={sniff.hex()} ascii={sniff!r}"
                    )
                    self._tcp_sniff_logged = True

                last_state_msg = None
                last_state_bin = None  # (cur, pos, gcur, gpos_lo, gpos_hi)

                if self._tcp_protocol == "binary":
                    while True:
                        parsed = self._bin_try_parse_one()
                        if not parsed:
                            break
                        mtype, seq, payload = parsed
                        if mtype == TCP_T_PONG:
                            self._tcp_pong_seen = True
                            self._tcp_hello_seen = True
                            self._last_pong_rx_t = time.time()
                            self.get_logger().info("TCP pong 수신 (프로토콜 OK)")
                        elif mtype == TCP_T_ACK:
                            ok = bool(payload[0]) if len(payload) >= 1 else False
                            err = ""
                            if (not ok) and len(payload) >= 3:
                                elen = (payload[1] << 8) | payload[2]
                                if elen > 0 and len(payload) >= 3 + elen:
                                    try:
                                        err = payload[3 : 3 + elen].decode("utf-8", errors="ignore")
                                    except Exception:
                                        err = ""
                            msg = {"type": "ack", "id": int(seq), "ok": ok, "err": err}
                            with self._ack_lock:
                                self._ack_results[int(seq)] = msg
                                ev = self._ack_waiters.get(int(seq))
                            if ev:
                                ev.set()
                            if not self._tcp_ack_seen:
                                self._tcp_ack_seen = True
                                self.get_logger().info("TCP ack 수신 시작")
                        elif mtype == TCP_T_STATE:
                            # payload: cur(i16), pos(i32), gcur(i16), gpos_lo(u16), gpos_hi(u16)
                            if len(payload) >= struct.calcsize(">hi hHH"):
                                try:
                                    cur, pos, gcur, gpos_lo, gpos_hi = struct.unpack(">hi hHH", payload[: struct.calcsize(">hi hHH")])
                                    last_state_bin = (int(cur), int(pos), int(gcur), int(gpos_lo), int(gpos_hi))
                                except Exception:
                                    pass
                else:
                    while len(self._tcp_rx_buf) >= 2:
                        n = (self._tcp_rx_buf[0] << 8) | self._tcp_rx_buf[1]
                        if len(self._tcp_rx_buf) < 2 + n:
                            break
                        payload = self._tcp_rx_buf[2 : 2 + n]
                        self._tcp_rx_buf = self._tcp_rx_buf[2 + n :]

                        try:
                            msg = json.loads(payload.decode("utf-8", errors="ignore"))
                        except Exception:
                            continue

                        mtype = msg.get("type", "")
                        if mtype == "state":
                            last_state_msg = msg
                        elif mtype == "hello":
                            self._tcp_hello_seen = True
                            self.get_logger().info(f"TCP hello 수신: {msg}")
                        elif mtype == "pong":
                            self._tcp_pong_seen = True
                            self._tcp_hello_seen = True
                            self._last_pong_rx_t = time.time()
                            self.get_logger().info("TCP pong 수신 (프로토콜 OK)")
                        elif mtype == "ack":
                            cmd_id = int(msg.get("id", 0))
                            with self._ack_lock:
                                self._ack_results[cmd_id] = msg
                                ev = self._ack_waiters.get(cmd_id)
                            if ev:
                                ev.set()
                            if not self._tcp_ack_seen:
                                self._tcp_ack_seen = True
                                self.get_logger().info("TCP ack 수신 시작")
                        elif mtype == "snap":
                            if "snap" in msg and isinstance(msg.get("snap"), dict):
                                self.get_logger().info(f"SNAP(270-290): {msg.get('snap')}")

                # Apply only the latest state in this batch (reduces latency under spam)
                if last_state_bin is not None:
                    cur, raw_pos, gcur, gpos_lo, gpos_hi = last_state_bin
                    self._current_hz_cur = int(cur)
                    # 32-bit pos를 16-bit/워드순서 옵션으로 보정
                    if self._pos_use_low:
                        raw_pos = raw_pos & 0xFFFF
                    elif self._pos_word_order == "lo_hi":
                        raw_pos = ((raw_pos & 0xFFFF) << 16) | ((raw_pos >> 16) & 0xFFFF)
                    if self._pos_scale and self._pos_scale != 1.0:
                        self._current_hz_pos = int(round(float(raw_pos) / self._pos_scale))
                    else:
                        self._current_hz_pos = int(raw_pos)
                    self._last_state_rx_t = time.time()
                    if not self._tcp_state_seen:
                        self._tcp_state_seen = True
                        self.get_logger().info("TCP state 수신 시작")
                    self._last_gpos_lo = int(gpos_lo)
                    # concise dbg
                    self.get_logger().info(
                        f"DBG cur(reg{self._present_current_reg})={self._current_hz_cur} "
                        f"gcur(reg{self._goal_cur_reg})={gcur} "
                        f"gpos_lo(reg{self._goal_pos_reg})={gpos_lo} gpos_hi(reg{self._goal_pos_reg}+1)={gpos_hi}"
                    )
                elif last_state_msg is not None:
                    msg = last_state_msg
                    self._current_hz_cur = msg.get("cur", 0)
                    raw_pos = int(msg.get("pos", 0))
                    # 32-bit pos를 16-bit/워드순서 옵션으로 보정
                    if self._pos_use_low:
                        raw_pos = raw_pos & 0xFFFF
                    elif self._pos_word_order == "lo_hi":
                        raw_pos = ((raw_pos & 0xFFFF) << 16) | ((raw_pos >> 16) & 0xFFFF)

                    if self._pos_scale and self._pos_scale != 1.0:
                        self._current_hz_pos = int(round(float(raw_pos) / self._pos_scale))
                    else:
                        self._current_hz_pos = int(raw_pos)
                    self._last_state_rx_t = time.time()
                    self._tcp_hello_seen = True
                    if not self._tcp_state_seen:
                        self._tcp_state_seen = True
                        self.get_logger().info("TCP state 수신 시작")
                    if "gcur" in msg:
                        srv_pcur = msg.get("pcur_reg", "?")
                        srv_gcur = msg.get("gcur_reg", "?")
                        srv_gpos = msg.get("gpos_reg", "?")
                        self.get_logger().info(
                            f"DBG cur(reg{srv_pcur})={self._current_hz_cur} "
                            f"gcur(reg{srv_gcur})={msg.get('gcur')} "
                            f"gpos_lo(reg{srv_gpos})={msg.get('gpos_lo')} gpos_hi(reg{srv_gpos}+1)={msg.get('gpos_hi')}"
                        )
                        if "gpos_lo" in msg:
                            try:
                                self._last_gpos_lo = int(msg.get("gpos_lo"))
                            except Exception:
                                pass

            except socket.timeout:
                continue
            except OSError as e:
                # If we intentionally closed the socket (reinject/reconnect), don't spam errors.
                if not self._socket_active:
                    break
                self.get_logger().error(f"소켓 수신 에러: {e}")
                break
            except Exception as e:
                self.get_logger().error(f"소켓 수신 에러: {e}")
                break
        self._socket_active = False

    def _send_frame(self, obj: dict) -> None:
        if self._tcp_protocol == "binary":
            # Only used for ping in binary mode.
            t = str(obj.get("type", "")).strip().lower()
            if t == "ping":
                self._send_bin(TCP_T_PING, 0, b"")
                return
            return
        payload = json.dumps(obj).encode("utf-8")
        header = struct.pack(">H", len(payload))
        self._sock.sendall(header + payload)

    def _send_cmd_fire_and_forget(
        self, frames: list[bytes], timeout_sec: float | None = None
    ) -> tuple[bool, str]:
        del timeout_sec
        if not self._socket_active:
            return False, "tcp offline"
        try:
            cmd_id = self._next_cmd_id
            self._next_cmd_id += 1
            if self._tcp_protocol == "binary":
                parts = [struct.pack(">H", len(frames))]
                for b in frames:
                    parts.append(struct.pack(">H", len(b)))
                    parts.append(b)
                payload = b"".join(parts)
                self._send_bin(TCP_T_CMD, cmd_id, payload)
            else:
                msg = {"type": "cmd", "id": cmd_id, "frames": [b.hex() for b in frames]}
                self._send_frame(msg)
            return True, ""
        except Exception as e:
            return False, f"send failed: {e}"

    def _send_cmd_and_wait_ack(self, frames: list[bytes], timeout_sec: float | None = None) -> tuple[bool, str]:
        # 서버가 framed JSON 프로토콜을 실제로 말하는지(hello/state/pong) 확인.
        # 재주입/재접속 직후에는 recv_loop가 아직 hello/state를 못 받은 타이밍이 있어
        # 여기서 1회 ping/pong 핸드셰이크를 강제 시도해 레이스를 줄인다.
        if not (self._tcp_hello_seen or self._tcp_state_seen or self._tcp_pong_seen):
            if not self._tcp_handshake(timeout_sec=1.5):
                return False, "no hello/state/pong from server"

        cmd_id = self._next_cmd_id
        self._next_cmd_id += 1

        ev = threading.Event()
        with self._ack_lock:
            self._ack_waiters[cmd_id] = ev
            self._ack_results.pop(cmd_id, None)

        try:
            if self._tcp_protocol == "binary":
                # payload: [num_frames u16] + (len u16 + frame bytes)...
                parts = [struct.pack(">H", len(frames))]
                for b in frames:
                    parts.append(struct.pack(">H", len(b)))
                    parts.append(b)
                payload = b"".join(parts)
                self._send_bin(TCP_T_CMD, cmd_id, payload)
            else:
                msg = {"type": "cmd", "id": cmd_id, "frames": [b.hex() for b in frames]}
                self._send_frame(msg)
        except Exception as e:
            with self._ack_lock:
                self._ack_waiters.pop(cmd_id, None)
            return False, f"send failed: {e}"

        to = float(timeout_sec) if timeout_sec is not None else float(self._tcp_ack_timeout)
        if not ev.wait(timeout=to):
            # Grace window: controller may deliver ACK slightly late under load/reinject.
            grace_deadline = time.time() + 1.0
            while time.time() < grace_deadline:
                with self._ack_lock:
                    late = self._ack_results.get(cmd_id)
                if late is not None:
                    ev.set()
                    break
                time.sleep(0.02)
            if not ev.is_set():
                with self._ack_lock:
                    self._ack_waiters.pop(cmd_id, None)
                return False, "ack timeout"

        with self._ack_lock:
            self._ack_waiters.pop(cmd_id, None)
            ack = self._ack_results.pop(cmd_id, None) or {}

        if bool(ack.get("ok", False)):
            return True, ""
        return False, str(ack.get("err", "unknown"))

    def _tcp_handshake(self, timeout_sec: float = 5.0) -> bool:
        """Connect 직후 서버가 우리 프레이밍을 이해하는지 ping/pong로 확인."""
        self._tcp_pong_seen = False
        try:
            if self._tcp_protocol == "binary":
                self._send_bin(TCP_T_PING, 0, b"")
            else:
                self._send_frame({"type": "ping"})
        except Exception:
            return False

        start = time.time()
        while time.time() - start < timeout_sec:
            if self._tcp_pong_seen or self._tcp_hello_seen or self._tcp_state_seen:
                return True
            time.sleep(0.05)
        return False

    def _goal_callback(self, goal_request):
        if self._executing:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def _resolve_preset(self, req):
        a = req.action
        if a in ("open", "release"):   return self._pulse_open, self._cur_init
        if a == "grab_cube":           return self._pulse_cube, self._cur_cube
        if a == "grab_rotate":         return self._pulse_rotate, self._cur_cube
        if a == "grab_repose":         return self._pulse_repose, self._cur_cube
        if a == "custom":
            return (req.pulse if req.pulse >= 0 else self._pulse_cube,
                    req.current if req.current > 0 else self._cur_init)
        return -1, -1

    def _execute_callback(self, goal_handle):
        self._executing = True
        try:
            req = goal_handle.request
            t_pulse, t_cur = self._resolve_preset(req)

            if t_pulse == -1:
                goal_handle.abort()
                return GripperCommand.Result(success=False, message="유효하지 않은 동작")

            # Modbus 바이너리 패킷 생성
            cur_pkt = ModbusRTU.fc06(self._slave_id, self._goal_cur_reg, t_cur)
            goal_val = int(round(float(t_pulse) * (self._goal_pos_scale if self._goal_pos_scale else 1.0)))
            # Goal position write packets (fc06 / fc16)
            pos_pkt_fc06 = ModbusRTU.fc06(self._slave_id, self._goal_pos_reg, int(goal_val) & 0xFFFF)
            pos_pkt_fc16 = ModbusRTU.fc16(self._slave_id, self._goal_pos_reg, 2, [int(goal_val) & 0xFFFF, 0])

            if self._cmd_transport == "drl":
                # DRL에서 move + present readback polling까지 수행 (되던 방식과 동일)
                drl_code = build_drl_move_and_poll(
                    slave_id=self._slave_id,
                    target_pulse=int(t_pulse),
                    target_current=int(t_cur),
                    grip_current_threshold=int(self._grip_threshold),
                    pos_tolerance=20,
                    max_loops=60,
                )
                ok = self._call_drl(drl_code, timeout_sec=15.0)
                if not ok:
                    goal_handle.abort()
                    return GripperCommand.Result(success=False, message="drl_start 명령 전송 실패")
                self.get_logger().info(f"DRL 직접 실행(명령+폴링): pos={t_pulse}, cur={t_cur}")

                result = GripperCommand.Result()
                result.success = True
                result.gripped = False
                result.final_position = int(t_pulse)
                result.final_current = int(t_cur)
                result.message = "완료(drl move+poll)"
                goal_handle.succeed()
                return result
            else:
                if not self._socket_active:
                    goal_handle.abort()
                    return GripperCommand.Result(success=False, message="TCP 오프라인")
                try:
                    ok1, err1 = self._send_cmd_and_wait_ack([cur_pkt], timeout_sec=None)
                    if not ok1:
                        raise RuntimeError(f"cur ack failed: {err1}")
                    time.sleep(0.05)
                    # Position write mode (auto fallback fc06 -> fc16)
                    mode = self._goal_pos_write_mode
                    regs = self._goal_pos_regs
                    if regs <= 1:
                        ok2, err2 = self._send_cmd_and_wait_ack([pos_pkt_fc06], timeout_sec=None)
                        if not ok2:
                            raise RuntimeError(f"pos(fc06) ack failed: {err2}")
                    else:
                        sent = False
                        if mode in ("fc06", "auto"):
                            before = self._last_gpos_lo
                            ok2, err2 = self._send_cmd_and_wait_ack([pos_pkt_fc06], timeout_sec=None)
                            if ok2:
                                # small window for state to reflect gpos change
                                t0 = time.time()
                                while time.time() - t0 < 0.8:
                                    time.sleep(0.05)
                                    if self._last_gpos_lo is not None and self._last_gpos_lo != before:
                                        sent = True
                                        break
                                if mode == "fc06" and not sent:
                                    sent = True  # sent anyway
                            elif mode == "fc06":
                                raise RuntimeError(f"pos(fc06) ack failed: {err2}")
                        if (not sent) and mode in ("fc16", "auto"):
                            ok3, err3 = self._send_cmd_and_wait_ack([pos_pkt_fc16], timeout_sec=None)
                            if not ok3:
                                raise RuntimeError(f"pos(fc16) ack failed: {err3}")
                            sent = True
                        if not sent:
                            raise RuntimeError("pos write failed")
                    self.get_logger().info(f"제어 명령 전송(TCP+ACK): pos={t_pulse}, cur={t_cur}")
                except Exception as e:
                    self.get_logger().error(f"소켓 송신 실패: {e}")
                    # ACK 타임아웃/논리 에러는 연결 문제와 다를 수 있어 소켓을 바로 죽이지 않음
                    # (실제 소켓 에러/끊김은 recv_loop가 _socket_active=False로 만들 것)
                    goal_handle.abort()
                    return GripperCommand.Result(success=False, message="소켓 송신 에러")

            # 대기 + 피드백 루프
            start_t = time.time()
            gripped = False
            start_pos = self._current_hz_pos
            reached = False
            is_grip_action = str(req.action) not in ("open", "release")

            max_wait = self._action_max_wait
            while (elapsed := time.time() - start_t) < max_wait:
                time.sleep(0.1)
                cur_now = self._current_hz_cur
                pos_now = self._current_hz_pos

                fb = GripperCommand.Feedback()
                fb.phase = "moving"
                fb.current_position = pos_now
                fb.target_position = t_pulse
                fb.current_load = cur_now
                fb.progress = min(1.0, elapsed / max_wait)
                goal_handle.publish_feedback(fb)

                if elapsed > 0.5:
                    # 상태 수신이 한동안 없다면(초기 연결/일시 정체) 조금 더 기다림
                    if self._last_state_rx_t and (time.time() - self._last_state_rx_t) > 2.0:
                        continue
                    # 전류로 파지(gripped) 판정:
                    # - open/release는 파지 판정을 하면 안 됨
                    # - custom도 사용자 의도에 따라 "잡기" 동작일 수 있으므로 포함
                    if self._grip_enabled and is_grip_action and abs(cur_now) > self._grip_threshold:
                        gripped = True
                        self.get_logger().info(f"파지 감지! 전류={cur_now}mA")
                        break
                    if abs(pos_now - t_pulse) < self._done_tol:
                        reached = True
                        self.get_logger().info(f"위치 도달! pos={pos_now}")
                        break

            result = GripperCommand.Result()
            moved = abs(self._current_hz_pos - start_pos) >= self._done_min_motion
            if self._done_require_reached:
                # For grip actions, "gripped" can be a valid terminal condition even if
                # position didn't reach (object contact before target).
                result.success = bool(reached or (gripped and is_grip_action))
            else:
                result.success = True
            result.gripped = gripped
            result.final_position = self._current_hz_pos
            result.final_current = self._current_hz_cur
            if result.success:
                result.message = "완료"
                goal_handle.succeed()
            else:
                result.message = "목표 미도달"
                goal_handle.abort()
            return result

        finally:
            self._executing = False

    def _publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["gripper_joint"]
        msg.position = [float(self._current_hz_pos)]
        msg.velocity = [0.0]
        msg.effort = [float(self._current_hz_cur)]
        self._state_pub.publish(msg)

    def destroy_node(self):
        if self._sock and self._socket_active:
            try:
                # framed STOP
                self._sock.sendall(struct.pack(">H", 4) + b"STOP")
                self._sock.close()
            except:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
