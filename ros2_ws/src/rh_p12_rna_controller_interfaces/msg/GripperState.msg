std_msgs/Header header

# Current feedback (scaled like /gripper/state)
int32 position
int32 current

# Last commanded goal (best-effort; may be unknown depending on transport)
int32 goal_position
int32 goal_current

# Status text for operators/debug
string status_text

