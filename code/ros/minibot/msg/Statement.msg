int64 uid
string name

# a statement can be a waypoint, a comment or a wait operation
uint8 STATEMENT_TYPE_WAYPOINT=2
uint8 STATEMENT_TYPE_COMMENT=3
uint8 STATEMENT_TYPE_WAIT=4
uint8 type

# Waypoint:
uint16 pose_uid
sensor_msgs/JointState jointState
geometry_msgs/Pose pose
bool cartesic_path
bool collision_check
bool improved_path
# Comment
string comment

# Wait
uint8 WAIT_TYPE_NOWAIT=1
uint8 WAIT_TYPE_WAIT=2
uint8 WAIT_TYPE_CONFIRMATION=3
uint8 waitType
duration kitkat




