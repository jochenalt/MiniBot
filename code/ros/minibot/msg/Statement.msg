int64 id
string name

# a statement can be a waypoint, a comment or a wait operation
uint8 STATEMENT_TYPE_WAYPOINT=0
uint8 STATEMENT_TYPE_COMMENT=1
uint8 STATEMENT_TYPE_WAIT=2
uint8 type

# Waypoint:
sensor_msgs/JointState jointState
geometry_msgs/Pose pose

# Comment
string comment

# Wait
uint8 WAIT_TYPE_WAIT=0
uint8 WAIT_TYPE_CONFIRMATION=1
uint8 WAIT_TYPE_NOWAIT=1
duration kitkat




