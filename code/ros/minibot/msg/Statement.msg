int64 uid
string name

# a statement can be a waypoint, a comment or a wait operation
uint8 STATEMENT_TYPE_WAYPOINT=2
uint8 STATEMENT_TYPE_COMMENT=3
uint8 STATEMENT_TYPE_WAIT=4
uint8 type

# Waypoint:
uint16 pose_uid
uint8 PLAN_SPACE_STRATEGY=0
uint8 PLAN_CARTESIC_STRATEGY=1
uint8 PLAN_SPLINE_STRATEGY=2
uint8 path_strategy
bool collision_check
# Comment
string comment

# Wait
uint8 WAIT_TYPE_NOWAIT=1
uint8 WAIT_TYPE_WAIT=2
uint8 WAIT_TYPE_CONFIRMATION=3
uint8 waitType
duration kitkat




