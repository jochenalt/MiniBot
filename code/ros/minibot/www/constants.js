/**
 * @author Jochen Alt
 */


var ErrorCode =  {};
//	MoveIT : {

ErrorCode.MOVEIT = {
		SUCCESS : 1,
		FAILURE : 99999,
		PLANNING_FAILED : -1,
		INVALID_MOTION_PLAN : -2,
		MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE : -3,
		CONTROL_FAILED : -4,
		UNABLE_TO_AQUIRE_SENSOR_DATA : -5,
		TIMED_OUT : -6,
		PREEMPTED : -7,
		START_STATE_IN_COLLISION : -10,
		START_STATE_VIOLATES_PATH_CONSTRAINTS : -11,
		GOAL_IN_COLLISION : -12,
		GOAL_VIOLATES_PATH_CONSTRAINTS : -13,
		GOAL_CONSTRAINTS_VIOLATED : -14,
		INVALID_GROUP_NAME : -15,
		INVALID_GOAL_CONSTRAINTS : -16,
		INVALID_ROBOT_STATE : -17,
		INVALID_LINK_NAME : -18,
		INVALID_OBJECT_NAME : -19,
		FRAME_TRANSFORM_FAILURE : -21,
		COLLISION_CHECKING_UNAVAILABLE : -22,
		ROBOT_STATE_STALE : -23,
		SENSOR_INFO_STALE : -24,
		NO_IK_SOLUTION : -31
}

ErrorCode.PLANNING = {
		SUCCESS : 1,
		FAILURE : 99999
}

var Constants = {};
Constants.Kinematics = {
		MAX_KINEMATICS_RATE : 10, // [Hz] max rate the UI sends new tcp or joints to the server
		BLOCK_UI_INPUT_TIME : 500 // [ms] As long as the sliders are moved, input from kinematic is blocked for this duration 
};

Constants.Planning = {
		ARM_GROUP : 'minibot_arm',  		// group of links as defined in SRDF defining the arm without the endeffector
		MINIBOT_GROUP : 'minibot',  // group of linksof the endeffector
		GRIPPER_GROUP : 'minibot_gripper',  // group of linksof the endeffector
		FIXED_FRAME : 'base_link', 			// first link 
		ACTION_PLAN_PATH : 3,				// plan and display the path		
		ACTION_CLEAR_PLAN : 4,				// clear the last plan		
		ACTION_SIMULATE_PLAN : 5			// simulate the last plan
};

Constants.Database = {
		READ_POSES : 0,
		READ_PROGRAMME : 1,
		WRITE_POSES : 2,
		WRITE_PROGRAMME :3,
		READ_CONFIGURATIION : 4,
		WRITE_CONFIGURATIION : 5,

		ANGLE_UNIT_RAD : 0,
		ANGLE_UNIT_GRAD : 1
};