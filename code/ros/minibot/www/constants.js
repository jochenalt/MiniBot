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
		ACTION_SELECT_LOCAL_PLAN : 3,	    // plan and display the path		
		ACTION_CLEAR_PLAN : 4,				// clear the last plan		
		ACTION_SIMULATE_PLAN : 5,			// simulate the last plan
		ACTION_GLOBAL_PLAN : 6,				// create a plan of the entire programm
		ACTION_VIS_GLOBAL_PLAN : 7,			// visualize the global plan 
		ACTION_VIS_LOCAL_PLAN : 8,			// visualize the current local plan
		ACTION_STEP_FORWARD : 9,			// visualize the current local plan

		ACTIVATE_STATEMENT : 1
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


Constants.Themes = {
 		Name : [ "cerulan", "cosmo", "cyborg", "darkly",  "flatly",  "journal", "litera", "lumen","lux","materia","pulse","sandstone", "simplex", "slate", "solar", "spacelab","superhero","united","yeti" ],
		Description : [ "A calm blue sky", "An ode to Metro", "Jet black", "Night mode", "Flat and modern", "Crsip like paper", "Medium is the message", "Light and shadow", "Touch of glass", "Material is a metaphor", "A trave of purple", "Touch of warmth", "Minimalist", "Shades of gunmetal", "Spin on Solarized", "Slivery and sleek","The brave and the Blue", "Ubuntu", "Yeti" ],
		BackgroundColor :  ["#FFFFFF", "#F1F3F5", "#222222" , "#ADB5BD", "#ECF0F1", "#FFFFFF" , "#F8F9FA", "#FFFFFF", "#FFFFFF" , "#FFFFFF", "#F9F8FC", "#F8F5F0" , "#FFFFFF", "#ECF0F1", "#FDF6E3", "#EEEEEE", "#ABB6C2", "#E9ECEF", "#EEEEEE"  ]
}


// Constants.Themes = {
//		Name : [ "cerulan", "cyborg", "darkly",  "sandstone", "simplex", "slate" ,"solar"],
//		Description : [ "A calm blue sky",  "Jet black", "Night mode", "Touch of warmth", "Minimalist", "Shades of gunmetal","Spin on Solarized" ],
//		BackgroundColor :  ["#F7F8F9",  "#222222" , "#ADB5BD",  "#F8F5F0" , "#FFFFFF", "#ECF0F1","FDF6E3"]
//}
