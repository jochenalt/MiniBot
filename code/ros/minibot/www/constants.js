/**
 * @author Jochen Alt
 */



var Constants = {};

Constants.Kinematics = {
		MAX_KINEMATICS_RATE : 10, // [Hz] max rate the UI sends new tcp or joints to the server
		BLOCK_UI_INPUT_TIME : 500 // [ms] As long as the sliders are moved, input from kinematic is blocked for this duration 
};

// constants from message PlannigAction
Constants.PlanningAction = {
		FORWARD :0,
		BACKWARD: 1,
		RUN:2,
		SELECT_LOCAL_PLAN : 3,	    // plan and display the path		
		CLEAR_PLAN : 4,				// clear the last plan		
		SIMULATE_LOCAL_PLAN : 5,			// simulate the last plan
		GLOBAL_PLAN : 6,			// create a plan of the entire programm
		VIS_GLOBAL_PLAN : 7,		// visualize the global plan 
		VIS_LOCAL_PLAN : 8,			// visualize the current local plan
		STEP_FORWARD : 9			// visualize the current local plan
};

Constants.Planning = {
		ARM_GROUP : 'minibot_arm',  		// group of links as defined in SRDF defining the arm without the endeffector
		MINIBOT_GROUP : 'minibot',  		// group of linksof the endeffector
		GRIPPER_GROUP : 'minibot_gripper',  // group of linksof the endeffector
		FIXED_FRAME : 'base_link', 			// first link 
};

// constants from message type Database
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

// constants from message type ErrorCodes
Constants.ErrorCodes = {
		SUCCESS : 1,
		FAILURE : 99999
}

// constants from message type Statements
Constants.Statement = {
	STATEMENT_TYPE_NONE: 0,
   	STATEMENT_TYPE_MOVEMENT: 1,
    STATEMENT_TYPE_WAYPOINT: 2,
    STATEMENT_TYPE_COMMENT: 3,
    STATEMENT_TYPE_WAIT: 4,
	PLAN_SPACE_STRATEGY:0,
	PLAN_CARTESIC_STRATEGY:1,
	PLAN_SPLINE_STRATEGY:2,    
	WAIT_TYPE_NOWAIT:1,
	WAIT_TYPE_WAIT:2,
	WAIT_TYPE_CONFIRMATION:3
};

Constants.Themes = {
 		Name : [ "cerulan", "cosmo", "cyborg", "darkly",  "flatly",  "journal", "litera", "lumen","lux","materia","pulse","sandstone", "simplex", "slate", "solar", "spacelab","superhero","united","yeti" ],
		Description : [ "A calm blue sky", "An ode to Metro", "Jet black", "Night mode", "Flat and modern", "Crsip like paper", "Medium is the message", "Light and shadow", "Touch of glass", "Material is a metaphor", "A trave of purple", "Touch of warmth", "Minimalist", "Shades of gunmetal", "Spin on Solarized", "Slivery and sleek","The brave and the Blue", "Ubuntu", "Yeti" ],
		BackgroundColor :  ["#FFFFFF", "#F1F3F5", "#222222" , "#ADB5BD", "#ECF0F1", "#FFFFFF" , "#F8F9FA", "#FFFFFF", "#FFFFFF" , "#FFFFFF", "#F9F8FC", "#F8F5F0" , "#FFFFFF", "#ECF0F1", "#FDF6E3", "#EEEEEE", "#ABB6C2", "#E9ECEF", "#EEEEEE"  ]
}
