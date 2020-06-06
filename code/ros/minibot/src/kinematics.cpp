

#include "ros/ros.h"

#define LOG_NAME "kinematics"
#include "kinematics.h"


std::vector<std::string> minibot_arm_joint_names;


#include <stdio.h>
#include <stdlib.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

#include "utils.h"

#define IKFAST_HAS_LIBRARY 	// Build IKFast with API functions
#define IKFAST_NO_MAIN 		// Don't include main() from IKFast

namespace ikfast {
#include "../src/minibot_minibot_arm_ikfast_solver.cpp"
}


float SIGN(float x) {
    return (x >= 0.0f) ? +1.0f : -1.0f;
}

float NORM(float a, float b, float c, float d) {
    return sqrt(a * a + b * b + c * c + d * d);
}

#define IKREAL_TYPE ikfast::IkReal

namespace Minibot {
namespace Kinematics {

void init() {
  ROS_INFO_STREAM_NAMED(LOG_NAME, "module kinematics init");

  // cache the joint names of the kinematics group
  robot_model::RobotModelPtr kinematic_model = Utils::getRobotModel();
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  const moveit::core::JointModelGroup* jmg = kinematic_state->getJointModelGroup(minibot_arm_group_name);
  if (jmg == NULL) {
      ROS_ERROR_STREAM("Minibot::Kinematics::init: did not find joint model group" << minibot_arm_group_name);
  }
  minibot_arm_joint_names = jmg->getActiveJointModelNames();

}


// computes all IK solutions of a given pose and returns those in solutions
// returns true if success
bool compute_ik(const geometry_msgs::Pose& pose, std::vector<sensor_msgs::JointState>& solutions) {
    // create a local state
    robot_model::RobotModelPtr kinematic_model = Utils::getRobotModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));

    IKREAL_TYPE eerot[9],eetrans[3];
    unsigned int numOfJoints = ikfast::GetNumJoints();

    // should be 0 on a 6DOF robot
    unsigned int num_free_parameters = ikfast::GetNumFreeParameters();

    ikfast::IkSolutionList<IKREAL_TYPE> ik_solutions;

    eetrans[0] = pose.position.x;
    eetrans[1] = pose.position.y;
    eetrans[2] = pose.position.z;

    double qw = pose.orientation.w;
    double qx = pose.orientation.x;
    double qy = pose.orientation.y;
    double qz = pose.orientation.z;

    // Convert input effector pose, in w x y z quaternion notation, to rotation matrix.
    const double n = 1.0f/sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
    qw *= n;
    qx *= n;
    qy *= n;
    qz *= n;
    eerot[0] = 1.0f - 2.0f*qy*qy - 2.0f*qz*qz;  eerot[1] = 2.0f*qx*qy - 2.0f*qz*qw;         eerot[2] = 2.0f*qx*qz + 2.0f*qy*qw;
    eerot[3] = 2.0f*qx*qy + 2.0f*qz*qw;         eerot[4] = 1.0f - 2.0f*qx*qx - 2.0f*qz*qz;  eerot[5] = 2.0f*qy*qz - 2.0f*qx*qw;
    eerot[6] = 2.0f*qx*qz - 2.0f*qy*qw;         eerot[7] = 2.0f*qy*qz + 2.0f*qx*qw;         eerot[8] = 1.0f - 2.0f*qx*qx - 2.0f*qy*qy;

    bool success = ikfast::ComputeIk(eetrans, eerot, NULL, ik_solutions);
    if( !success )
	return false;

    unsigned int num_of_solutions = (int)ik_solutions.GetNumSolutions();
    ROS_DEBUG_STREAM_NAMED(LOG_NAME, "compute_ik(pos=(x=" << pose.position.x << " y=" << pose.position.y << " z=" << pose.position.z << ")"
			   << " orientation=(x=" << pose.orientation.x << " y=" << pose.orientation.y << " z=" << pose.orientation.z << " w=" << pose.orientation.w <<") "
			   << num_of_solutions << " solutions.\n");
    std::vector<IKREAL_TYPE> solValues(numOfJoints);
    solutions.clear();
    for(std::size_t i = 0; i < num_of_solutions; ++i) {

	const ikfast::IkSolutionBase<IKREAL_TYPE>& sol = ik_solutions.GetSolution(i);
	int this_sol_free_params = (int)sol.GetFree().size();

	sol.GetSolution(&solValues[0],NULL);
	sensor_msgs::JointState jointState;
	for( std::size_t j = 0; j < solValues.size(); ++j) {
	    jointState.name.push_back(minibot_arm_joint_names[j]);
	    jointState.position.push_back(solValues[j]);
	}

	// check the position limits defined in the urdf
	bool limits_are_ok = satisfiesBounds(kinematic_state, jointState);

	// check for self-collision if limits are fine
	// planning_scene::PlanningScenePtr ptr(&planning_scene);
	bool limits_and_no_collision  = limits_are_ok && !inSelfCollision(kinematic_state);


	ROS_DEBUG_STREAM_NAMED(LOG_NAME, " sol[" << i << "]=[" << std::setprecision(5)
			       << solValues[0]<< ", "
			       << solValues[1]<< ", "
			       << solValues[2]<< ", "
			       << solValues[3]<< ", "
			       << solValues[4]<< ", "
			       << solValues[5]  << "] "
			       << (limits_are_ok?"in bounds":"out of bounds") << ", "
			       << (limits_are_ok && limits_and_no_collision?"non colliding":(limits_are_ok?"self-colling":"")));


	if (limits_and_no_collision)
	  solutions.push_back(jointState);
    }
    return true;
}

void compute_fk(const sensor_msgs::JointState& jointState, geometry_msgs::Pose& pose) {
   IKREAL_TYPE eerot[9],eetrans[3];
   unsigned int numOfJoints = ikfast::GetNumJoints();

   // Put input joint values into array
   IKREAL_TYPE joints[numOfJoints];
   for (int i = 0;i<numOfJoints;i++)
    joints[i] = jointState.position[i];

   ikfast::ComputeFk(joints, eetrans, eerot); // void return
   printf("Found fk solution for end frame: \n\n");
   printf("  Translation:  x: %f  y: %f  z: %f  \n", eetrans[0], eetrans[1], eetrans[2] );
   printf("\n");
   printf("     Rotation     %f   %f   %f  \n", eerot[0], eerot[1], eerot[2] );
   printf("       Matrix:    %f   %f   %f  \n", eerot[3], eerot[4], eerot[5] );
   printf("                  %f   %f   %f  \n", eerot[6], eerot[7], eerot[8] );
   printf("\n");

   // Display equivalent Euler angles
   float yaw;
   float pitch;
   float roll;
   if ( eerot[5] > 0.999 || eerot[5] < -0.999 ) { // singularity
   	yaw = ikfast::IKatan2( -eerot[6], eerot[0] );
	pitch = 0;
   } else {
	yaw = ikfast::IKatan2( eerot[2], eerot[8] );
	pitch = ikfast::IKatan2( eerot[3], eerot[4] );
   }
   roll = ikfast::IKasin( eerot[5] );
   printf(" Euler angles: \n");
   printf("       Yaw:   %f    ", yaw ); printf("(1st: rotation around vertical blue Z-axis in ROS Rviz) \n");
   printf("       Pitch: %f  \n", pitch );
   printf("       Roll:  %f  \n", roll );
   printf("\n");

   // Convert rotation matrix to quaternion (Daisuke Miyazaki)
   float q0 = ( eerot[0] + eerot[4] + eerot[8] + 1.0f) / 4.0f;
   float q1 = ( eerot[0] - eerot[4] - eerot[8] + 1.0f) / 4.0f;
   float q2 = (-eerot[0] + eerot[4] - eerot[8] + 1.0f) / 4.0f;
   float q3 = (-eerot[0] - eerot[4] + eerot[8] + 1.0f) / 4.0f;
   if(q0 < 0.0f) q0 = 0.0f;
   if(q1 < 0.0f) q1 = 0.0f;
   if(q2 < 0.0f) q2 = 0.0f;
   if(q3 < 0.0f) q3 = 0.0f;
   q0 = sqrt(q0);
   q1 = sqrt(q1);
   q2 = sqrt(q2);
   q3 = sqrt(q3);
   if(q0 >= q1 && q0 >= q2 && q0 >= q3) {
       q0 *= +1.0f;
       q1 *= SIGN(eerot[7] - eerot[5]);
       q2 *= SIGN(eerot[2] - eerot[6]);
       q3 *= SIGN(eerot[3] - eerot[1]);
   } else if(q1 >= q0 && q1 >= q2 && q1 >= q3) {
       q0 *= SIGN(eerot[7] - eerot[5]);
       q1 *= +1.0f;
       q2 *= SIGN(eerot[3] + eerot[1]);
       q3 *= SIGN(eerot[2] + eerot[6]);
   } else if(q2 >= q0 && q2 >= q1 && q2 >= q3) {
       q0 *= SIGN(eerot[2] - eerot[6]);
       q1 *= SIGN(eerot[3] + eerot[1]);
       q2 *= +1.0f;
       q3 *= SIGN(eerot[7] + eerot[5]);
   } else if(q3 >= q0 && q3 >= q1 && q3 >= q2) {
	q0 *= SIGN(eerot[3] - eerot[1]);
        q1 *= SIGN(eerot[6] + eerot[2]);
        q2 *= SIGN(eerot[7] + eerot[5]);
        q3 *= +1.0f;
   } else {
       ROS_DEBUG_STREAM_NAMED("kinematics","Error while converting to quaternion!");
   }
   float r = NORM(q0, q1, q2, q3);
   q0 /= r;
   q1 /= r;
   q2 /= r;
   q3 /= r;

   pose.position.x = eetrans[0];
   pose.position.y = eetrans[1];
   pose.position.z = eetrans[2];

   pose.orientation.w = q0;
   pose.orientation.x = q1;
   pose.orientation.y = q2;
   pose.orientation.z = q3;

   printf("  Quaternion:  %f   %f   %f   %f   \n", q0, q1, q2, q3 );
   printf("               ");
   // print quaternion with convention and +/- signs such that it can be copy-pasted into WolframAlpha.com
   printf("%f ", q0);
   if (q1 > 0) printf("+ %fi ", q1); else if (q1 < 0) printf("- %fi ", -q1); else printf("+ 0.00000i ");
   if (q2 > 0) printf("+ %fj ", q2); else if (q2 < 0) printf("- %fj ", -q2); else printf("+ 0.00000j ");
   if (q3 > 0) printf("+ %fk ", q3); else if (q3 < 0) printf("- %fk ", -q3); else printf("+ 0.00000k ");
   printf("  (alternate convention) \n");
   printf("\n\n");
}



bool satisfiesBounds(const robot_state::RobotStatePtr& kinematic_state, const sensor_msgs::JointState& joint_state) {
  for (size_t i = 0;i<minibot_arm_joint_names.size(); i++) {
      kinematic_state->setVariablePosition(minibot_arm_joint_names[i], joint_state.position[i]);
  }
  bool ok = (kinematic_state->satisfiesBounds ());
  ROS_INFO_STREAM("result=" << ok
		  << " " << kinematic_state->getVariablePosition(0)<< "," << kinematic_state->getVariablePosition(1) <<
		  kinematic_state->getVariablePosition(2)<< "," << kinematic_state->getVariablePosition(3) <<
		  kinematic_state->getVariablePosition(4)<< "," << kinematic_state->getVariablePosition(5) );

  return ok;
}

bool inSelfCollision(const robot_state::RobotStatePtr& kinematic_state) {
  robot_model::RobotModelPtr kinematic_model = Minibot::Utils::getRobotModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);
  planning_scene.setCurrentState(*kinematic_state);
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene.checkSelfCollision(collision_request, collision_result);
  return collision_result.collision;
  ROS_INFO_STREAM("Test 1: Current state is "
                  << (collision_result.collision ? "in" : "not in")
                  << " self collision");
}

bool compute_all_ik_service(minibot::GetPositionAllIK::Request  &req,
			    minibot::GetPositionAllIK::Response &res) {

  std::vector<sensor_msgs::JointState> solutions;
  bool result = compute_ik(req.ik_request.pose_stamped.pose, solutions);
  if (result) {
    for (size_t i = 0;i<solutions.size();i++) {
	moveit_msgs::RobotState rs;
	rs.joint_state = solutions[i];
	res.solution.push_back(rs);
    }
    res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  }
  else {
      res.error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
  }
}

}
}


