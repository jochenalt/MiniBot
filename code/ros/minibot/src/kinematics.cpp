#include <thread>
#include <mutex>

#include "ros/ros.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model/joint_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

#include "utils.h"
#include "kinematics.h"


#define IKFAST_HAS_LIBRARY 	// Build IKFast with API functions
#define IKFAST_NO_MAIN 		// Don't include main() from IKFast
namespace ikfast {
#include "../src/minibot_minibot_arm_ikfast_solver.cpp"
}

#define LOG_NAME "kinematics"

double SIGN(double x) {
    return (x >= 0.0f) ? +1.0f : -1.0f;
}

namespace Minibot {

// cache the joint names of the group "minibot_arm" as defined in SRDF
std::vector<std::string> minibot_arm_joint_names; 			// joint names of the arm without the gripper
std::vector<std::string> minibot_gripper_joint_names;		// joint names of the gripper without the arm
std::vector<std::string> minibot_joint_names;				// all joint names, including gripper


namespace Kinematics {

minibot::MinibotState last_joint_state;

void construct() {
  ROS_INFO_STREAM_NAMED(LOG_NAME, "module kinematics init");

  // cache the joint names of the kinematics group
  robot_model::RobotModelPtr kinematic_model = Utils::getRobotModel();
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  const moveit::core::JointModelGroup* jmg = kinematic_state->getJointModelGroup(minibot_arm_group_name);
  if (jmg == NULL) {
      ROS_ERROR_STREAM("Minibot::Kinematics::init: did not find joint model group"
    		  << minibot_arm_group_name);
  }
  minibot_arm_joint_names = jmg->getActiveJointModelNames();

  jmg = kinematic_state->getJointModelGroup(minibot_gripper_group_name);
  if (jmg == NULL) {
      ROS_ERROR_STREAM("Minibot::Kinematics::init: did not find joint model group"
    		  << minibot_gripper_group_name);
  }

  minibot_gripper_joint_names = jmg->getActiveJointModelNames();

  jmg = kinematic_state->getJointModelGroup(minibot_group_name);
  if (jmg == NULL) {
      ROS_ERROR_STREAM("Minibot::Kinematics::init: did not find joint model group"
    		  << minibot_group_name);
  }
  minibot_joint_names = jmg->getActiveJointModelNames();


  for (size_t i = 0;i < minibot_joint_names.size();i++) {
	  last_joint_state.joint_state.name.push_back(minibot_joint_names[i]);
	  last_joint_state.joint_state.position.push_back(0);
  }

}

// get/set last minibot state containing pose, joint state and configurations
std::mutex last_joint_state_mutex;
void setLastMinibotState(const minibot::MinibotState& state) {
	std::unique_lock<std::mutex> lock(last_joint_state_mutex);
	last_joint_state= state;
}

minibot::MinibotState getLastMinibotState() {
	std::unique_lock<std::mutex> lock(last_joint_state_mutex);
	return last_joint_state;
}


// returns an artifical distance between two joint states, used to asses different configurations and find
// the one which is closest to the current joints position
// many small changes in the joints are considered closer source than a few big changes.
double jointModelDistance(const sensor_msgs::JointState& a, const sensor_msgs::JointState& b) {
  double sum = 0;
  for (size_t i = 0;i< minibot_arm_joint_names.size();i++) {
	  std::string joint_name = minibot_arm_joint_names[i];
	  int a_idx = Minibot::Utils::findJoint(a, joint_name);
	  int b_idx = Minibot::Utils::findJoint(b, joint_name);

	  if ((a_idx < 0) || (b_idx < 0)) {
		  ROS_ERROR_STREAM("jointModelDistance: jointStates are not compatible");
	  	  return 0;
      }
      sum += pow(fmod(fabs(a.position[a_idx]-b.position[b_idx]), 2.0 * M_PI), 2.0);
  }
  return sum;
}

// computes all IK solutions of a given pose and returns those in solutions
// returns true if success.
// the passed joint state is used to sort the configurations such that the closest is returned first
bool computeIK(const geometry_msgs::Pose& pose, const sensor_msgs::JointState& jointState, minibot::JointStateConfiguration& solutions) {
    // create a local state
    robot_model::RobotModelPtr kinematic_model = Utils::getRobotModel();
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();

    ikfast::IkReal eerot[9],eetrans[3];
    unsigned int numOfJoints = ikfast::GetNumJoints();

    ikfast::IkSolutionList<ikfast::IkReal> ik_solutions;

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
    std::vector<ikfast::IkReal> solValues(numOfJoints);
    solutions.configuration.clear();
    for(std::size_t i = 0; i < num_of_solutions; ++i) {
		const ikfast::IkSolutionBase<ikfast::IkReal>& sol = ik_solutions.GetSolution(i);
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


		if (limits_and_no_collision)
		  solutions.configuration.push_back(jointState);
    }

    if (solutions.configuration.size() > 0) {
		struct JointStateComparer {
			JointStateComparer(const sensor_msgs::JointState& current) { this->current = current; }

			bool operator()(const sensor_msgs::JointState& a, const sensor_msgs::JointState& b) const {
				 return jointModelDistance(current, a) < jointModelDistance(current, b);
			}
			sensor_msgs::JointState current;
		};
		std::sort(solutions.configuration.begin(), solutions.configuration.end(),
				JointStateComparer(jointState));

    }


    return solutions.configuration.size() > 0;
}

// set the position of the end effctor as defined in eff into joint_state
bool setEndEffectorPosition(sensor_msgs::JointState& joint_state, const sensor_msgs::JointState& eff) {
	for (size_t j = 0;j<minibot_gripper_joint_names.size();j++) {
		std::string gripper_joint_name = minibot_gripper_joint_names[j];
		int idx = Minibot::Utils::findJoint(eff, gripper_joint_name);
		if (idx >= 0) {
			int sidx = Minibot::Utils::findJoint(joint_state, gripper_joint_name);
			if (sidx >= 0) {
				joint_state.position[sidx] = eff.position[idx];
			} else {
				joint_state.name.push_back(gripper_joint_name);
				joint_state.position.push_back(eff.position[idx]);
			}
		} else {
			ROS_ERROR_STREAM_NAMED(LOG_NAME, "addEndEffector: did not find joint " <<  gripper_joint_name);
			return false;
		}
	}
	return true;
}

void computeFK(const sensor_msgs::JointState& jointState, geometry_msgs::Pose& pose) {
   ikfast::IkReal eerot[9],eetrans[3];
   unsigned int numOfJoints = ikfast::GetNumJoints();

   // Put input joint values into array
   ikfast::IkReal joints[numOfJoints];
   for (size_t idx = 0;idx<minibot_arm_joint_names.size();idx++)
	   joints[idx] = Utils::getJointValue(jointState, minibot_arm_joint_names[idx]);

   ikfast::ComputeFk(joints, eetrans, eerot); // void return

   // Convert rotation matrix to quaternion (Daisuke Miyazaki)
   double q0 = ( eerot[0] + eerot[4] + eerot[8] + 1.0f) / 4.0f;
   double q1 = ( eerot[0] - eerot[4] - eerot[8] + 1.0f) / 4.0f;
   double q2 = (-eerot[0] + eerot[4] - eerot[8] + 1.0f) / 4.0f;
   double q3 = (-eerot[0] - eerot[4] + eerot[8] + 1.0f) / 4.0f;
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
       ROS_DEBUG_STREAM_NAMED(LOG_NAME,"Error while converting to quaternion!");
   }
   double r = q0*q0 + q1*q1 + q2*q2+ q3*q3;
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
}


// returns true, if the state as defined in joint_state does not violate the joint limits as defined in the URDF
bool satisfiesBounds(const robot_state::RobotStatePtr& kinematic_state, const sensor_msgs::JointState& joint_state) {
  for (size_t i = 0;i<joint_state.name.size(); i++) {
      kinematic_state->setVariablePosition(joint_state.name[i], joint_state.position[i]);
  }
  bool ok = (kinematic_state->satisfiesBounds (0.001));

  return ok;
}

// return true, if the passed joint_state is in self collision
bool inSelfCollision(const robot_state::RobotStatePtr& kinematic_state) {
  robot_model::RobotModelPtr kinematic_model = Minibot::Utils::getRobotModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);
  planning_scene.setCurrentState(*kinematic_state);
  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene.checkSelfCollision(collision_request, collision_result);
  return collision_result.collision;
}


// to allow a dynamic tcp, compute the base point of the tool out of the pose and a tool length
geometry_msgs::Pose computeTCPBase(const geometry_msgs::Pose& tcpPose, double tool_distance) {

	// tool_distance += 0.01;
	// get the tool length
    ros::NodeHandle nh;

    // compute the homogeneous matrix representing the pose
    // Convert input effector pose, in w x y z quaternion notation, to rotation matrix.
    ikfast::IkReal eerot[9],eetrans[3];
    ikfast::IkReal  qw = tcpPose.orientation.w;
    ikfast::IkReal  qx = tcpPose.orientation.x;
    ikfast::IkReal  qy = tcpPose.orientation.y;
    ikfast::IkReal  qz = tcpPose.orientation.z;
    const double n = 1.0f/sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
    qw *= n;
    qx *= n;
    qy *= n;
    qz *= n;

    // multiply homogeneous matrix of pose with homogeneous vector of (0,0,tcpDistance, 1)
    geometry_msgs::Pose result(tcpPose);
    result.position.x -=  (2.0f*qx*qz + 2.0f*qy*qw) 		* tool_distance;
    result.position.y -=  (2.0f*qy*qz - 2.0f*qx*qw) 		* tool_distance;
    result.position.z -=  (1.0f - 2.0f*qx*qx - 2.0f*qy*qy) 	* tool_distance;

    ROS_DEBUG_STREAM_NAMED(LOG_NAME, "computeFlange ("
  			       << std::setprecision(5)
  			       << tool_distance << ")"
  			       << "pose=("
  			       << "pos.x=" << tcpPose.position.x << ","
  			       << "pos.y=" << tcpPose.position.y << ","
  			       << "pos.z=" << tcpPose.position.z << ","
  			       << "orient.x=" << tcpPose.orientation.x << ","
  			       << "orient.y=" << tcpPose.orientation.y << ","
  			       << "orient.z=" << tcpPose.orientation.z << ","
  			       << "orient.w=" << tcpPose.orientation.w << ") -> "
  			       << "result=("
  			       << "result.x=" << tcpPose.position.x << ","
  			       << "result.y=" << tcpPose.position.y << ","
  			       << "result.z=" << tcpPose.position.z << ")");
    return result;
}

geometry_msgs::Pose computeTCPTip(const geometry_msgs::Pose& flangePose, double tool_distance) {

	// tool_distance += 0.01;

    // get the tool length
    ros::NodeHandle nh;

    // compute the homogeneous matrix representing the pose
    // Convert input effector pose, in w x y z quaternion notation, to rotation matrix.
    ikfast::IkReal eerot[9],eetrans[3];
    ikfast::IkReal  qw = flangePose.orientation.w;
    ikfast::IkReal  qx = flangePose.orientation.x;
    ikfast::IkReal  qy = flangePose.orientation.y;
    ikfast::IkReal  qz = flangePose.orientation.z;

    // normalize the quaternion
    const double n = 1.0f/sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
    qw *= n;
    qx *= n;
    qy *= n;
    qz *= n;

    // multiply homogeneous matrix of pose with homogeneous vector of (0,0,-tcpDistance, 1)
    geometry_msgs::Pose result(flangePose);
    result.position.x +=  (2.0f*qx*qz + 2.0f*qy*qw) 		* tool_distance;
    result.position.y +=  (2.0f*qy*qz - 2.0f*qx*qw) 		* tool_distance;
    result.position.z +=  (1.0f - 2.0f*qx*qx - 2.0f*qy*qy) 	* tool_distance;


    ROS_DEBUG_STREAM_NAMED(LOG_NAME, "computeTCP ("
  			       << std::setprecision(5)
  			       << tool_distance << ")"
  			       << "pose=("
  			       << "pos.x=" << flangePose.position.x << ","
  			       << "pos.y=" << flangePose.position.y << ","
  			       << "pos.z=" << flangePose.position.z << ","
  			       << "orient.x=" << flangePose.orientation.x << ","
  			       << "orient.y=" << flangePose.orientation.y << ","
  			       << "orient.z=" << flangePose.orientation.z << ","
  			       << "orient.w=" << flangePose.orientation.w << ") -> "
  			       << "result=("
  			       << "result.x=" << result.position.x << ","
  			       << "result.y=" << result.position.y << ","
  			       << "result.z=" << result.position.z << ")");
    return result;
}


}
}


