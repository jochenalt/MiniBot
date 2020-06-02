
#define IKFAST_HAS_LIBRARY 	// Build IKFast with API functions
#define IKFAST_NO_MAIN 		// Don't include main() from IKFast


#include "kinematics.h"
#include "../src/minibot_minibot_arm_ikfast_solver.cpp"


#include <stdio.h>
#include <stdlib.h>

float SIGN(float x);
float NORM(float a, float b, float c, float d);

#define IKREAL_TYPE IkReal


// computes all IK solutions of a given pose and returns those in solutions
// returns true if success
bool compute_ik(const geometry_msgs::Pose& pose, std::vector<sensor_msgs::JointState>& solutions) {
    IKREAL_TYPE eerot[9],eetrans[3];

    unsigned int numOfJoints = GetNumJoints();

    // should be 0 on a 6DOF robot
    unsigned int num_free_parameters = GetNumFreeParameters();

    IkSolutionList<IKREAL_TYPE> ik_solutions;

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

    bool success = ComputeIk(eetrans, eerot, NULL, ik_solutions);
    if( !success )
	return false;

    unsigned int num_of_solutions = (int)ik_solutions.GetNumSolutions();
    ROS_DEBUG_STREAM_NAMED(LOG_NAME, "compute_ik(" << pose << ")" << num_of_solutions << "solutions.");
    std::vector<IKREAL_TYPE> solValues(numOfJoints);
    solutions.clear();
    for(std::size_t i = 0; i < num_of_solutions; ++i) {
	const IkSolutionBase<IKREAL_TYPE>& sol = ik_solutions.GetSolution(i);
	int this_sol_free_params = (int)sol.GetFree().size();

	printf("sol%d ", (int)i);

	sol.GetSolution(&solValues[0],NULL);
	sensor_msgs::JointState jointState;
	for( std::size_t j = 0; j < solValues.size(); ++j) {
	    jointState.position.push_back(solValues[j]);
	    printf("%.15f, ", solValues[j]);
	}
	solutions.push_back(jointState);

	printf("\n");

	ROS_DEBUG_STREAM_NAMED(LOG_NAME, " sol%d "
			       << round(solValues[0]*10000.0)/10000.0  << ", "
			       << round(solValues[1]*10000.0)/10000.0 << ", "
			       << round(solValues[2]*10000.0)/10000.0  << ", "
			       << round(solValues[3]*10000.0)/10000.0 << ", "
			       << round(solValues[4]*10000.0)/10000.0  << ", "
			       << round(solValues[5]*10000.0)/10000.0);
    }
    return true;
}

void compute_fk(const sensor_msgs::JointState& jointState, geometry_msgs::Pose& pose) {
   IKREAL_TYPE eerot[9],eetrans[3];
   unsigned int numOfJoints = GetNumJoints();

   // Put input joint values into array
   IKREAL_TYPE joints[numOfJoints];
   for (int i = 0;i<numOfJoints;i++)
    joints[i] = jointState.position[i];

   ComputeFk(joints, eetrans, eerot); // void return
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
   	yaw = IKatan2( -eerot[6], eerot[0] );
	pitch = 0;
   } else {
	yaw = IKatan2( eerot[2], eerot[8] );
	pitch = IKatan2( eerot[3], eerot[4] );
   }
   roll = IKasin( eerot[5] );
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
       ROS_DEBUG_STREAM_NAMED(LOG_NAME,"Error while converting to quaternion!");
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

float SIGN(float x) {
    return (x >= 0.0f) ? +1.0f : -1.0f;
}

float NORM(float a, float b, float c, float d) {
    return sqrt(a * a + b * b + c * c + d * d);
}


