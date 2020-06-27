/*
 * uniform_sample_filter.h
 *
 *  Created on: Jun 27, 2020
 *      Author: jochen
 */

#ifndef INCLUDE_UNIFORM_SAMPLE_FILTER_H_
#define INCLUDE_UNIFORM_SAMPLE_FILTER_H_

class UniformSampleFilter
{
	public:
		UniformSampleFilter();
		~UniformSampleFilter();
		void configure(double sample_time);
		bool update(const trajectory_msgs::JointTrajectory & trajectory_in, trajectory_msgs::JointTrajectory& trajectory_out);
		bool interpolatePt(trajectory_msgs::JointTrajectoryPoint & p1, trajectory_msgs::JointTrajectoryPoint & p2,
						   double time_from_start, trajectory_msgs::JointTrajectoryPoint & interp_pt);
		private:
			double sample_duration_;
};



#endif /* INCLUDE_UNIFORM_SAMPLE_FILTER_H_ */
