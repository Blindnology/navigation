#pragma once

// General
#include <string>
#include <Eigen/Core>

// ROS
#include <geometry_msgs/Twist.h>

// Base local planner
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/odometry_helper_ros.h>

namespace eyeguide_local_planner
{

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///  @class PivotRotationController
	///  @brief Controller for rotating the robot in place towards the target heading if the difference
	///  between current heading and target heading is larger than some limit.
	///  Currently, no stopping is performed before rotating.
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	class PivotRotationController
	{
	public:
		/**
		 * @brief Constructor
		 */
		PivotRotationController(): controller_on_(true), in_action_(false) {}

		/**
		 * @brief Destructor
		 */
		~PivotRotationController() {}

		/**
		 * @brief Resets controller state
		 */
		void reset() { in_action_ = false; }

		/**
		 * @brief This function is to be called only when parameters change
		 *
		 * @param[in]	pivot_turn_angle		Difference angle (rad) between current robot's heading and target heading to switch to pivot rotation
		 * @param[in]	yaw_target_tolerance	Tolerance (rad) for difference angle to consider the robot facing the target direction
		 * @param[in]	path_heading_distance	Path distance to compute global path heading (m)
		 */
		void setParameters(double pivot_turn_angle, double yaw_target_tolerance, double path_heading_distance);

		/**
		 * @brief Checks if we need to run the controller
		 *
		 * @param[in]	robot_pose		Current robot's pose
		 * @param[in]	global_plan		Global plan
		 * @param[in]	planner_util	Planner utils
		 * @param[in]	odom_helper		Robot odometry helper
		 * @param[out]	heading_path	Global heading path
		 * @return True if we need to run the controller
		 */
		bool isRunNeeded(const geometry_msgs::PoseStamped &robot_pose,
						 const std::vector<geometry_msgs::PoseStamped> &global_plan,
						 base_local_planner::LocalPlannerUtil &planner_util,
						 base_local_planner::OdometryHelperRos &odom_helper,
						 std::vector<Eigen::Vector2f> &heading_path);

		/**
		 * @brief Compute the velocity commands
		 * Should be called if isRunNeeded returned true
		 *
		 * @param[in]	sim_period		Simulation time step (s)
		 * @param[out]	cmd_vel			The velocity commands to be filled
		 * @return True if a valid command was found
		 */
		bool computeVelocityCommand(double sim_period,
									geometry_msgs::Twist &cmd_vel);

	private:
		static constexpr double epsilon = 1e-5;
		static constexpr double epsilon2 = epsilon*epsilon;

		inline double sign(double x) { return x < 0.0 ? -1.0 : 1.0; }

		/**
		 * @brief Rotate toward the target heading respecting velocity/acceleration limits.
		 *
		 * @param[in]	diff_angle		Difference angle (rad)
		 * @param[in]	vel_theta_cur	Robot's angular speed (rad/s)
		 * @param[in]	sim_period		Simulation time step (s)
		 * @param[in]	limits			Velocity/acceleration limits
		 * @param[out]	cmd_vel			The velocity commands to be filled
		 * @return True if a valid command was found
		 */
		bool rotateInPlace(double diff_angle,
						   double vel_theta_cur,
						   double sim_period,
						   const base_local_planner::LocalPlannerLimits &limits,
						   geometry_msgs::Twist &cmd_vel);

		/**
		 * @brief Calc difference angle between current robot's heading and target heading.
		 *
		 * @param[in]	robot_pose		Current robot's pose
		 * @param[in]	global_plan		Global plan
		 * @param[out]	heading_path	Global heading path
		 * @return Difference angle (rad)
		 */
		double calcDifferenceAngle(const geometry_msgs::PoseStamped &robot_pose,
								   const std::vector<geometry_msgs::PoseStamped> &global_plan,
								   std::vector<Eigen::Vector2f> &heading_path);

		double pivot_turn_angle_, yaw_target_tolerance_, path_heading_distance_;
		bool controller_on_; // Controller's turned on/off
		bool in_action_; // Controller's state

		double diff_angle_;
		geometry_msgs::PoseStamped robot_vel_;
		base_local_planner::LocalPlannerLimits limits_;
	};

} // namespace eyeguide_local_planner
