// General
#include <cmath>

// ROS
#include <angles/angles.h>
#include <tf2/utils.h>

// Base local planner
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/local_planner_limits.h>

#include <eyeguide_local_planner/path_functions.h>
#include <eyeguide_local_planner/pivot_rotation_controller.h>

namespace eyeguide_local_planner
{

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///  This function is to be called only when parameters change.
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	void PivotRotationController::setParameters(double pivot_turn_angle, double yaw_target_tolerance, double path_heading_distance)
	{
		pivot_turn_angle_ = std::max(0.0, pivot_turn_angle);
		yaw_target_tolerance_ = std::max(0.0, yaw_target_tolerance);
		path_heading_distance_ = std::max(0.0, path_heading_distance);
		// If pivot angle is less than tolerance, turn the controller off
		if (pivot_turn_angle_ <= yaw_target_tolerance_)
			controller_on_ = false;
		else
			controller_on_ = true;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///  Checks if we need to run the controller.
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool PivotRotationController::isRunNeeded(const geometry_msgs::PoseStamped &robot_pose,
											  const std::vector<geometry_msgs::PoseStamped> &global_plan,
											  base_local_planner::LocalPlannerUtil &planner_util,
											  base_local_planner::OdometryHelperRos &odom_helper,
											  std::vector<Eigen::Vector2f> &heading_path)
	{
		// Check for controller on/off state
		if (!controller_on_) {
			in_action_ = false;
			return false;
		}

		if (global_plan.empty())
			return false;

		// Difference angle between current robot's heading and target heading
		diff_angle_ = calcDifferenceAngle(robot_pose, global_plan, heading_path);

		// Get robot's velocity
		odom_helper.getRobotVel(robot_vel_);

		// Get current limits
		limits_ = planner_util.getCurrentLimits();

		// If already in action
		if (in_action_) {
			// If tolerance difference angle was reached then stop acting
			if (fabs(diff_angle_) < yaw_target_tolerance_ + epsilon) {
				nav_msgs::Odometry base_odom;
				base_odom.twist.twist.linear.x = robot_vel_.pose.position.x;
				base_odom.twist.twist.linear.y = robot_vel_.pose.position.y;
				base_odom.twist.twist.angular.z = tf2::getYaw(robot_vel_.pose.orientation);
				// Make sure that we're actually stopped before stopping acting
				if (base_local_planner::stopped(base_odom, limits_.theta_stopped_vel, limits_.trans_stopped_vel)) {
					ROS_DEBUG_NAMED("pivot_rotation_controller", "Desired orientation is reached, stop rotating.");
					in_action_ = false;
				}
				else
					ROS_DEBUG_NAMED("pivot_rotation_controller", "Desired orientation is reached, stopping rotating...");
			}
		}
		// If not yet in action
		else {
			// If difference is larger than pivot angle then start acting
			if (fabs(diff_angle_) > pivot_turn_angle_ - epsilon) {
				ROS_DEBUG_NAMED("pivot_rotation_controller", "Desired orientation is far (%.0f deg), start rotating.", angles::to_degrees(diff_angle_));
				in_action_ = true;
			}
		}

		return in_action_;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///  Compute the velocity commands.
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool PivotRotationController::computeVelocityCommand(double sim_period,
														 geometry_msgs::Twist &cmd_vel)
	{
		// Set velocity to zero in case we reached yaw goal tolerance
		if (fabs(diff_angle_) <= yaw_target_tolerance_) {
			cmd_vel.linear.x = 0.0;
			cmd_vel.linear.y = 0.0;
			cmd_vel.angular.z = 0.0;
		}
		else {
			if (!rotateInPlace(diff_angle_,
							   tf2::getYaw(robot_vel_.pose.orientation),
							   sim_period,
							   limits_,
							   cmd_vel))
			{
				ROS_WARN_NAMED("pivot_rotation_controller", "Error when pivot rotating.");
				return false;
			}
		}

		return true;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///  Rotate toward the target heading respecting velocity/acceleration limits.
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool PivotRotationController::rotateInPlace(double diff_angle,
												double vel_theta_cur,
												double sim_period,
												const base_local_planner::LocalPlannerLimits &limits,
												geometry_msgs::Twist &cmd_vel)
	{
		// Zero translational velocity
		cmd_vel.linear.x = 0.0;
		cmd_vel.linear.y = 0.0;

		// Difference angle between current robot's heading and target heading
		double sign_diff_angle = sign(diff_angle);

		// Request maximum rotational velocity
		double vel_theta = sign_diff_angle * limits.max_vel_theta;

		// Take the acceleration limits of the robot into account
		double max_acc_vel_theta = vel_theta_cur + limits.acc_lim_theta * sim_period;
		double min_acc_vel_theta = vel_theta_cur - limits.acc_lim_theta * sim_period;
		vel_theta = std::min(max_acc_vel_theta, std::max(min_acc_vel_theta, vel_theta));

		// If commanded velocity is toward the target
		if (sign(vel_theta) == sign_diff_angle) {
			// Keep minimum velocity.
			// Keep velocity below value that allows us to stop when we reach the target, given our acceleration limit.
			double max_vel_theta_to_stop = sqrt(2.0 * limits.acc_lim_theta * fabs(diff_angle));
			vel_theta = sign_diff_angle * std::min(max_vel_theta_to_stop, std::max(limits.min_vel_theta, fabs(vel_theta)));
		}

		// Don't exceed maximum velocity
		vel_theta = sign(vel_theta) * std::min(limits.max_vel_theta, fabs(vel_theta));

		ROS_DEBUG_NAMED("pivot_rotation_controller", "Rotating toward desired orientation (%.0f deg), cmd %.2f", angles::to_degrees(diff_angle), vel_theta);
		cmd_vel.angular.z = vel_theta;

		return true;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///  Calc difference angle between current robot's heading and target heading.
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	double PivotRotationController::calcDifferenceAngle(const geometry_msgs::PoseStamped &robot_pose,
														const std::vector<geometry_msgs::PoseStamped> &global_plan,
														std::vector<Eigen::Vector2f> &heading_path)
	{
		// Approximate global plan with linear section
		heading_path.resize(2);

		// If path consists of 1 point only, use it as target heading from robot's pose
		if (global_plan.size() == 1) {
			const geometry_msgs::Point &pos0 = robot_pose.pose.position;
			const geometry_msgs::Point &pos1 = global_plan[0].pose.position;
			heading_path[0] = Eigen::Vector2f(pos0.x, pos0.y);
			heading_path[1] = Eigen::Vector2f(pos1.x, pos1.y);
		}
		// Compute local path heading
		else {
			// Find the point on the global plan closest to the robot position
			Eigen::Vector2f min_pt, path_pt;
			float min_isect, min_dist;
			getPathClosestPoint(robot_pose, global_plan, min_pt, min_isect, min_dist);

			// Find point along the global plan at specified longitudinal distance from the closest point
			getPointAlongPath(global_plan, min_pt, min_isect, path_heading_distance_, path_pt);

			// Use two points to approximate local path heading
			heading_path[0] = min_pt;
			heading_path[1] = path_pt;
			Eigen::Vector2f heading_vec = heading_path[1] - heading_path[0];

			// If two points on the path are too close
			// (bcz path_heading_distance_ = 0 or the end of plan is reached),
			// use closest section heading.
			if (fabs(heading_vec.x()) < epsilon && fabs(heading_vec.y()) < epsilon) {
				// Find last path vertex before starting position
				unsigned int min_i = (unsigned int)min_isect;
				// Start from the next section if very close to the end of the section
				if (min_isect > min_i + 1 - epsilon)
					min_i++;
				// Use last section if we are after the end of plan
				min_i = std::min(min_i, (unsigned int)global_plan.size() - 2);
				const geometry_msgs::Point &pos0 = global_plan[min_i].pose.position;
				const geometry_msgs::Point &pos1 = global_plan[min_i+1].pose.position;
				heading_path[0] = Eigen::Vector2f(pos0.x, pos0.y);
				heading_path[1] = Eigen::Vector2f(pos1.x, pos1.y);
			}
		}

		// Compute target heading
		Eigen::Vector2f heading_vec = heading_path[1] - heading_path[0];
		float target_heading = atan2(heading_vec.y(), heading_vec.x());

		// Difference angle between current robot's heading and target heading
		return base_local_planner::getGoalOrientationAngleDifference(robot_pose, target_heading);
	}

} // namespace eyeguide_local_planner
