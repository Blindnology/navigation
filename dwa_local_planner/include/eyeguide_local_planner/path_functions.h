#pragma once

// General
#include <Eigen/Core>

// ROS
#include <geometry_msgs/PoseStamped.h>

namespace eyeguide_local_planner
{

	/**
	 * @brief Get point on the path closest to specified position.
	 *
	 * @param[in]	pose		Position to find closest point to
	 * @param[in]	path		Path
	 * @param[out]	pt			Closest point
	 * @param[out]	isect		Closest path section [0, path.size()-1], fractional part for any point along the section
	 * @param[out]	distance	Closest distance
	 * @return True if valid point was found
	 */
	bool getPathClosestPoint(const geometry_msgs::PoseStamped &pose,
							 const std::vector<geometry_msgs::PoseStamped> &path,
							 Eigen::Vector2f &pt,
							 float &isect,
							 float &distance);

	/**
	 * @brief Get point along the path at specified longitudinal distance from specified starting position.
	 *
	 * @param[in]	path		Path
	 * @param[in]	pt_start	Starting point
	 * @param[in]	isect_start	Starting section [0, path.size()-1], fractional part for any point along the section
	 * @param[in]	distance	Distance along the path (must be positive)
	 * @param[out]	pt			Point found
	 * @return True if valid point was found
	 */
	bool getPointAlongPath(const std::vector<geometry_msgs::PoseStamped> &path,
						   const Eigen::Vector2f &pt_start,
						   float isect_start,
						   float distance,
						   Eigen::Vector2f &pt);


} // namespace eyeguide_local_planner
