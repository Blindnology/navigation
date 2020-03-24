#include <eyeguide_local_planner/path_functions.h>

namespace eyeguide_local_planner
{

	constexpr float epsilon = 1e-5f;
	constexpr float epsilon2 = epsilon*epsilon;

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///  Get point on the path closest to specified position.
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool getPathClosestPoint(const geometry_msgs::PoseStamped &pose,
							 const std::vector<geometry_msgs::PoseStamped> &path,
							 Eigen::Vector2f &pt,
							 float &isect,
							 float &distance)
	{
		if (path.empty())
			return false;

		float min_dist2 = std::numeric_limits<float>::max();

		if (path.size() == 1) {
			const geometry_msgs::Point &pos0 = path[0].pose.position;
			const geometry_msgs::Point &pos = pose.pose.position;
			Eigen::Vector2f vpos(pos.x - pos0.x, pos.y - pos0.y);
			min_dist2 = vpos.squaredNorm();
			isect = 0;
		}
		else {
			for (unsigned int i = 0; i < path.size()-1; ++i) {
				const geometry_msgs::Point &pos0 = path[i].pose.position;
				const geometry_msgs::Point &pos1 = path[i+1].pose.position;
				const geometry_msgs::Point &pos = pose.pose.position;
				Eigen::Vector2f vsect(pos1.x - pos0.x, pos1.y - pos0.y);
				Eigen::Vector2f vpos0(pos.x - pos0.x, pos.y - pos0.y);

				// Calc distance to path's directed section
				float dist2; // Distance squared
				float fsect; // Closest section fraction [0,1]
				float sect_length2 = vsect.squaredNorm();
				float dot = vsect.dot(vpos0); // Dot-product
				// Section moving away
				if (dot < epsilon2) {
					dist2 = vpos0.squaredNorm();
					fsect = 0.0;
				}
				// Section approaching
				else if (dot > sect_length2 - epsilon2) {
					Eigen::Vector2f vpos1(pos.x - pos1.x, pos.y - pos1.y);
					dist2 = vpos1.squaredNorm();
					fsect = 1.0;
				}
				// Section bypassing
				else {
					float cross = vsect.x()*vpos0.y() - vsect.y()*vpos0.x(); // Cross-product
					dist2 = cross*cross/sect_length2;
					fsect = dot/sect_length2;
				}

				// Update min distance to the path and section index
				if (dist2 < min_dist2 + epsilon2) {
					min_dist2 = dist2;
					isect = i + fsect;
					pt.x() = pos0.x + fsect*vsect.x();
					pt.y() = pos0.y + fsect*vsect.y();
				}
			}
		}

		distance = sqrt(min_dist2);
		return true;
	}

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///  Get point along the path at specified longitudinal distance from specified starting position.
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	bool getPointAlongPath(const std::vector<geometry_msgs::PoseStamped> &path,
						   const Eigen::Vector2f &pt_start,
						   float isect_start,
						   float distance,
						   Eigen::Vector2f &pt)
	{
		if (path.empty() || distance < -epsilon)
			return false;

		if (distance <= epsilon) {
			pt = pt_start;
			return true;
		}

		// Find last path vertex before starting position
		unsigned int i = (unsigned int)isect_start;
		float fsect_left = i+1 - isect_start; // Fraction of the starting section left
		// Start from the next section if very close to the end of the section
		if (fsect_left < epsilon) {
			fsect_left = 1;
			i++;
		}

		float distance_left = distance;
		// Walk along the path
		while (i < path.size() - 1 && distance_left > epsilon) {
			const geometry_msgs::Point &pos0 = path[i].pose.position;
			const geometry_msgs::Point &pos1 = path[i+1].pose.position;
			Eigen::Vector2f vsect(pos1.x - pos0.x, pos1.y - pos0.y);
			float sect_length_left = vsect.norm()*fsect_left;
			// End-point found
			if (distance_left < sect_length_left + epsilon) {
				float fsect = fsect_left*distance_left/sect_length_left + 1 - fsect_left;
				pt.x() = pos0.x + fsect*vsect.x();
				pt.y() = pos0.y + fsect*vsect.y();
				break;
			}
			distance_left -= sect_length_left;
			fsect_left = 1; // Consider full sections after the starting one
			i++;
		}

		// Use the last point if path consists of 1 point only, or end of the path is reached
		if (i == path.size() - 1) {
			const geometry_msgs::Point &pos0 = path.back().pose.position;
			pt.x() = pos0.x;
			pt.y() = pos0.y;
		}

		return true;
	}

} // namespace eyeguide_local_planner
