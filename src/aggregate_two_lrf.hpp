#ifndef AGGREGATE_TWO_LRF_HPP
#define AGGREGATE_TWO_LRF_HPP

#include "geometry.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>

#include <tf2_ros/transform_listener.h>

class AggregatorTwoLRF
{
public:
	AggregatorTwoLRF(
		float angle_cutoff_lrf_front,
		float range_cutoff_lower_lrf_front,
		float range_cutoff_upper_lrf_front,
		float angle_cutoff_lrf_rear,
		float range_cutoff_lower_lrf_rear,
		float range_cutoff_upper_lrf_rear);

	void initialize(std::shared_ptr<rclcpp::Node> node);

	unsigned int size();
	const Geometry2D::Vec2& getPoint(unsigned int index);

	void callbackLRFFront(const sensor_msgs::msg::LaserScan::SharedPtr lrf_msg);
	void callbackLRFRear(const sensor_msgs::msg::LaserScan::SharedPtr lrf_msg);

	std::vector<Geometry2D::Vec2> points_front;
	std::vector<Geometry2D::Vec2> points_rear;

	// parameters lrf front
	const float angle_cutoff_lrf_front;
	const float range_cutoff_lower_lrf_front;
	const float range_cutoff_upper_lrf_front;
	// parameters lrf rear
	const float angle_cutoff_lrf_rear;
	const float range_cutoff_lower_lrf_rear;
	const float range_cutoff_upper_lrf_rear;

	std::shared_ptr<tf2_ros::Buffer> tf_buffer;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener;

	struct IndexOutOfRangeException { };

private:
	void getPointsFromLRF(const sensor_msgs::msg::LaserScan::SharedPtr lrf_msg,
		float angle_cutoff, float range_cutoff_lower, float range_cutoff_upper,
		std::vector<Geometry2D::Vec2>* result_points);
};

/*struct AggregatorTwoLRFDepthCamera : public AggregatorTwoLRF
{
	std::vector<Geometry2D::Vec2> points_depth_camera;
};*/

#endif
