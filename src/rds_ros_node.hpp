#ifndef RDS_ROS_NODE_HPP
#define RDS_ROS_NODE_HPP

#include "aggregate_two_lrf.hpp"
#include "geometry.hpp"
#include "rds_5.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.h>
#include <geometry_msgs/msg/twist.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>

#include <vector>
#include <string>

#include <chrono>

#ifdef RDS_ROS_USE_TRACKER
	struct MovingObject3
	{
		tf2::Vector3 position, velocity;
	};

	struct PersonTracks
	{
		PersonTracks() { }
		PersonTracks(const frame_msgs::TrackedPersons::ConstPtr& tracker_message);
		void updatePositions(const std::chrono::time_point<std::chrono::high_resolution_clock>& time_now);
		const std::vector<MovingObject3>& getPersonsGlobal() const { return persons_global; }
		const std::string& getFrameId() const { return frame_id; }

		std::vector<MovingCircle> persons_local;
	private:
		std::vector<MovingObject3> persons_global;
		std::chrono::time_point<std::chrono::high_resolution_clock> time;
		std::string frame_id;
		float delay;
	};
#endif

class RDSNode : public rclcpp::Node
{
public:
	RDSNode(AggregatorTwoLRF& agg);

	//bool commandCorrectionService(rds_network_ros::VelocityCommandCorrectionRDS::Request& request,
	//	rds_network_ros::VelocityCommandCorrectionRDS::Response& response);

#ifdef RDS_ROS_USE_TRACKER
	void callbackTracker(const frame_msgs::TrackedPersons::ConstPtr& tracks_msg);

	int obtainTf(const std::string& frame_id_1, const std::string& frame_id_2, tf2::Transform* tf);

	int makeLocalPersons(const std::vector<MovingObject3>& persons_global,
		const std::string& tracks_frame_id, std::vector<MovingCircle>* persons_local);

#endif
	void cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

	AggregatorTwoLRF& m_aggregator_two_lrf;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_lrf_front;
	rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_lrf_rear;

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_cmd_vel;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel;

#ifdef RDS_ROS_USE_TRACKER
//	ros::Subscriber subscriber_tracker;
#endif

//	ros::Publisher publisher_for_gui;
//	ros::ServiceServer command_correction_server;

#ifdef RDS_ROS_USE_TRACKER
	std::vector<MovingCircle> m_tracked_persons;
	PersonTracks m_person_tracks;
#endif

	tf2_ros::Buffer tf_buffer;
	std::shared_ptr<tf2_ros::TransformListener> tf_listener;
	float command_correct_previous_linear, command_correct_previous_angular;
private:
    float capsule_center_front_y, capsule_center_rear_y, capsule_radius, reference_point_y,\
          rds_tau, rds_delta, vel_lim_linear_min, vel_lim_linear_max, vel_lim_angular_abs_max,\
          vel_linear_at_angular_abs_max, acc_limit_linear_abs_max, acc_limit_angular_abs_max, dt;
    bool lrf_point_obstacles;
};

#endif
