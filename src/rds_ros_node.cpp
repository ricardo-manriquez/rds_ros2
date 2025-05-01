#include "rds_ros_node.hpp"

#include "capsule.hpp"
#include "config_rds_5.hpp"
#include "distance_minimizer.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

#define _USE_MATH_DEFINES
#include <cmath>

using Geometry2D::Vec2;
using Geometry2D::Capsule;
using AdditionalPrimitives2D::Circle;

#ifdef RDS_ROS_USE_TRACKER
	PersonTracks::PersonTracks(const frame_msgs::TrackedPersons::ConstPtr& tracks_msg)
		: time(std::chrono::high_resolution_clock::now())
		, frame_id(tracks_msg->header.frame_id)
		, delay(0.1f)
	{
		MovingObject3 pers_global;
		for (const auto& track : tracks_msg->tracks)
		{
			pers_global.position.setX(track.pose.pose.position.x);
			pers_global.position.setY(track.pose.pose.position.y);
			pers_global.position.setZ(track.pose.pose.position.z);
			pers_global.velocity.setX(track.twist.twist.linear.x);
			pers_global.velocity.setY(track.twist.twist.linear.y);
			pers_global.velocity.setZ(track.twist.twist.linear.z);
			persons_global.push_back(pers_global);
		}
		for (auto& pers : persons_global)
			pers.position += delay*pers.velocity;
	}

	void PersonTracks::updatePositions(const std::chrono::time_point<std::chrono::high_resolution_clock>& time_now)
	{
		std::chrono::duration<double> t_step_duration(time_now - time);
		time = time_now;
		for (auto& pers : persons_global)
			pers.position += t_step_duration.count()*pers.velocity;
	}

	int RDSNode::obtainTf(const std::string& frame_id_1, const std::string& frame_id_2, tf2::Transform* tf)
	{
		geometry_msgs::TransformStamped transformStamped;
		try
		{
			transformStamped = tf_buffer.lookupTransform(
				frame_id_1, frame_id_2 //"sick_laser_front"//
				, ros::Time(0));
		}
		catch (tf2::TransformException &ex)
		{
			RCLCPP_WARN(this->get_logger(), "%s excpetion, when looking up tf from %s to %s", ex.what(), frame_id_1.c_str(), frame_id_2.c_str());
			return 1;
		}

		tf2::Quaternion rotation(transformStamped.transform.rotation.x,
			transformStamped.transform.rotation.y,
			transformStamped.transform.rotation.z,
			transformStamped.transform.rotation.w);

		tf2::Vector3 translation(transformStamped.transform.translation.x,
			transformStamped.transform.translation.y,
			transformStamped.transform.translation.z);

		*tf = tf2::Transform(rotation, translation);
		return 0;
	}

	void RDSNode::callbackTracker(const frame_msgs::TrackedPersons::ConstPtr& tracks_msg)
	{
		m_person_tracks = PersonTracks(tracks_msg);
	}

	int RDSNode::makeLocalPersons(const std::vector<MovingObject3>& persons_global,
		const std::string& tracks_frame_id, std::vector<MovingCircle>* persons_local)
	{
		if (persons_global.size() == 0)
			return 0;
		tf2::Transform tf;
		int error_tf_lookup = obtainTf("tf_rds", tracks_frame_id, &tf);
		if (error_tf_lookup)
			return 1;
		tf2::Transform tf_only_rotation(tf.getRotation());

		if (persons_global.size() != persons_local->size())
			persons_local->resize(persons_global.size());

		tf2::Vector3 position_local, velocity_local;
		for (unsigned int i = 0; i != persons_global.size(); i++)
		{
			position_local = tf*persons_global[i].position;
			velocity_local = tf_only_rotation*persons_global[i].velocity;
			(*persons_local)[i].circle.center.x = position_local.getX();
			(*persons_local)[i].circle.center.y = position_local.getY();
			(*persons_local)[i].circle.radius = 0.3f;
			(*persons_local)[i].velocity.x = velocity_local.getX();
			(*persons_local)[i].velocity.y = velocity_local.getY();
		}
		return 0;
	}
#endif

void RDSNode::cmdvel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
	// prepare pedestrian tracks/ scan points retrieved from recent messages
	std::vector<MovingCircle> lrf_moving_objects;
	std::vector<MovingCircle> all_moving_objects;
	if (lrf_point_obstacles)
	{
		MovingCircle moving_object;
		moving_object.velocity = Vec2(0.0, 0.0);
		moving_object.circle.radius = 0.0;
		for (unsigned int i = 0; i < m_aggregator_two_lrf.size(); i++)
		{
			moving_object.circle.center = m_aggregator_two_lrf.getPoint(i);
			lrf_moving_objects.push_back(moving_object);
			all_moving_objects.push_back(moving_object);
		}
	}

#ifdef RDS_ROS_USE_TRACKER
	m_person_tracks.updatePositions(std::chrono::high_resolution_clock::now());
	if (makeLocalPersons(m_person_tracks.getPersonsGlobal(),
		m_person_tracks.getFrameId(), &m_person_tracks.persons_local) != 0)
		RCLCPP_WARN(this->get_logger(), "Using old local person positions (could be none).");
	for (auto& pedestrian : m_person_tracks.persons_local)
		all_moving_objects.push_back(pedestrian);
#endif

	// parse service parameters
	float a_v_min = command_correct_previous_linear - dt*acc_limit_linear_abs_max;
	float a_v_max = command_correct_previous_linear + dt*acc_limit_linear_abs_max;
	float a_w_min = command_correct_previous_angular - dt*acc_limit_angular_abs_max;
	float a_w_max = command_correct_previous_angular + dt*acc_limit_angular_abs_max;
	VWBox vw_box_limits(a_v_min, a_v_max, a_w_min, a_w_max);

	VWDiamond vw_diamond_limits(vel_lim_linear_min, vel_lim_linear_max,
		vel_lim_angular_abs_max, vel_linear_at_angular_abs_max);

	const RDS5CapsuleConfiguration rds_5_config = ConfigRDS5::ConfigWrap(dt).rds_5_config;

	float tau = rds_tau;// rds_5_config.tau;
	float delta = rds_delta;// rds_5_config.delta;
	float y_p_ref = reference_point_y;// rds_5_config.y_p_ref;

	Geometry2D::RDS5 rds_5(tau, delta, y_p_ref, vw_box_limits, vw_diamond_limits);

	rds_5.use_conservative_shift = false;
	rds_5.keep_origin_feasible = false;
	rds_5.no_VO_shift_at_contact = false;
	rds_5.shift_reduction_range = 0.35f;
	rds_5.ORCA_use_p_ref = true;
	rds_5.ORCA_solver = true;

	Capsule robot_shape(capsule_radius, Vec2(0.0, capsule_center_front_y),
		Vec2(0.0, capsule_center_rear_y)); //0.45, 0.05, -0.5

	Vec2 v_nominal_p_ref(-y_p_ref*msg->angular.z, msg->linear.x);

	Vec2 v_previous_command(-command_correct_previous_angular*y_p_ref,
		command_correct_previous_linear);

	Vec2 v_corrected_p_ref(0.f, 0.f);

	if (v_nominal_p_ref.norm() > std::abs(vw_diamond_limits.v_max))
		v_nominal_p_ref = v_nominal_p_ref.normalized()*std::abs(vw_diamond_limits.v_max);

	// compute collision avoidance command
	try
	{
		rds_5.computeCorrectedVelocity(robot_shape, v_nominal_p_ref, v_previous_command,
			std::vector<MovingCircle>(), all_moving_objects, &v_corrected_p_ref);
	}
	catch (Geometry2D::DistanceMinimizer::InfeasibilityException e)
	{
		float breaking_step_linear = dt*rds_5_config.breaking_deceleration_linear;
		float breaking_step_angular = dt*rds_5_config.breaking_deceleration_angular;
		float new_v_linear, new_v_angular;
		if (command_correct_previous_linear > 0.f)
			new_v_linear = std::max(0.f, command_correct_previous_linear - breaking_step_linear);
		else
			new_v_linear = std::min(0.f, command_correct_previous_linear + breaking_step_linear);
		if (command_correct_previous_angular > 0.f)
			new_v_angular = std::max(0.f, command_correct_previous_angular - breaking_step_angular);
		else
			new_v_angular = std::min(0.f, command_correct_previous_angular + breaking_step_angular);
		v_corrected_p_ref.y = new_v_linear;
		v_corrected_p_ref.x = -new_v_angular*y_p_ref;//rds_5_config.y_p_ref;
	}

	// communicate the result and the underlying representations
    auto pub_vel = geometry_msgs::msg::Twist();
    pub_vel.linear.x = v_corrected_p_ref.y;
    pub_vel.angular.z = -1.0/y_p_ref*v_corrected_p_ref.x;

	command_correct_previous_linear = v_corrected_p_ref.y;
	command_correct_previous_angular = -1.0/y_p_ref*v_corrected_p_ref.x;

    publisher_cmd_vel->publish(pub_vel);
}

RDSNode::RDSNode(AggregatorTwoLRF& agg) :
	Node("rds_ros2_node"),
	m_aggregator_two_lrf(agg),
	tf_buffer(this->get_clock())
{
    this->declare_parameter("front_lidar", "front_lidar/scan");
    this->declare_parameter("rear_lidar", "rear_lidar/scan");
    this->declare_parameter("cmd_vel_in", "cmd_vel_in");
    this->declare_parameter("cmd_vel_out", "cmd_vel_out");

    std::string front_lidar = this->get_parameter("front_lidar").as_string();
    std::string rear_lidar = this->get_parameter("rear_lidar").as_string();
    std::string cmd_vel_in = this->get_parameter("cmd_vel_in").as_string();
    std::string cmd_vel_out = this->get_parameter("cmd_vel_out").as_string();

	auto default_qos = rclcpp::QoS(rclcpp::SensorDataQoS());
	subscriber_lrf_front = this->create_subscription<sensor_msgs::msg::LaserScan>(
		front_lidar,
		default_qos,
		std::bind(&AggregatorTwoLRF::callbackLRFFront, &m_aggregator_two_lrf, std::placeholders::_1)
	);
	subscriber_lrf_rear = this->create_subscription<sensor_msgs::msg::LaserScan>(
		rear_lidar,
		default_qos,
		std::bind(&AggregatorTwoLRF::callbackLRFRear, &m_aggregator_two_lrf, std::placeholders::_1)
	);
	subscriber_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
		cmd_vel_in,
		10,
		std::bind(&RDSNode::cmdvel_callback, this, std::placeholders::_1)
	);
	publisher_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>(
		cmd_vel_out,
		10
	);
	tf_listener = std::make_shared<tf2_ros::TransformListener>(tf_buffer, this, false);
	command_correct_previous_linear = 0.f;
	command_correct_previous_angular = 0.f;
    this->declare_parameter("capsule_center_front_y", 0.18);
    this->declare_parameter("capsule_center_rear_y", -0.5);
    this->declare_parameter("capsule_radius", 0.45);
    this->declare_parameter("reference_point_y", 0.18);
    this->declare_parameter("rds_tau", 1.5);
    this->declare_parameter("rds_delta", 0.05);
    this->declare_parameter("vel_lim_linear_min", -0.5);
    this->declare_parameter("vel_lim_linear_max", 1.5);
    this->declare_parameter("vel_lim_angular_abs_max", 1.0);
    this->declare_parameter("vel_linear_at_angular_abs_max", 0.2);
    this->declare_parameter("acc_limit_linear_abs_max", 0.5);
    this->declare_parameter("acc_limit_angular_abs_max", 0.5);
    this->declare_parameter("dt", 0.01);
    this->declare_parameter("lrf_point_obstacles", true);

   capsule_center_front_y = this->get_parameter("capsule_center_front_y").as_double();
   capsule_center_rear_y = this->get_parameter("capsule_center_rear_y").as_double();
   capsule_radius = this->get_parameter("capsule_radius").as_double();
   reference_point_y = this->get_parameter("reference_point_y").as_double();
   rds_tau = this->get_parameter("rds_tau").as_double();
   rds_delta = this->get_parameter("rds_delta").as_double();
   vel_lim_linear_min = this->get_parameter("vel_lim_linear_min").as_double();
   vel_lim_linear_max = this->get_parameter("vel_lim_linear_max").as_double();
   vel_lim_angular_abs_max = this->get_parameter("vel_lim_angular_abs_max").as_double();
   vel_linear_at_angular_abs_max = this->get_parameter("vel_linear_at_angular_abs_max").as_double();
   acc_limit_linear_abs_max = this->get_parameter("acc_limit_linear_abs_max").as_double();
   acc_limit_angular_abs_max = this->get_parameter("acc_limit_angular_abs_max").as_double();
   dt = this->get_parameter("dt").as_double();
   lrf_point_obstacles = this->get_parameter("lrf_point_obstacles").as_bool();
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	AggregatorTwoLRF aggregator_two_lrf(
		3.f*M_PI/4.f,
		0.05,
		100.f,
		3.f*M_PI/4.f,
		0.05,
		100.f);
	auto node = std::make_shared<RDSNode>(aggregator_two_lrf);
	aggregator_two_lrf.initialize(node);
	rclcpp::spin(node);
	rclcpp::shutdown();
	return 0;
}
