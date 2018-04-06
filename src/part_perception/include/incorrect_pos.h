/**
 * @brief This class publish part position that is incorrect on AGV_1
 * 1. publish topic /ariac/incorrect_parts_agv_1
 * 2. create server /ariac/incorrect_parts_agv_1
 */

#ifndef PART_PERCEPTION_INCLUDE_INCORRECT_POS_H_
#define PART_PERCEPTION_INCLUDE_INCORRECT_POS_H_


#include <vector>
#include <ros/ros.h>
#include <math.h>
#include "tf2_msgs/TFMessage.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Transform.h"
#include "std_msgs/Header.h"
#include "osrf_gear/Order.h"
#include "part_perception/Incorrect_Part.h"

class Incorrect_Pos_AGV {
private:
	std::vector<osrf_gear::Order> received_orders;	// recieved orders
	ros::NodeHandle node;	// ros nodehandle
	tf2_msgs::TFMessage incorrect_part_pos_agv_1;		// the position of all parts on agv_1

	const double translation_tolerance = 0.03;		// the tolerance in translation is 0.03m
	const double orientation_tolerance = 0.1;		// the tolerance in orientation is 0.1 rad

	ros::Publisher agv_1_incorrect_part_publisher;	// initialize publisher to publish incorrect_part of AVG
	const int publish_rate = 10;					// the publish rate is 10 Hz
	tf::TransformListener listener;					// transform listener

	geometry_msgs::TransformStamped part_pos_agv_1;	// part_position to agv 1

	std::string agv_1_reference_frame = "agv1_load_point_frame";		// reference frame of agv_1

	std::string which_order;						// order find same possible part
	std::string which_kit;							// the kit find same possible part

	ros::ServiceServer Incorrect_Part_Server;		// server to query incorrect part in agv_1

	tf2_msgs::TFMessage server_data;				// data for /ariac/check_part_pos_agv_1 server




public:
	explicit Incorrect_Pos_AGV(ros::NodeHandle node);
	void order_callback(const osrf_gear::Order::ConstPtr & order_msg);	// recieve orders
	void agv_part_detect(const tf2_msgs::TFMessage::ConstPtr& msg);
	bool is_within_tolerance(const geometry_msgs::Transform& actual_pose, const geometry_msgs::Pose& desired_pose);;
	bool is_within_translation_tolerance(const geometry_msgs::Vector3& actual_trans, const geometry_msgs::Point& desired_trans);
	bool is_within_orientation_tolerance(const geometry_msgs::Quaternion& actual_orient, const geometry_msgs::Quaternion& desired_orient);
	bool is_type(std::string part_name, std::string part_type);
	void publish_incorrect_part(const int& freq);


	bool convert_pos_to_agv_1(const geometry_msgs::TransformStamped part_pos_logical_2, \
			std::string reference_frame);
	void incorrect_part_call_back(const tf2_msgs::TFMessage::ConstPtr& msg);

	bool is_part_in_order(const geometry_msgs::TransformStamped& test_part,\
			 const std::vector<osrf_gear::Order>& current_received_orders);

	bool check_parts_pos(part_perception::Incorrect_Part::Request& req, \
			part_perception::Incorrect_Part::Response& res);

	void server_data_call_back(const tf2_msgs::TFMessage::ConstPtr& msg);

};



#endif /* PART_PERCEPTION_INCLUDE_INCORRECT_POS_H_ */
