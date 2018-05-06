/**
 * @brief This code is to detect all parts on the belt
 * 1. obtain the information about logical_camera_1 from rostopic /tf
 * 2. initialize database 'belt_inventory'as priority queue to store the position of each part on the belt
 * 3. push child_frame information to inventory if it does not exist on the database
 * 4. update the database using the timestamp of newest element and conveyer velocity
 * 5. pop the first element on the database if its out of range
 * @author Shaotu Jia
 */

#include <vector>
#include <ros/ros.h>
#include <math.h>
#include "tf2_msgs/TFMessage.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Header.h"
#include "Conveyor_Belt/parts_belt.h"
#include "part_perception/Inventory_Predication.h"
#include "part_perception/TwoInts.h"



/*
 * @breif this constructor function maintain belt_inventory and publish topic /belt_inventory
 * 1. initialize publisher to publish current_belt_inventory
 * 2. initialize server to predicate desired part position
 * @param node ros nodehandle
 */
Belt_Inventory::Belt_Inventory(ros::NodeHandle node):node(node) {

	belt_inventory_publisher = node.advertise<tf2_msgs::TFMessage>("/ariac/belt_inventory", 1000);

	// publish service server
	// predicate_inventory = node.advertiseService("part_predication", find_parts);
	// add_server = node.advertiseService("add_two_ints", add);

}


// callback function to compute belt velocity
// compute the difference of msg about same part on the belt
/**
 * @brief compute the belt velocity
 * @param msg the new message from rostopic /tf
 * 1. get the last element from 'belt_inventory' vector
 * 2. find same part in coming new msg
 * 3. compute belt_velocity : (new_y_position - old_y_position)/(new_timestamp - old_timestamp)
 * @result update  belt_velo variable
 */
void Belt_Inventory::belt_velo_compute(const tf2_msgs::TFMessage::ConstPtr& msg) {


	// send log message
	// ROS_INFO_STREAM("belt_velo_compute start !!" );

	// get last element from old vector belt_inventory
	auto old_last_element = belt_inventory.back();

	// initial belt velocity
	double belt_velo = 0;

	// time difference between new_timestamp and old_timestamp for same part
	double time_stamp_diff = 0;

	// y - position difference between new_y_poistion and old_y_position
	double position_diff = 0;

	// find same part in coming msg
	for (auto possible_part : msg->transforms) {

		if (possible_part.child_frame_id != old_last_element.child_frame_id) {
			break;
		} else {

				// time = stamp.sec + stamp.nsec/(10^9)
				auto possible_part_timestamp = possible_part.header.stamp.sec \
						+ possible_part.header.stamp.nsec/(pow(10, 9));

				// auto possible_part_timestamp = possible_part.header.stamp.toSec();

				auto old_last_element_timestamp = old_last_element.header.stamp.sec \
						+ old_last_element.header.stamp.nsec/(pow(10, 9));
				// auto old_last_element_timestamp = old_last_element.header.stamp.toSec();

				// set up the time stamp difference
				time_stamp_diff = possible_part_timestamp - old_last_element_timestamp;


				if (time_stamp_diff == 0) {

					ROS_INFO_STREAM(" No belt_velo update !!");
					break;
				} else {
					position_diff = possible_part.transform.translation.y \
							- old_last_element.transform.translation.y;

					 current_belt_velo = position_diff / time_stamp_diff;	// update current belt_velo
					 current_time_stamp_sec = possible_part.header.stamp.sec;			// update current time stamp
					 current_time_stamp_nsec = possible_part.header.stamp.nsec;

					 // compute average belt velocity
					 accumulate_velo += current_belt_velo;
					 velo_measure_times += 1;
					 average_belt_velo = accumulate_velo/velo_measure_times;

					// update last element in belt_inventory
					belt_inventory.back() = possible_part;
				}

		}
	}

}

// callback function part_detect filters information about parts
// from other transfromation in /tf
void Belt_Inventory:: part_detect(const tf2_msgs::TFMessage::ConstPtr& msg) {


	// send log message
	// ROS_INFO_STREAM("part_detect function invoked !!");

	for (auto possible_part : msg->transforms) {

		bool on_inventory = false;		// the part whether exist on belt_inventory

			// check if the logical_camera_1 in correct location of tf
		bool is_logical_camera_1_correct = is_logical_camera_1_correct_location(\
			logical_camera_1_location,logical_camera_incorrect_location_translation_tolerance);

		// filter part referrring to frame logical_camera_1_frame and also on the convey belt(function is_on_belt())
		if (possible_part.header.frame_id == "logical_camera_1_frame" \
				&& possible_part.child_frame_id != "logical_camera_1_kit_tray_1_frame" \
				&& possible_part.child_frame_id != "logical_camera_1_agv1_frame" \
				&& is_on_belt(possible_part, on_belt_upper_z_limit, on_belt_lower_z_limit)
				&& is_logical_camera_1_correct
				&& !(is_on_gripper(possible_part, gripper_frame, gripper_tol))) {
			for (auto& part : belt_inventory) {

				if (possible_part.child_frame_id == part.child_frame_id) {
					on_inventory = true;
					break;
				}
			}

			// if the possible_part does not exist in the belt inventory, add it to belt_inventory
			if (!on_inventory) {
				belt_inventory.push_back(possible_part);

				ROS_INFO_STREAM("Find part under logical camera");
				ROS_INFO_STREAM("The size of inventory is " << belt_inventory.size());
			}
		}
	}

}


/*
 * @brief This function removes the first part that is far away on the conveyer belt
 * @param distance; the distance between logical_camera_1 and part
 * @return void; but update belt_inventory vector
 */
void Belt_Inventory::remove_part_from_belt(const double& distance) {

	// ROS_INFO_STREAM("remove_part_from_belt invoked !!");

	// get the first element(farest element) from belt_inventory
	auto far_element = belt_inventory.begin();

	auto time = ros::Time::now();

	auto predicate_far_element = predicate_part_position(*far_element, time);

	if (predicate_far_element.transform.translation.y < distance) {

			ROS_INFO_STREAM("Farthest current part position y = " << \
					predicate_far_element.transform.translation.y);

			belt_inventory.erase(belt_inventory.begin());
			ROS_INFO_STREAM("REMOVE FAR Part");
		}

}

/**
 * @brief if part out of range return true; else return false
 */
bool Belt_Inventory::out_of_range(const geometry_msgs::TransformStamped& predicate_part) {

	if (predicate_part.transform.translation.y < farthest_distance) {
		return true;
	} else {
		return false;
	}
}


/**
 * @breif predicate the part position on the belt
 * @param part the info of part
 * @param time the time you want to need the position of part
 * @return predicated position and timestamp
 */

geometry_msgs::TransformStamped Belt_Inventory::predicate_part_position(const geometry_msgs::TransformStamped& part, ros::Time time) {

	// compute the time difference
	double duration = time.toSec() - part.header.stamp.toSec();

	// predicate_part
	geometry_msgs::TransformStamped predicate_part;

 	predicate_part.transform.translation.y = part.transform.translation.y - (fabs(average_belt_velo) * duration);

 	return predicate_part;
}



/**
 * @breif update part position in belt_inventory based on current belt_velo
 * @param  msg the new message from rostopic /tf
 * @detail
 * 1. compute y - position in current time
 * 2. update y - position and timestamp in belt_inventory vector
 * 3. remove part out of range (y - position exceed far_distance)
 *
 */
void Belt_Inventory::update_belt_inventory(std::vector<geometry_msgs::TransformStamped>& inventory_belt) {


	// send log message
	// ROS_INFO_STREAM("update_belt_inventory function invoked !!");

	auto Time = ros::Time::now();	// current time

	// assign belt_inventory to current_belt_inventory
	current_belt_inventory = convert_belt_inventory_type_to_publish(inventory_belt);

	// geometry_msgs::TransformStamped temp_part_info;

	 for (auto& part : current_belt_inventory.transforms) {

		 // compute the time difference
		 double duration = Time.toSec() - part.header.stamp.toSec();

		 // send messge
		 ROS_INFO_STREAM("duration = " << duration);

		 // update time
		 part.header.stamp = Time;


		 // update part position
		 part.transform.translation.y = part.transform.translation.y - (fabs(average_belt_velo) * duration);

	 }


}


/**
 * @brief convert data type from std::vector to tf2_msgs
 */
tf2_msgs::TFMessage Belt_Inventory::convert_belt_inventory_type_to_publish(const std::vector<geometry_msgs::TransformStamped>& inventory) {

	tf2_msgs::TFMessage publish_data;

	publish_data.transforms = inventory;


	return publish_data;

}




/**
 * @brief publish belt_inventory vector topic belt_inventory
 * 1. set up publisher
 */
void Belt_Inventory::publish_belt_inventory(const int& freq) {

	// send log message
	// ROS_INFO_STREAM(" publish_belt_inventory function Invoke !!");

	// convert belt_inventory to output type tf2_message
	// tf2_msgs::TFMessage publish_data = convert_belt_inventory_type_to_publish(belt_inventory);

	// output size of belt_inventory vector
	ROS_INFO_STREAM("Publish belt_inventory size = " << belt_inventory.size());

	// output size of publish data
	ROS_INFO_STREAM("Publish Size of publish data = " << current_belt_inventory.transforms.size());


	if (!current_belt_inventory.transforms.empty()) {
			 belt_inventory_publisher.publish(current_belt_inventory);
	}



}




/**
 * @brief combine all functions in this class to build belt inventory
 * @param msg the new from rostopic /tf
 * 1. receive msg from /tf topic using callback function
 * 2. detect part from belt using function 'part_detect'
 * 3. compute belt_velo using belt_velo_compute if belt_inventory is not empty
 * 4. remove unpickable part from belt_inventory list using 'remove_part_from_belt'
 * 5. update the position of part in belt_inventory using 'update_belt_inventory'
 * 6. publish a topic 'belt_inventory' to ros master
 *
 */
void Belt_Inventory::build_belt_inventory(const tf2_msgs::TFMessage::ConstPtr& msg) {


	auto recieved_msg = msg;

	// detect part from belt using function 'part_detect'
	part_detect(recieved_msg);

	// if belt_inventory is not empty
	if (!belt_inventory.empty()) {

		// compute belt_velo
		belt_velo_compute(recieved_msg);

		// update the position of part in belt_inventory using 'update_belt_inventory'
		update_belt_inventory(belt_inventory);

		// remove unpickable part from belt_inventory list using 'remove_part_from_belt'
		remove_part_from_belt(farthest_distance);

		 // publish belt_inventory vector
		publish_belt_inventory(publish_freq);

	} else {

		// cleare current_belt Inventory if no parts are available for picking up
		current_belt_inventory.transforms.clear();
	}


	// publish service server
	// predicate_inventory = node.advertiseService("part_predication", find_parts);
	// add_server = node.advertiseService("add_two_ints", add);

}

/**
 * @brief find desired part in belt_inventory and predicate its future position
 */
bool Belt_Inventory::find_parts(part_perception::Inventory_Predication::Request& req, \
		part_perception::Inventory_Predication::Response& res) {

	// assign belt_inventory to predicate_belt_inventory
	predicate_belt_inventory = convert_belt_inventory_type_to_publish(belt_inventory);

	// desired parts
	tf2_msgs::TFMessage desired_parts;


	for (auto it = predicate_belt_inventory.transforms.begin(); \
			it != predicate_belt_inventory.transforms.end(); it ++) {

		// the time difference between future time and part timestamp
		double time_diff = req.future_time.toSec() - it->header.stamp.toSec();

		it ->transform.translation.y -= (fabs(average_belt_velo) * time_diff);

		// if the predicate y - position bigger than farthest distance, keep this element
		if (it ->transform.translation.y > farthest_distance && \
				is_type(it ->child_frame_id, req.part_type)) {

			//assign query time to part info in future time
			it ->header.stamp = req.future_time;

			// push valid part to desired_part vector for output
			desired_parts.transforms.push_back(*it);
		}

	}

	if (!desired_parts.transforms.empty()) {
		res.parts_info = desired_parts;
		res.success = true;
		res.message = "Find part " + req.part_type + "! \n";
	} else {
		res.success = false;
		res.message = "Cannot find part" + req.part_type + "! \n";
	}

	return true;

}

/**
 * @brief check if the part is desired part type
 * @param part_name The full name of part
 * @param part_type The name of desired type
 * @return true if the part is desired part; else return false
 */
bool Belt_Inventory::is_type(std::string part_name, std::string part_type) {

	ROS_INFO_STREAM("part_name: " << part_name);
	ROS_INFO_STREAM("part_type: " << part_type);

	if (part_name.find(part_type) != std::string::npos) {
		return true;
	} else {
		return false;
	}
}

/**
 * @brief check if the part is on the belt under logical camera 1
 * @param part The detected under logical_camera_1
 * @param upper_bound The upper bound of a part being on belt
 * @param lower_bound The lower bound of a part being on belt
 * @return true if the part is on the belt; else return false
 */
bool Belt_Inventory::is_on_belt(const geometry_msgs::TransformStamped part, \
		const double& upper_bound, const double& lower_bound) {

	ROS_INFO_STREAM("!! invoke is_on_belt function");

	if ((part.transform.translation.z < upper_bound) && (part.transform.translation.z > lower_bound)) {
		ROS_INFO_STREAM("detect_part z = " << part.transform.translation.z);
		return true;
	} else {
		return false;
	}

}

/**
 * !!! Test Service
 */
bool Belt_Inventory::add(part_perception::TwoInts::Request  &req,
		part_perception::TwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("  sending back response: [%ld]", (long int)res.sum);
  return true;
}

/**
 * @brief check whether logical_camera_1 is in the correct position
 * @param the tolerance of logical_camera location incorrect
 * @return bool; true if within tolerance; otherwise false
 */
bool Belt_Inventory::is_logical_camera_1_correct_location(const geometry_msgs::Pose& config_Pose, \
		const double& tolerance) {

	// location of logical_camera referring to /world frame
	tf::StampedTransform relative_transform;


	// wait for transform in 0.1 sec
	logical_camera_listener.waitForTransform(world_frame, logical_camera_1_frame, ros::Time(0), ros::Duration(0.1));

	// listen transform between transform
	logical_camera_listener.lookupTransform(world_frame, logical_camera_1_frame, ros::Time(0), relative_transform);


	// compare to logical_camera_1 to the configuration position
	if (is_desired_Pos(config_Pose, relative_transform, tolerance)) {

		return true;
	} else {
		return false;
	}


}


/**
 * @brief set up the location of logical_camera_1 referring to world
 * @param x;
 * @param y;
 * @param z;
 */
geometry_msgs::Pose Belt_Inventory::set_up_pose(const double& x, const double& y, const double& z) {

	// initialize location in type geometry_msgs::Pose
	geometry_msgs::Pose location;

	// assign value for logical_camera_1_location position
	location.position.x = x;
	location.position.y = y;
	location.position.z = z;

	// return location;
	return location;

}

/**
 * @brief check whether in actual Pos within the translation_tolerance
 * @param desired_Pos; the desired Pos
 * @param actual_Pos; the actual Pos
 * @param translation_tolerance;  the tolerance in translation
 * @return true if every actual value is within the tolerance; else return false
 * @warning: we only check the tolerance in translation; No check for orientation
 */
bool Belt_Inventory::is_desired_Pos(const geometry_msgs::Pose& desired_Pos, const tf::StampedTransform& actual_Pos, \
			const double& translation_tolerance) {

	// get coordinate of actual location
	double actual_x = actual_Pos.getOrigin().x();
	double actual_y = actual_Pos.getOrigin().y();
	double actual_z = actual_Pos.getOrigin().z();

	// get coordinate of desired location
	double desired_x = desired_Pos.position.x;
	double desired_y = desired_Pos.position.y;
	double desired_z = desired_Pos.position.z;

	// return true if every actual value is within the tolerance; else return false
	if (is_within_tolerance(desired_x, actual_x, translation_tolerance) \
			&& is_within_tolerance(desired_y, actual_y, translation_tolerance) \
			&& is_within_tolerance(desired_z, actual_z, translation_tolerance)) {

		return true;
	} else {
		return false;
	}

}

/**
 * @brief check whether the actual_value is within the tolerance of configure value
 * @param config_value;	 the value we configured in configure file
 * @param actual_value; the actual value we get in simulation
 * @param tolerance; the tolerance that we can accept
 * @return bool; true if within the tolerance; else, false;
 */
bool Belt_Inventory::is_within_tolerance(const double& desired_value, const double& actual_value, \
		const double& tolerance) {

	// the difference between config_value and actual_value in absolute value
	double diff = std::abs(desired_value - actual_value);

	// check whether the diff smaller than the tolerance
	if (diff < tolerance) {
		return true;
	} else {
		return false;
	}
}

/**
 * @brief check whether part is attached to gripper
 */
bool Belt_Inventory::is_on_gripper(geometry_msgs::TransformStamped part, std::string gripper_frame, const double& tolerance) {

		// location of logical_camera referring to /world frame
	tf::StampedTransform relative_transform;


	// wait for transform in 0.1 sec
	logical_camera_listener.waitForTransform(gripper_frame, part.child_frame_id, ros::Time(0), ros::Duration(0.5));

	// listen transform between transform
	logical_camera_listener.lookupTransform(gripper_frame, part.child_frame_id, ros::Time(0), relative_transform);


	double x_diff = fabs(relative_transform.getOrigin().getX());
	double y_diff = fabs(relative_transform.getOrigin().getY());
	double z_diff = fabs(relative_transform.getOrigin().getZ());


	ROS_INFO_STREAM(part.child_frame_id <<" x_diff = " << x_diff);
	ROS_INFO_STREAM(part.child_frame_id << "y_diff = " << y_diff);
	ROS_INFO_STREAM(part.child_frame_id << "z_diff = " << z_diff);

	// compare to logical_camera_1 to the configuration position
	if ((x_diff < tolerance) && (y_diff < tolerance) && (z_diff < tolerance)) {

		ROS_INFO_STREAM("This part is attached on gripper");

		return true;
	} else {

		return false;
	}

}
