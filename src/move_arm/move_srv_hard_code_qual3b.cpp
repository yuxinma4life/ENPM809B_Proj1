

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Empty.h"
#include <osrf_gear/VacuumGripperState.h>
#include <tf/transform_listener.h>
#include <osrf_gear/VacuumGripperControl.h>

#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>

#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>

#include <move_arm/Pick.h>

//gripper offset
#include "tf2_msgs/TFMessage.h"
#include "part_perception/Part_Offset_Gripper.h"

#include <vector>
#include "math.h"

#include <osrf_gear/AGVControl.h>

tf::StampedTransform gripper_transform;
tf::StampedTransform tray_transform;
geometry_msgs::PoseStamped gripper_target;
geometry_msgs::Pose gripper_target_world;
geometry_msgs::Pose target_pose3;
osrf_gear::VacuumGripperControl srv;
std::vector<geometry_msgs::Pose> waypoints;
ros::Publisher joint_trajectory_publisher_;
float duration_time_ = 0.2;

sensor_msgs::JointState current_joint_states_;
trajectory_msgs::JointTrajectory msg;



moveit::planning_interface::MoveGroupInterface *group;
ros::ServiceClient client;
ros::ServiceClient part_perception_client;

//arm state variables
bool grabbing = false;
bool enabled = false;
bool attached = false;

double gripper_to_base_offset = -0.169946;


/// Start the competition by waiting for and then calling the start ROS Service.
void start_competition(ros::NodeHandle & node) {
	// Create a Service client for the correct service, i.e. '/ariac/start_competition'.
	ros::ServiceClient start_client =
	    node.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
	// If it's not already ready, wait for it to be ready.
	// Calling the Service using the client before the server is ready would fail.
	if (!start_client.exists()) {
		ROS_INFO("Waiting for the competition to be ready...");
		start_client.waitForExistence();
		ROS_INFO("Competition is now ready.");
	}
	ROS_INFO("Requesting competition start...");
	std_srvs::Trigger srv;  // Combination of the "request" and the "response".
	start_client.call(srv);  // Call the start Service.
	if (!srv.response.success) {  // If not successful, print out why.
		ROS_ERROR_STREAM("Failed to start the competition: " << srv.response.message);
	} else {
		ROS_INFO("Competition started!");
	}
}


void generate_gripper_target(float dx, float dy, float dz)
{
	gripper_target.header.frame_id = "world";
	gripper_target.pose.position.x = gripper_transform.getOrigin().x();
	gripper_target.pose.position.y = gripper_transform.getOrigin().y();
	gripper_target.pose.position.z = gripper_transform.getOrigin().z();
	gripper_target.pose.orientation.w = 0.707;
	gripper_target.pose.orientation.x = 0.0;
	gripper_target.pose.orientation.y = 0.707;
	gripper_target.pose.orientation.z = 0.0;


}



void pne(float t, float stepSize)
{
	group->setMaxVelocityScalingFactor(1.0);
	group->setMaxAccelerationScalingFactor(1.0);
	group->setPlanningTime(t);

	moveit_msgs::RobotTrajectory trajectory;
	double fraction = group->computeCartesianPath(waypoints,
	                  stepSize,  // eef_step
	                  0.0,   // jump_threshold
	                  trajectory);

	ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
	         fraction * 100.0);


	moveit::planning_interface::MoveGroup::Plan my_plan;
	my_plan.trajectory_ = trajectory;
	group->execute(my_plan);
}






void check_stable(float tolerance)

{
	int i = 0;
	bool isStable = false;
	while (!isStable) {
		isStable = true;
		ros::spinOnce();
		for (int j = 0; j < 6; j++) {
			// if(!(msg.points[0].positions[j]+tolerance > current_joint_states_.position[0] && msg.points[0].positions[j]-tolerance < current_joint_states_.position[0])){
			// 	isStable = false;
			// 	//ROS_INFO("position OK");
			// }
			if (std::abs(current_joint_states_.velocity[j]) > tolerance) {
				//ROS_INFO("speed  not ok: %d : %d",j,current_joint_states_.velocity[j]);
				isStable = false;
			}

		}
		i++;
		if (i > 1000) {
			isStable = true;
		}
		//ROS_INFO_STREAM("waiting for stable"<<current_joint_states_);
		ros::Duration(0.1).sleep();
	}
	//ROS_INFO_STREAM("done waiting"<<current_joint_states_);
	//ROS_INFO_STREAM("final i: "<<i);

}

void move_joints(float p0, float p1, float p2, float p3, float p4, float p5, float p6, float duration) {
	msg.joint_names.clear();
	msg.joint_names.push_back("elbow_joint");
	msg.joint_names.push_back("linear_arm_actuator_joint");
	msg.joint_names.push_back("shoulder_lift_joint");
	msg.joint_names.push_back("shoulder_pan_joint");
	msg.joint_names.push_back("wrist_1_joint");
	msg.joint_names.push_back("wrist_2_joint");
	msg.joint_names.push_back("wrist_3_joint");

	msg.points.resize(1);
	msg.points[0].positions = {p0, p1, p2, p3, p4, p5, p6};
	// How long to take getting to the point (floating point seconds).
	msg.points[0].time_from_start = ros::Duration(duration);
	msg.header.stamp = ros::Time::now() + ros::Duration();
	//ROS_INFO_STREAM("Sending command:\n" << msg);
	joint_trajectory_publisher_.publish(msg);
	ros::spinOnce();
}



void move_to(float x, float y, float z, float dyaw) {
	tf::TransformListener listener;
	listener.waitForTransform("/world", "/tool0", ros::Time(0), ros::Duration(10.0));
	listener.lookupTransform("/world", "/tool0", ros::Time(0), gripper_transform);
	ros::spinOnce();
	waypoints.clear();
	generate_gripper_target(0, 0, 0);
	geometry_msgs::Pose target_pose3 = gripper_target.pose;

	double safeHeight = 1.0f;
	double safeTrackFront = 1.0f;
	//before moving in x,y, make sure z is above certain height
	if (gripper_target.pose.position.z < safeHeight) {

		waypoints.push_back(target_pose3);
		target_pose3.position.z = safeHeight;
		waypoints.push_back(target_pose3);
		pne(15.0, 0.01);
		ros::spinOnce();
	}



	//move from mid to agv1 direction
	int i = 0;
	if (gripper_target.pose.position.y < 1.5 && gripper_target.pose.position.y > -1.5 && y > 1.5) {


		// Create a message to send.
		move_joints(2.1,
		            0,
		            -1.57,
		            3.14,
		            current_joint_states_.position[4],
		            current_joint_states_.position[5],
		            current_joint_states_.position[6],
		            1);

		check_stable(0.03);

		// Create a message to send.
		move_joints(2.1,
		            0,
		            -1.57,
		            1.57,
		            current_joint_states_.position[4],
		            current_joint_states_.position[5],
		            current_joint_states_.position[6],
		            1);
		i = 0;
		while (!(1.55 < current_joint_states_.position[3] && 1.59 > current_joint_states_.position[3]) && i < 1000) {
			//ROS_INFO("waitinf for arm move");
			ros::spinOnce();
			sleep(0.1);
			i++;
		}


		move_joints(current_joint_states_.position[0],
		            1.8,
		            current_joint_states_.position[2],
		            current_joint_states_.position[3],
		            current_joint_states_.position[4],
		            current_joint_states_.position[5],
		            current_joint_states_.position[6],
		            1);


		check_stable(0.03);


		move_joints(1.13,
		            2.0,
		            -0.5,
		            1.57,
		            current_joint_states_.position[4],
		            current_joint_states_.position[5],
		            current_joint_states_.position[6],
		            1);


		check_stable(0.03);
	}


	//back from agv1 to mid
	if (gripper_target.pose.position.y > 1.5 && y < 1.5 && y > -1.5)
	{
		move_joints(2.1,
		            0,
		            -1.57,
		            1.57,
		            current_joint_states_.position[4],
		            current_joint_states_.position[5],
		            current_joint_states_.position[6],
		            1);
		i = 0;
		while (current_joint_states_.position[1] > 0.5 && i < 1000) {
			//ROS_INFO("waitinf for arm move");
			ros::spinOnce();
			sleep(0.1);
			i++;
		}


		check_stable(0.05);

		move_joints(current_joint_states_.position[0],
		            0,
		            current_joint_states_.position[2],
		            3.14,
		            current_joint_states_.position[4],
		            current_joint_states_.position[5],
		            current_joint_states_.position[6],
		            1);


		check_stable(0.03);

	}


	/*
	*===================
	*2nd bin holding right
	*elbow_joint 2.25
	*linear_arm_actuator_joint 0.5
	*shoulder_lift_joint -0.75
	*shoulder_pan_joint 3.14159
	*wrist_1_joint 3.22
	*wrist_2_joint -1.57
	*wrist_3_joint 0.00
	*/

	//from camera to avg2
	if (gripper_target.pose.position.y > 1.5 && y < -1.5)
	{
		move_joints(2.1,
		            current_joint_states_.position[1],
		            -1.57,
		            1.57,
		            current_joint_states_.position[4],
		            current_joint_states_.position[5],
		            current_joint_states_.position[6],
		            1);
		ros::spinOnce();
		move_joints(2.1,
		            0,
		            -1.57,
		            3.14,
		            current_joint_states_.position[4],
		            current_joint_states_.position[5],
		            current_joint_states_.position[6],
		            1);
		i = 0;
		while (current_joint_states_.position[1] > 0.5 && i < 1000) {
			//ROS_INFO("waiting for arm move");
			ros::spinOnce();
			sleep(0.1);
			i++;
		}


		ros::spinOnce();
		check_stable(0.05);
		ros::spinOnce();
		move_joints(1.13,
		            0,
		            -0.5,
		            4.71,
		            current_joint_states_.position[4],
		            current_joint_states_.position[5],
		            current_joint_states_.position[6],
		            1);

		ros::spinOnce();
		check_stable(0.05);
		ros::spinOnce();

		move_joints(1.13,
		            -1.8,
		            -0.5,
		            4.71,
		            current_joint_states_.position[4],
		            current_joint_states_.position[5],
		            current_joint_states_.position[6],
		            1);


		check_stable(0.03);
		ros::spinOnce();

	}

	//go to agv2
	if (gripper_target.pose.position.y < 1.5 && gripper_target.pose.position.y > -1.5 && y < -1.5) {


		// Create a message to send.
		move_joints(2.1,
		            0,
		            -1.57,
		            3.14,
		            current_joint_states_.position[4],
		            current_joint_states_.position[5],
		            current_joint_states_.position[6],
		            1);

		check_stable(0.03);

		// Create a message to send.
		move_joints(2.1,
		            0,
		            -1.57,
		            4.71,
		            current_joint_states_.position[4],
		            current_joint_states_.position[5],
		            current_joint_states_.position[6],
		            1);
		i = 0;
		while (!(4.69 < current_joint_states_.position[3] && 4.73 > current_joint_states_.position[3]) && i < 1000) {
			//ROS_INFO("waitinf for arm move");
			sleep(0.1);
			ros::spinOnce();
			i++;
		}


		move_joints(current_joint_states_.position[0],
		            -1.8,
		            current_joint_states_.position[2],
		            current_joint_states_.position[3],
		            current_joint_states_.position[4],
		            current_joint_states_.position[5],
		            current_joint_states_.position[6],
		            1);


		check_stable(0.03);


		move_joints(1.13,
		            -2.0,
		            -0.5,
		            4.71,
		            current_joint_states_.position[4],
		            current_joint_states_.position[5],
		            current_joint_states_.position[6],
		            1);


		check_stable(0.03);
	}


	if (gripper_target.pose.position.y < -1.5 && y < 1.5 && y > -1.5)
	{
		move_joints(2.1,
		            0,
		            -1.57,
		            4.71,
		            current_joint_states_.position[4],
		            current_joint_states_.position[5],
		            current_joint_states_.position[6],
		            1);

		while (current_joint_states_.position[1] < - 0.5) {
			//ROS_INFO("waitinf for arm move");
			ros::spinOnce();
		}


		check_stable(0.05);

		move_joints(current_joint_states_.position[0],
		            0,
		            current_joint_states_.position[2],
		            3.14,
		            current_joint_states_.position[4],
		            current_joint_states_.position[5],
		            current_joint_states_.position[6],
		            1);


		check_stable(0.03);

	}





	check_stable(0.03);
	ros::spinOnce();

	listener.waitForTransform("/world", "/tool0", ros::Time(0), ros::Duration(10.0));
	listener.lookupTransform("/world", "/tool0", ros::Time(0), gripper_transform);
	ros::spinOnce();
	waypoints.clear();
	generate_gripper_target(0, 0, 0);
	target_pose3 = gripper_target.pose;
	waypoints.push_back(target_pose3);

	//do rotation
	tf::Quaternion q = tf::createQuaternionFromRPY(0, 1.57, dyaw);
	q.normalize();
	ROS_INFO("Quaternion: %f, %f, %f, %f", q[0], q[1], q[2], q[3]);

	target_pose3.orientation.w = q[3];
	target_pose3.orientation.x = q[0];
	target_pose3.orientation.y = q[1];
	target_pose3.orientation.z = q[2];


	//maintain height before dropping to avoid tray edge collision
	if (z < safeHeight) {
		target_pose3.position.x = x;
		target_pose3.position.y = y;
		target_pose3.position.z = safeHeight;
		waypoints.push_back(target_pose3);

	}

	target_pose3.position.x = x;
	target_pose3.position.y = y;
	target_pose3.position.z = z;
	waypoints.push_back(target_pose3);

	pne(15.0, 0.01);


}




/*
*===================
*pre belt pickup holding position
*elbow_joint 1.65
*linear_arm_actuator_joint 0.00
*shoulder_lift_joint -0.69
*shoulder_pan_joint 0.00
*wrist_1_joint 3.73
*wrist_2_joint -1.51
*wrist_3_joint 0.00
*/

void go_to_belt(float y) {
	tf::TransformListener listener;
	listener.waitForTransform("/world", "/tool0", ros::Time(0), ros::Duration(10.0));
	listener.lookupTransform("/world", "/tool0", ros::Time(0), gripper_transform);
	ros::spinOnce();

	y = y + gripper_to_base_offset;

	if (gripper_transform.getOrigin().y() > 1.5 || gripper_transform.getOrigin().y() < -1.5) {
		move_joints(2.1,
		            0,
		            -1.57,
		            current_joint_states_.position[3],
		            current_joint_states_.position[4],
		            current_joint_states_.position[5],
		            current_joint_states_.position[6],
		            1);
		while (!(-0.1 < current_joint_states_.position[1] && 0.1 > current_joint_states_.position[1])) {
			//ROS_INFO("waitinf for arm move");
			ros::spinOnce();
		}

	}



	//ROS_INFO("go to belt move");
	// // rotate towards
	move_joints(2.1,
	            0,
	            -1.57,
	            3.14,
	            current_joint_states_.position[4],
	            current_joint_states_.position[5],
	            current_joint_states_.position[6],
	            1);
	sleep(1.0);
	move_joints(2.1,
	            y,
	            -1.57,
	            0,
	            current_joint_states_.position[4],
	            current_joint_states_.position[5],
	            current_joint_states_.position[6],
	            1);
	sleep(1.0);
	move_joints(1.65,
	            y,
	            -0.69,
	            0.00,
	            3.73,
	            -1.51,
	            0.00,
	            1);
	check_stable(0.01);


}

void fast_pick_up() {
	srv.request.enable = true;
	client.call(srv);
	ros::spinOnce();

	move_joints(1.69,
	            current_joint_states_.position[1],
	            current_joint_states_.position[2],
	            current_joint_states_.position[3],
	            current_joint_states_.position[4],
	            current_joint_states_.position[5],
	            current_joint_states_.position[6],
	            0.1);
	sleep(1.0);
	move_joints(1.60,
	            current_joint_states_.position[1],
	            current_joint_states_.position[2],
	            current_joint_states_.position[3],
	            current_joint_states_.position[4],
	            current_joint_states_.position[5],
	            current_joint_states_.position[6],
	            0.1);

}


void fast_pick_up_at_time(double secs) {
	bool isTime = false;
	double now_secs = 0;
	secs = secs - 0.1;
	while (!isTime) {
		now_secs = ros::Time::now().toSec();
		if (now_secs > secs) {
			isTime = true;
		}
		sleep(0.1);
		ROS_INFO("waiting time: %f  now: %f", secs, now_secs);
	}


	srv.request.enable = true;
	client.call(srv);
	ros::spinOnce();

	move_joints(1.68,
	            current_joint_states_.position[1],
	            -0.665,
	            0.00,
	            3.72,
	            -1.57,
	            current_joint_states_.position[6],
	            0.1);
	sleep(1.0);
	move_joints(1.60,
	            current_joint_states_.position[1],
	            current_joint_states_.position[2],
	            current_joint_states_.position[3],
	            current_joint_states_.position[4],
	            -1.57,
	            current_joint_states_.position[6],
	            0.1);

}


/*
*===================
*2nd bin holding right
*elbow_joint 2.25
*linear_arm_actuator_joint 0.5
*shoulder_lift_joint -0.75
*shoulder_pan_joint 3.14159
*wrist_1_joint 3.22
*wrist_2_joint -1.57
*wrist_3_joint 0.00
*/
/*
*===================
*2nd bin holding right
*elbow_joint 2.25
*linear_arm_actuator_joint 0.5
*shoulder_lift_joint -0.75
*shoulder_pan_joint 3.14159
*wrist_1_joint 3.22
*wrist_2_joint 0.00
*wrist_3_joint 0.00
*/

void flip_part() {
	ros::spinOnce();

	//raise shoulder for clearance
	move_joints(-1.62,
	            2.0,
	            -1.57,
	            3.14,
	            5.78,
	            0,
	            0.0,
	            0.5);
	sleep(1.0);

	//flip facing the part to down the belt
	move_joints(-1.62,
	            2.0,
	            -2.45,
	            3.14,
	            5.78,
	            0,
	            0.0,
	            0.75);
	check_stable(0.05);
	sleep(1.0);

	srv.request.enable = false;
	client.call(srv);
	ros::spinOnce();
	sleep(2.0);

	//raise shoulder
	move_joints(-1.62,
	            2.0,
	            -1.57,
	            3.14,
	            5.78,
	            0,
	            0.0,
	            0.5);
	sleep(1.0);
	ros::spinOnce();

	//move down and flip
	move_joints(1.62,
	            0,
	            -1.57,
	            0,
	            3.22,
	            0,
	            0.00,
	            0.1);
	sleep(1.0);

	move_joints(1.62,
	            0,
	            -1.57,
	            0,
	            3.22,
	            0,
	            0.00,
	            0.3);
	sleep(1.0);

	move_joints(1.62,
	            0,
	            -0.69,
	            0,
	            4.0,
	            0,
	            0.00,
	            0.5);
	sleep(1.0);

	srv.request.enable = true;
	client.call(srv);
	ros::spinOnce();

	while (!attached) {
		sleep(0.1);
		ros::spinOnce();
	}

	move_joints(1.62,
	            0,
	            -1.57,
	            0,
	            4.0,
	            0,
	            0.00,
	            0.1);
	sleep(1.0);

	move_joints(1.62,
	            2.0,
	            -0.75,
	            0,
	            3.34,
	            -1.57,
	            0.00,
	            0.3);
	sleep(1.0);
	check_stable(0.05);



}


void go_to_camera() {
	tf::TransformListener listener;
	listener.waitForTransform("/world", "/tool0", ros::Time(0), ros::Duration(10.0));
	listener.lookupTransform("/world", "/tool0", ros::Time(0), gripper_transform);
	ros::spinOnce();
	waypoints.clear();
	generate_gripper_target(0, 0, 0);
	geometry_msgs::Pose target_pose3 = gripper_target.pose;

	double safeHeight = 1.0f;
	double safeTrackFront = 1.0f;
	//before moving in x,y, make sure z is above certain height
	if (gripper_target.pose.position.z < safeHeight) {

		waypoints.push_back(target_pose3);
		target_pose3.position.z = safeHeight;
		waypoints.push_back(target_pose3);
		pne(15.0, 0.01);
		ros::spinOnce();
	}


	ros::spinOnce();
	move_joints(current_joint_states_.position[0],
	            0.0,
	            current_joint_states_.position[2],
	            current_joint_states_.position[3],
	            current_joint_states_.position[4],
	            current_joint_states_.position[5],
	            current_joint_states_.position[6],
	            0.1);
	sleep(2.0);
	move_joints(2.25,
	            0.0,
	            -1.57,
	            3.14159,
	            3.22,
	            -1.57,
	            0.00,
	            0.3);
	sleep(2.0);
	move_joints(2.25,
	            0.0,
	            -1.57,
	            0,
	            3.22,
	            -1.57,
	            0.00,
	            0.3);
	sleep(2.0);
	move_joints(2.25,
	            2.0,
	            -1.57,
	            0,
	            3.22,
	            -1.57,
	            0.00,
	            0.3);
	sleep(1.0);

	move_joints(1.62,
	            2.0,
	            -0.75,
	            0,
	            3.34,
	            -1.57,
	            0.00,
	            0.3);
	sleep(1.0);
	check_stable(0.05);


	// listener.waitForTransform("/world", "/tool0", ros::Time(0), ros::Duration(10.0));
	// listener.lookupTransform("/world", "/tool0", ros::Time(0), gripper_transform);
	// ros::spinOnce();
	// waypoints.clear();
	// generate_gripper_target(0, 0, 0);
	// target_pose3 = gripper_target.pose;
	// waypoints.push_back(target_pose3);
	// target_pose3.position.z = 1.4f;
	// waypoints.push_back(target_pose3);
	// pne(15.0, 0.01);
	// ros::spinOnce();

}





void gripper_callback(const osrf_gear::VacuumGripperState msg)
{
	enabled = msg.enabled;
	attached = msg.attached;

}

bool check_release(float x, float y, float z, float tolerance) {
	tf::TransformListener listener;
	listener.waitForTransform("/world", "/tool0", ros::Time(0), ros::Duration(10.0));
	listener.lookupTransform("/world", "/tool0", ros::Time(0), gripper_transform);
	ros::spinOnce();
	x = std::abs(gripper_transform.getOrigin().x() - x);
	y = std::abs(gripper_transform.getOrigin().y() - y);
	z = std::abs(gripper_transform.getOrigin().z() - z);


	if (x <= tolerance && y <= tolerance && z <= tolerance) {
		ROS_INFO("tolerance passed: %f ,%f ,%f", x, y, z);
		return true;
	}



	return false;

}



// void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg){
// 	r1 = msg->pose.pose.position.x;
// 	r2 = msg->pose.pose.position.y;
// 	r3 = msg->pose.pose.position.z;
// 	tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
// 	tf::Matrix3x3 m(q);
// 	double roll, pitch;
// 	m.getRPY(roll, pitch, yaw);
// 	yaw = (yaw/(pi))*180.0;
// 	if(yaw <0) yaw+= 360.0;
// 	roll = (roll/(2*pi))*180.0;
// 	pitch = (pitch/(2*pi))*180.0;
//     //ROS_INFO("roll pitch yaw: %f %f %f", roll, pitch, yaw);
//     //ROS_INFO("yaw: %f",yaw);

// 	//v1 = msg->twist.twist.linear.x;
// 	//v2 = msg->twist.twist.linear.y;
// 	//v3 = msg->twist.twist.linear.z;
// 	// seconds = msg->header.stamp.sec + ((double)msg->header.stamp.nsec)/1000000000;
//     // ROS_INFO("[%f]", seconds);

// }

// tf::TransformListener listener;
// tf::StampedTransform camera_transform;
// listener.waitForTransform("/world", "/logical_camera_1_frame", ros::Time(0), ros::Duration(10.0));
// listener.lookupTransform("/world", "/logical_camera_1_frame", ros::Time(0), camera_transform);
// ros::spinOnce();

// // !!!!!!!!!!!!!hard coded camera location here
// float x = 1.24;
// float y = 1.24;
// float z = 1.24;

//if (std::abs(camera_transform.getOrigin().x() - x) > 0.01 ||


//returns relative x, y, z and dyaw for gripper to accomplish
geometry_msgs::Pose compute_offset_transform() {

	geometry_msgs::Pose p;
	p.position.x = 0;
	p.position.y = 0;
	p.position.z = 0;
	p.orientation.x = 0;
	p.orientation.y = 0;
	p.orientation.z = 0;
	p.orientation.w = 0;

	ROS_INFO("Computing Offset");
	if (!part_perception_client.exists()) {
		ROS_INFO("Part Perception Service not Started");
		part_perception_client.waitForExistence();
	}

	part_perception::Part_Offset_Gripper part_perception_srv;
	part_perception_srv.request.check_part_offset = true;
	part_perception_client.call(part_perception_srv);


	if (!part_perception_srv.response.part_offset_info.transforms.empty()) {
		float x = part_perception_srv.response.part_offset_info.transforms[0].transform.translation.x;
		float y = part_perception_srv.response.part_offset_info.transforms[0].transform.translation.y;
		float z = part_perception_srv.response.part_offset_info.transforms[0].transform.translation.z;
		if (std::abs(x) > 0.5 || std::abs(y) > 0.5 || std::abs(z) > 0.5) {
			ROS_INFO("Camera has moved!!!");

			bool ok = false;
			int i = 0;
			while (!ok && i < 1000) {
				part_perception_srv.request.check_part_offset = true;
				part_perception_client.call(part_perception_srv);
				x = part_perception_srv.response.part_offset_info.transforms[0].transform.translation.x;
				y = part_perception_srv.response.part_offset_info.transforms[0].transform.translation.y;
				z = part_perception_srv.response.part_offset_info.transforms[0].transform.translation.z;
				if (std::abs(x) < 0.5 && std::abs(y) < 0.5 && std::abs(z) < 0.5) {
					ok = true;
				}
				ROS_INFO("Trying to perceive: %d", i);
				i++;
				sleep(0.3);
			}

		}
		// ROS_INFO("Part Perception: %f",
		//          part_perception_srv.response.part_offset_info.transforms[0].transform.translation.x);
		tf::Quaternion q(part_perception_srv.response.part_offset_info.transforms[0].transform.rotation.x,
		                 part_perception_srv.response.part_offset_info.transforms[0].transform.rotation.y,
		                 part_perception_srv.response.part_offset_info.transforms[0].transform.rotation.z,
		                 part_perception_srv.response.part_offset_info.transforms[0].transform.rotation.w);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		ROS_INFO("Part Perception: x %f y %f z %f", part_perception_srv.response.part_offset_info.transforms[0].transform.translation.x,
		         part_perception_srv.response.part_offset_info.transforms[0].transform.translation.y,
		         part_perception_srv.response.part_offset_info.transforms[0].transform.translation.z);
		ROS_INFO("Part Perception: Roll %f Pitch %f Yaw %f", roll, pitch, yaw);

		p.orientation.x = roll;
		p.orientation.y = pitch;
		p.orientation.z = yaw;

		x = part_perception_srv.response.part_offset_info.transforms[0].transform.translation.x;
		y = part_perception_srv.response.part_offset_info.transforms[0].transform.translation.y;
		z = part_perception_srv.response.part_offset_info.transforms[0].transform.translation.z;


		p.position.x = -y * cos(yaw) + x * sin(yaw);
		p.position.y =  - y * sin(yaw) - x * cos(yaw);
		p.position.z = z * 2.0;

		ROS_INFO("offset ysin: %f", y * sin(yaw));
		ROS_INFO("offset xcos: %f", x * cos(yaw));







	} else {
		ROS_INFO("Gripper is holding nothing!");
	}

	return p;

}




bool add(move_arm::Pick::Request  &req, move_arm::Pick::Response &res)
{
	int i = 0;
	res.sum = 0;
	//ROS_INFO("move order received");
	if (req.mode == 1) {
		srv.request.enable = true;
		client.call(srv);

		ros::spinOnce();
		move_to(req.pose.position.x, req.pose.position.y, req.pose.position.z, 0);
		i = 0;
		while (!check_release(req.pose.position.x, req.pose.position.y, req.pose.position.z, 0.05f ) && i < 10000) {
			//ROS_INFO("waiting for arm to arrive");
			ros::spinOnce();
			sleep(0.1);
			i++;
		}
		check_stable(0.03);
	}


	if (req.mode == 2) {

		go_to_camera();


		if (std::abs(req.pose.orientation.x) > 0 || std::abs(req.pose.orientation.y) > 0) {
			flip_part();
		}
		check_stable(0.03);
		geometry_msgs::Pose p = compute_offset_transform();
		float x = req.pose.position.x + p.position.x;
		float y = req.pose.position.y + p.position.y;
		float z = req.pose.position.z + p.position.z;
		move_to(x, y, z, req.pose.orientation.z + p.orientation.z + 1.57);

		//ROS_INFO("not printing");
		ros::spinOnce();
		i = 0;
		while (!check_release(x, y, z, 0.01f ) && i < 10000) {
			//ROS_INFO("waiting for arm to arrive");
			ros::spinOnce();
			sleep(0.1);
			i++;
		}

		if (attached == false) {
			ROS_INFO("!!!!!!!!!!!part dropped!");
			res.sum = -1;
		}

		srv.request.enable = false;
		//ROS_INFO("calling gripper release service");
		client.call(srv);

		ros::spinOnce();
		i = 0;
		while (!check_release(x, y, z, 0.05f ) && i < 10000) {
			//ROS_INFO("waiting for arm to arrive");
			ros::spinOnce();
			sleep(0.1);
			i++;
		}
		check_stable(0.03);

	}

	if (req.mode == 5) {



		if (std::abs(req.pose.orientation.x) > 0 || std::abs(req.pose.orientation.y) > 0) {
			flip_part();
		}
		check_stable(0.03);
		geometry_msgs::Pose p = compute_offset_transform();
		float x = req.pose.position.x + p.position.x;
		float y = req.pose.position.y + p.position.y;
		float z = req.pose.position.z + p.position.z;
		move_to(x, y, z, req.pose.orientation.z + p.orientation.z + 1.57);

		//ROS_INFO("not printing");
		ros::spinOnce();
		i = 0;
		while (!check_release(x, y, z, 0.01f ) && i < 10000) {
			//ROS_INFO("waiting for arm to arrive");
			ros::spinOnce();
			sleep(0.1);
			i++;
		}

		if (attached == false) {
			ROS_INFO("!!!!!!!!!!!part dropped!");
			res.sum = -1;
		}

		srv.request.enable = false;
		//ROS_INFO("calling gripper release service");
		client.call(srv);

		ros::spinOnce();
		i = 0;
		while (!check_release(x, y, z, 0.05f ) && i < 10000) {
			//ROS_INFO("waiting for arm to arrive");
			ros::spinOnce();
			sleep(0.1);
			i++;
		}
		check_stable(0.03);

	}




	if (req.mode == 3) {
		go_to_belt(req.pose.position.y);
		fast_pick_up_at_time(req.future_time.toSec());
	}

	if (req.mode == 4) {
		srv.request.enable = true;
		client.call(srv);

		ros::spinOnce();
		move_to(req.pose.position.x, req.pose.position.y, req.pose.position.z, 0);
		while (!check_release(req.pose.position.x, req.pose.position.y, req.pose.position.z, 0.05f )) {
			//ROS_INFO("waiting for arm to arrive");
			ros::spinOnce();
			sleep(0.1);
		}

		move_to(req.pose.position.x, req.pose.position.y, req.pose.position.z + 0.2f, 0);
		while (!check_release(req.pose.position.x, req.pose.position.y, req.pose.position.z + 0.2, 0.05f )) {
			//ROS_INFO("waiting for arm to arrive");
			ros::spinOnce();
			sleep(0.1);
		}


		go_to_camera();
		compute_offset_transform();


	}


	return true;
}

void send_arm_to_zero_state() {
	// Create a message to send.
	trajectory_msgs::JointTrajectory msg;

	// Fill the names of the joints to be controlled.
	// Note that the vacuum_gripper_joint is not controllable.
	msg.joint_names.clear();
	msg.joint_names.push_back("elbow_joint");
	msg.joint_names.push_back("linear_arm_actuator_joint");
	msg.joint_names.push_back("shoulder_lift_joint");
	msg.joint_names.push_back("shoulder_pan_joint");
	msg.joint_names.push_back("wrist_1_joint");
	msg.joint_names.push_back("wrist_2_joint");
	msg.joint_names.push_back("wrist_3_joint");
	// Create one point in the trajectory.
	msg.points.resize(1);
	// Resize the vector to the same length as the joint names.
	// Values are initialized to 0.
	//msg.points[0].positions = {1.76, 0.48, -0.47, 3.23,3.58,-1.51,0.0};
	msg.points[0].positions = {1.56, 0.48, -1.13, 3.14, 3.58, -1.51, 0.0};
	// How long to take getting to the point (floating point seconds).
	msg.points[0].time_from_start = ros::Duration(duration_time_);
	msg.header.stamp = ros::Time::now() + ros::Duration();
	ROS_INFO_STREAM("Sending command:\n" << msg);
	joint_trajectory_publisher_.publish(msg);
}

/// Called when a new JointState message is received.
void joint_state_callback(
    const sensor_msgs::JointState::ConstPtr & joint_state_msg)
{

	current_joint_states_ = *joint_state_msg;

}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "move_arm");
	ros:: NodeHandle n;
	tf::TransformListener listener;
	client = n.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/gripper/control");
	part_perception_client = n.serviceClient<part_perception::Part_Offset_Gripper>("/ariac/check_part_offset");

	group = new moveit::planning_interface::MoveGroupInterface("manipulator");
	group->startStateMonitor();
	ros::Subscriber gripper_sub = n.subscribe("ariac/gripper/state", 1000, gripper_callback);
	ros::Subscriber joint_state_subscriber = n.subscribe("/ariac/joint_states", 10, joint_state_callback);

	//wait for specific time to start comp
	double secs = 15;
	double now_secs = 0;
	bool isTime = false;
	while (!isTime) {
		now_secs = ros::Time::now().toSec();
		if (now_secs > secs) {
			isTime = true;
		}
		sleep(0.1);
		ROS_INFO("waiting time: %f  now: %f", secs, now_secs);
	}


	start_competition(n);

	group->setGoalTolerance(0.02);

	ros::ServiceServer service = n.advertiseService("/move_arm/toPose", add);

	joint_trajectory_publisher_ = n.advertise<trajectory_msgs::JointTrajectory>(
	                                  "/ariac/arm/command", 10);

	ros::ServiceClient client_agv1 = n.serviceClient<osrf_gear::AGVControl>("/ariac/agv1");
    ros::ServiceClient client_agv2 = n.serviceClient<osrf_gear::AGVControl>("/ariac/agv2");
    osrf_gear::AGVControl Go_delivery;


	int current_step = 0;
	secs = secs + 1;
	ros::Rate loop_rate(100);
	while (ros::ok())
	{
		listener.waitForTransform("/world", "/vacuum_gripper_link", ros::Time(0), ros::Duration(10.0));
		listener.lookupTransform("/world", "/vacuum_gripper_link", ros::Time(0), gripper_transform);

		srv.request.enable = true;
		client.call(srv);
		ros::spinOnce();

		if (current_step == 0) {

			if (ros::Time::now().toSec() > (secs)) {
				srv.request.enable = true;
				client.call(srv);
				ros::spinOnce();
				//xyz: [0.1, -0.15, 0]
				//rpy: [0, 0, 'pi/2']


				move_to(-0.2, -1.23, 0.745, 0);
				while (!attached) {
					sleep(0.1);
					ros::spinOnce();
				}

				//fast_pick_up_at_time(40.922);
				go_to_camera();
				//flip_part();
				//[0.300, -3.300, 0.750]
				float tarx = 0.3 + 0.1;
				float tary = -3.3 + ( -0.15) + 0.2;
				float tarz = 0.9;
				float taryaw = 1.57;
				check_stable(0.03);
				geometry_msgs::Pose p = compute_offset_transform();
				float x = tarx + p.position.x;
				float y = tary + p.position.y;
				float z = tarz + p.position.z;
				move_to(x, y, z, taryaw + p.orientation.z + 1.57);

				while (!check_release(x, y, z, 0.01f )) {
					//ROS_INFO("waiting for arm to arrive");
					ros::spinOnce();
					sleep(0.1);
				}

				srv.request.enable = false;
				client.call(srv);



				current_step ++;
				ROS_INFO("code run");
				secs = ros::Time::now().toSec() + 1;
			}

		}


		if (current_step == 1) {

			if (ros::Time::now().toSec() > (secs)) {
				ROS_INFO("NOW executing step %d ...", current_step);
				srv.request.enable = true;
				client.call(srv);
				ros::spinOnce();
				//xyz: [0.1, -0.15, 0]
				//rpy: [0, 0, 'pi/2']


				go_to_belt(0.146168);
				fast_pick_up_at_time(44.013);


				go_to_camera();
				//flip_part();
				//[0.300, -3.300, 0.750]
				float tarx = 0.3 - 0.1;
				float tary = -3.3 + ( -0.15) + 0.2;
				float tarz = 0.9;
				float taryaw = 1.57;
				check_stable(0.03);
				geometry_msgs::Pose p = compute_offset_transform();
				float x = tarx + p.position.x;
				float y = tary - p.position.y;
				float z = tarz + p.position.z;
				move_to(x, y, z, taryaw + p.orientation.z + 1.57);

				while (!check_release(x, y, z, 0.01f )) {
					//ROS_INFO("waiting for arm to arrive");
					ros::spinOnce();
					sleep(0.1);
				}

				srv.request.enable = false;
				client.call(srv);




				ROS_INFO("code run");
				secs = ros::Time::now().toSec() + 1;
				current_step ++;
			}

		}


		if (current_step == 2) {

			if (ros::Time::now().toSec() > (secs)) {
				ROS_INFO("NOW executing step %d ...", current_step);
				srv.request.enable = true;
				client.call(srv);
				ros::spinOnce();
				// xyz: [0.1, 0.15, 0]
				//      		 rpy: [0, 0, 0]


				move_to(-0.100000, -0.335000, 0.745, 0);
				while (!attached) {
					sleep(0.1);
					ros::spinOnce();
				}

				//fast_pick_up_at_time(40.922);
				go_to_camera();
				//flip_part();
				//[0.300, -3.300, 0.750]
				float tarx = 0.3 + 0.1;
				float tary = -3.3 + ( 0.15) + 0.2;
				float tarz = 0.9;
				float taryaw = 0;
				check_stable(0.03);
				geometry_msgs::Pose p = compute_offset_transform();
				float x = tarx + p.position.x;
				float y = tary + p.position.y;
				float z = tarz + p.position.z;
				move_to(x, y, z, taryaw + p.orientation.z + 1.57);

				while (!check_release(x, y, z, 0.01f )) {
					//ROS_INFO("waiting for arm to arrive");
					ros::spinOnce();
					sleep(0.1);
				}

				srv.request.enable = false;
				client.call(srv);



				current_step ++;
				ROS_INFO("code run");
				secs = ros::Time::now().toSec() + 1;
			}

		}

		if (current_step == 3) {

			if (ros::Time::now().toSec() > (secs)) {
				ROS_INFO("NOW executing step %d ...", current_step);
				srv.request.enable = true;
				client.call(srv);
				ros::spinOnce();
				//xyz: [0.1, -0.15, 0]
				//rpy: [0, 0, 'pi/2']


				go_to_belt(-0.477030);
				fast_pick_up_at_time(86.729);


				go_to_camera();
				//flip_part();
				//[0.300, -3.300, 0.750]
				float tarx = 0.3 - 0.15;
				float tary = 3.3 - ( 0.15) - 0.2;
				float tarz = 0.9;
				float taryaw = 0;
				check_stable(0.03);
				geometry_msgs::Pose p = compute_offset_transform();
				float x = tarx + p.position.x;
				float y = tary + p.position.y;
				float z = tarz + p.position.z;
				move_to(x, y, z, taryaw + p.orientation.z + 1.57);

				while (!check_release(x, y, z, 0.01f )) {
					//ROS_INFO("waiting for arm to arrive");
					ros::spinOnce();
					sleep(0.1);
				}

				srv.request.enable = false;
				client.call(srv);




				ROS_INFO("code run");
				secs = ros::Time::now().toSec() + 1;
				current_step ++;
			}

		}

		if (current_step == 4) {

			if (ros::Time::now().toSec() > (secs)) {
				srv.request.enable = true;
				client.call(srv);
				ros::spinOnce();
				// xyz: [0.12, -0.2, 0]
				//rpy: ['pi', 0, 0]


				move_to(-0.150000, 0.845, 0.81, 0);
				while (!attached) {
					sleep(0.1);
					ros::spinOnce();
				}

				
				go_to_camera();
				flip_part();
				//[0.300, -3.300, 0.750]
				float tarx = 0.3 + 0.12;
				float tary = 3.3 - ( -0.2) - 0.2;
				float tarz = 1.2;
				float taryaw = 0;
				check_stable(0.03);
				geometry_msgs::Pose p = compute_offset_transform();
				float x = tarx + p.position.x;
				float y = tary + p.position.y;
				float z = tarz + p.position.z;
				move_to(x, y, z, taryaw + p.orientation.z + 1.57);

				while (!check_release(x, y, z, 0.01f )) {
					//ROS_INFO("waiting for arm to arrive");
					ros::spinOnce();
					sleep(0.1);
				}

				srv.request.enable = false;
				client.call(srv);



				current_step ++;
				ROS_INFO("code run");
				secs = ros::Time::now().toSec() + 1;
			}

		}

		if (current_step == 5) {

			if (ros::Time::now().toSec() > (secs)) {
				srv.request.enable = true;
				client.call(srv);
				ros::spinOnce();
				// xyz: [0.12, -0.2, 0]
				//rpy: ['pi', 0, 0]


				move_to(-0.450000, 0.845, 0.81, 0);
				while (!attached) {
					sleep(0.1);
					ros::spinOnce();
				}

				
				go_to_camera();
				flip_part();
				//[0.300, -3.300, 0.750]
				float tarx = 0.3 - 0.12;
				float tary = 3.3 - ( -0.2) - 0.2;
				float tarz = 1.2;
				float taryaw = 3.1415;
				check_stable(0.03);
				geometry_msgs::Pose p = compute_offset_transform();
				float x = tarx + p.position.x;
				float y = tary + p.position.y;
				float z = tarz + p.position.z;
				move_to(x, y, z, taryaw + p.orientation.z + 1.57);

				while (!check_release(x, y, z, 0.01f )) {
					//ROS_INFO("waiting for arm to arrive");
					ros::spinOnce();
					sleep(0.1);
				}

				srv.request.enable = false;
				client.call(srv);



				current_step ++;
				ROS_INFO("code run");
				secs = ros::Time::now().toSec() + 1;
			}

		}

		if (current_step == 6) {

			if (ros::Time::now().toSec() > (secs)) {
				ROS_INFO("NOW executing step %d ...", current_step);
				srv.request.enable = true;
				client.call(srv);
				ros::spinOnce();
				// xyz: [0.1, 0.15, 0]
				//      		 rpy: [0, 0, 0]


				move_to(-0.233333, -0.468333, 0.745, 0);
				while (!attached) {
					sleep(0.1);
					ros::spinOnce();
				}

				//fast_pick_up_at_time(40.922);
				go_to_camera();
				//flip_part();
				//[0.300, -3.300, 0.750]
				float tarx = 0.3 + 0.15;
				float tary = 3.3 - ( 0.15) - 0.2;
				float tarz = 0.9;
				float taryaw = 0;
				check_stable(0.03);
				geometry_msgs::Pose p = compute_offset_transform();
				float x = tarx + p.position.x;
				float y = tary + p.position.y;
				float z = tarz + p.position.z;
				move_to(x, y, z, taryaw + p.orientation.z + 1.57);

				while (!check_release(x, y, z, 0.01f )) {
					//ROS_INFO("waiting for arm to arrive");
					ros::spinOnce();
					sleep(0.1);
				}

				srv.request.enable = false;
				client.call(srv);



				current_step ++;
				ROS_INFO("code run");
				secs = ros::Time::now().toSec() + 1;
			}

		}

		if (current_step == 7) {

			if (ros::Time::now().toSec() > (secs)) {
				ROS_INFO("NOW executing step %d ...", current_step);
				srv.request.enable = true;
				client.call(srv);
				ros::spinOnce();
				// xyz: [0.1, 0.15, 0]
				//      		 rpy: [0, 0, 0]


				move_to(-0.366667, -0.601667, 0.744, 0);
				while (!attached) {
					sleep(0.1);
					ros::spinOnce();
				}

				//fast_pick_up_at_time(40.922);
				go_to_camera();
				//flip_part();
				//[0.300, -3.300, 0.750]
				float tarx = 0.3;
				float tary = 3.3 - ( 0.15) - 0.2;
				float tarz = 0.9;
				float taryaw = 0;
				check_stable(0.03);
				geometry_msgs::Pose p = compute_offset_transform();
				float x = tarx + p.position.x;
				float y = tary + p.position.y;
				float z = tarz + p.position.z;
				move_to(x, y, z, taryaw + p.orientation.z + 1.57);

				while (!check_release(x, y, z, 0.01f )) {
					//ROS_INFO("waiting for arm to arrive");
					ros::spinOnce();
					sleep(0.1);
				}

				srv.request.enable = false;
				client.call(srv);
				ros::spinOnce();


				current_step ++;
				ROS_INFO("code run");
				secs = ros::Time::now().toSec() + 1;
				sleep(2.0);
				Go_delivery.request.kit_type = "kit1";
				client_agv1.call(Go_delivery);
				ros::spinOnce();

			}

		}

		if (current_step == 8) {
			

			if (ros::Time::now().toSec() > (secs)) {
				ROS_INFO("NOW executing step %d ...", current_step);
				srv.request.enable = true;
				client.call(srv);
				ros::spinOnce();
				// xyz: [0.1, 0.15, 0]
				//      		 rpy: [0, 0, 0]


				move_to(-0.500000, -0.735000, 0.743, 0);
				while (!attached) {
					sleep(0.1);
					ros::spinOnce();
				}

				//fast_pick_up_at_time(40.922);
				go_to_camera();
				//flip_part();
				//[0.300, -3.300, 0.750]
				float tarx = 0.3 - 0.1;
				float tary = -3.3 + ( 0.15) + 0.2;
				float tarz = 0.9;
				float taryaw = 0;
				check_stable(0.03);
				geometry_msgs::Pose p = compute_offset_transform();
				float x = tarx + p.position.x;
				float y = tary + p.position.y;
				float z = tarz + p.position.z;
				move_to(x, y, z, taryaw + p.orientation.z + 1.57);

				while (!check_release(x, y, z, 0.01f )) {
					//ROS_INFO("waiting for arm to arrive");
					ros::spinOnce();
					sleep(0.1);
				}

				srv.request.enable = false;
				client.call(srv);

				ros::spinOnce();

				current_step ++;
				ROS_INFO("code run");
				secs = ros::Time::now().toSec() + 1;

				sleep(2.0);


				Go_delivery.request.kit_type = "kit0";
				client_agv2.call(Go_delivery);
				ros::spinOnce();
			}

		}




		ros::spinOnce();
		loop_rate.sleep();


	}
}

