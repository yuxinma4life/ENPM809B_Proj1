

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

#include <vector>





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



/*
This mode takes in object world locations and moves the manipulator to that location

rostopic pub -1 /move_arm geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "world"}, 
pose: {position: {x: -0.3, y: 0.43, z: 0.9}, orientation: {w:  0.707, x: 0.0, y: 0.707, z: 0}}}'


*/


moveit::planning_interface::MoveGroupInterface *group;
ros::ServiceClient client;

//arm state variables
bool grabbing = false;
bool enabled = false;
bool attached = false;


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

void generate_gripper_target_absolute(float x, float y, float z)
{

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

void pick()
{
	tf::TransformListener listener;
	listener.waitForTransform("/world","/tool0",ros::Time(0),ros::Duration(10.0));
	listener.lookupTransform("/world","/tool0",ros::Time(0), gripper_transform);
	srv.request.enable = true;
	client.call(srv);
	ros::spinOnce();
	waypoints.clear();

	generate_gripper_target(0,0,0);
	geometry_msgs::Pose target_pose3 = gripper_target.pose;
	waypoints.push_back(target_pose3); 

	target_pose3.position.z = 0.745;
	waypoints.push_back(target_pose3); 
	group->setMaxVelocityScalingFactor(0.00001);
	pne(15.0,0.001);

	int i = 0;
	while(!attached){
		ros::spinOnce();
		ROS_INFO("waiting to attach");
		sleep(0.1);
		i++;
		if(i>100) break;
	}

	listener;
	listener.waitForTransform("/world","/tool0",ros::Time(0),ros::Duration(10.0));
	listener.lookupTransform("/world","/tool0",ros::Time(0), gripper_transform);

	ros::spinOnce();
	waypoints.clear();

	generate_gripper_target(0,0,0);
	target_pose3 = gripper_target.pose;
	waypoints.push_back(target_pose3); 

	target_pose3.position.z = 0.9;
	waypoints.push_back(target_pose3); 
	group->setMaxVelocityScalingFactor(0.00001);
	pne(15.0,0.1);

}

//geometry_msgs::PoseStamped offset
void to_robot(float x, float y)
{
	tf::TransformListener listener;
	listener.waitForTransform("/world","/tool0",ros::Time(0),ros::Duration(10.0));
	listener.lookupTransform("/world","/tool0",ros::Time(0), gripper_transform);
	listener.waitForTransform("/world","/agv1_load_point_frame",ros::Time(0),ros::Duration(10.0));
	listener.lookupTransform("/world","/agv1_load_point_frame",ros::Time(0), tray_transform);

	waypoints.clear();
	generate_gripper_target(0,0,0);
	geometry_msgs::Pose target_pose3 = gripper_target.pose;
	waypoints.push_back(target_pose3); 

	target_pose3.position.x = -0.1;
	target_pose3.position.y = 2.5;
	target_pose3.position.z = 0.9;
	waypoints.push_back(target_pose3); //parking
	target_pose3.position.x = tray_transform.getOrigin().x()+ x;
	target_pose3.position.y = tray_transform.getOrigin().y() + y;
	target_pose3.position.z = 0.9;
	waypoints.push_back(target_pose3);  // left

	pne(15.0,0.1);

}

void to_bin_slot(int b, int s)
{
	tf::TransformListener listener;
	listener.waitForTransform("/world","/tool0",ros::Time(0),ros::Duration(10.0));
	listener.lookupTransform("/world","/tool0",ros::Time(0), gripper_transform);
	srv.request.enable = true;
	client.call(srv);
	ros::spinOnce();
	waypoints.clear();
	generate_gripper_target(0,0,0);
	geometry_msgs::Pose target_pose3 = gripper_target.pose;
	waypoints.push_back(target_pose3); 

	switch(b){
		case 7: target_pose3.position.x = -0.1;
		target_pose3.position.y = 0.43;
		target_pose3.position.z = 0.9;
		waypoints.push_back(target_pose3);
		ROS_INFO("going to bin %d", b);
		break;
		case 6:
		target_pose3.position.x = -0.1;
		target_pose3.position.y = -0.335;
		target_pose3.position.z = 0.9;
		waypoints.push_back(target_pose3);
		ROS_INFO("going to bin %d", b);
		break;
	}
	pne(15.0,0.01);
}



void check_stable(float tolerance)

{
	int i = 0;
	bool isStable = false;
	while(!isStable){
		isStable = true;
		ros::spinOnce();
		for(int j = 0; j<6; j++){
			// if(!(msg.points[0].positions[j]+tolerance > current_joint_states_.position[0] && msg.points[0].positions[j]-tolerance < current_joint_states_.position[0])){
			// 	isStable = false;
			// 	//ROS_INFO("position OK");
			// }
			if(std::abs(current_joint_states_.velocity[j]) > tolerance){
				ROS_INFO("speed  not ok: %d : %d",j,current_joint_states_.velocity[j]);
				isStable = false;
			}
			
		}
		i++;
		if(i>1000){
			isStable = true;
		}
		//ROS_INFO_STREAM("waiting for stable"<<current_joint_states_);
		ros::Duration(0.1).sleep();
	}
	ROS_INFO_STREAM("done waiting"<<current_joint_states_);
	ROS_INFO_STREAM("final i: "<<i);

}

void move_to(float x, float y, float z){
	tf::TransformListener listener;
	listener.waitForTransform("/world","/tool0",ros::Time(0),ros::Duration(10.0));
	listener.lookupTransform("/world","/tool0",ros::Time(0), gripper_transform);
	
	//REMOVE THIS!!!!!
	// srv.request.enable = true;
	// client.call(srv);
	
	ros::spinOnce();
	waypoints.clear();
	generate_gripper_target(0,0,0);
	geometry_msgs::Pose target_pose3 = gripper_target.pose;
	waypoints.push_back(target_pose3); 
	double safeHeight = 1.0f;
	double safeTrackFront = 1.0f;
	//before moving in x,y, make sure z is above certain height
	if(gripper_target.pose.position.z < safeHeight){
		target_pose3.position.z = safeHeight;
		waypoints.push_back(target_pose3);
		pne(15.0,0.01);
		ros::spinOnce();
	}

	





	if(gripper_target.pose.position.y < 1.5 && y > 1.5){

			 // Create a message to send.


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

		msg.points[0].positions = {current_joint_states_.position[0], 
			-0.5, 
			current_joint_states_.position[2], 
			1.57,
			current_joint_states_.position[4],
			current_joint_states_.position[5],
			current_joint_states_.position[6]};
		// How long to take getting to the point (floating point seconds).
			msg.points[0].time_from_start = ros::Duration(duration_time_);
			msg.header.stamp = ros::Time::now() + ros::Duration();
		//ROS_INFO_STREAM("Sending command:\n" << msg);
			joint_trajectory_publisher_.publish(msg);

			while(!(1.55< current_joint_states_.position[3] && 1.59> current_joint_states_.position[3])){
			//ROS_INFO("waitinf for arm move");
				ros::spinOnce();
			}


			ros::spinOnce();
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

			msg.points[0].positions = {current_joint_states_.position[0], 
				1.7, 
				current_joint_states_.position[2], 
				current_joint_states_.position[3],
				current_joint_states_.position[4],
				current_joint_states_.position[5],
				current_joint_states_.position[6]};
		// How long to take getting to the point (floating point seconds).
				msg.points[0].time_from_start = ros::Duration(duration_time_);
				msg.header.stamp = ros::Time::now() + ros::Duration();
		//ROS_INFO_STREAM("Sending command:\n" << msg);
				joint_trajectory_publisher_.publish(msg);
				while(!(1.55< current_joint_states_.position[3] && 1.59> current_joint_states_.position[3])){
			ROS_INFO("waiting for arm move");
					ros::spinOnce();
				}
				check_stable(0.05);
			}
			if(gripper_target.pose.position.y > 1.5 && y < 1.5)
			{
				ros::spinOnce();
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

				msg.points[0].positions = {current_joint_states_.position[0], 
					-0.5, 
					current_joint_states_.position[2], 
					current_joint_states_.position[3],
					current_joint_states_.position[4],
					current_joint_states_.position[5],
					current_joint_states_.position[6]};
		// How long to take getting to the point (floating point seconds).
					msg.points[0].time_from_start = ros::Duration(duration_time_);
					msg.header.stamp = ros::Time::now() + ros::Duration();
		//ROS_INFO_STREAM("Sending command:\n" << msg);
					joint_trajectory_publisher_.publish(msg);
					while(!(1.55< current_joint_states_.position[3] && 1.59> current_joint_states_.position[3])){
			//ROS_INFO("waitinf for arm move");
						ros::spinOnce();
					}
					check_stable(0.05);

				}




				listener.waitForTransform("/world","/tool0",ros::Time(0),ros::Duration(10.0));
				listener.lookupTransform("/world","/tool0",ros::Time(0), gripper_transform);
				ros::spinOnce();
				waypoints.clear();
				generate_gripper_target(0,0,0);
				target_pose3 = gripper_target.pose;
				waypoints.push_back(target_pose3); 


		//maintain height before dropping to avoid tray edge collision
				if(z < safeHeight){
					target_pose3.position.x = x;
					target_pose3.position.y = y;
					target_pose3.position.z = safeHeight;
					waypoints.push_back(target_pose3);

				}

				target_pose3.position.x = x;
				target_pose3.position.y = y;
				target_pose3.position.z = z;
				waypoints.push_back(target_pose3);

				pne(15.0,0.01);


			}





void grab(const std_msgs::Empty msg)
{
	grabbing = true;
	ROS_INFO_STREAM("grabbing");
	ROS_INFO_STREAM("location x: " << gripper_transform.getOrigin().x());
	ROS_INFO_STREAM("location y: " << gripper_transform.getOrigin().y());
	ROS_INFO_STREAM("location z: " << gripper_transform.getOrigin().z());
	generate_gripper_target(0,0,0);
	gripper_target.pose.position.z = gripper_transform.getOrigin().z()-0.01;
	srv.request.enable = true;
	client.call(srv);
	

	moveit::planning_interface::MoveGroup::Plan my_plan;
	group->setPlanningTime(1.0);
	group->setPoseTarget(gripper_target);
	if(group->plan(my_plan))  group->move();
	

	
}



void gripper_callback(const osrf_gear::VacuumGripperState msg)
{
	enabled = msg.enabled;
	attached = msg.attached;
	
}

bool check_release(float x, float y, float tolerance){
	tf::TransformListener listener;
	listener.waitForTransform("/world","/tool0",ros::Time(0),ros::Duration(10.0));
	listener.lookupTransform("/world","/tool0",ros::Time(0), gripper_transform);
	ros::spinOnce();
	x = std::abs(gripper_transform.getOrigin().x() - x);
	y = std::abs(gripper_transform.getOrigin().y() - y);
	if(x <= tolerance && y <= tolerance){
		return true;
	}

	return false;

}

bool add(move_arm::Pick::Request  &req, move_arm::Pick::Response &res)
{
	
	res.sum = 0;
  //ROS_INFO("%d",req.pose.position.x);
	if(req.mode == 1){
		srv.request.enable = true;
		client.call(srv);
	
		ros::spinOnce();
		move_to(req.pose.position.x,req.pose.position.y,req.pose.position.z);
	}

	
	if(req.mode == 2){
		move_to(req.pose.position.x,req.pose.position.y,req.pose.position.z);
		
		
		while(!check_release(req.pose.position.x,req.pose.position.y, 0.02f )){
			ROS_INFO("waiting for arm to arrive");
		}

		srv.request.enable = false;
		ROS_INFO("calling gripper release service");
		client.call(srv);
		
		ros::spinOnce();

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
	msg.points[0].positions = {1.56, 0.48, -1.13, 3.14,3.58,-1.51,0.0};
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


	group = new moveit::planning_interface::MoveGroupInterface("manipulator");
	group->startStateMonitor();
  //subscribe to /move_arm which is a PoseStamped msg
	ros::Subscriber grab_sub = n.subscribe("grab",1000, grab);
	ros::Subscriber gripper_sub = n.subscribe("ariac/gripper/state", 1000, gripper_callback);
	ros::Subscriber joint_state_subscriber = n.subscribe("/ariac/joint_states", 10,joint_state_callback);



	start_competition(n);

	group->setGoalTolerance(0.02);

	ros::ServiceServer service = n.advertiseService("/move_arm/toPose", add);

	joint_trajectory_publisher_ = n.advertise<trajectory_msgs::JointTrajectory>(
		"/ariac/arm/command", 10);


   //keep looping
	ros::Rate loop_rate(100);
	while (ros::ok())
	{
		listener.waitForTransform("/world","/vacuum_gripper_link",ros::Time(0),ros::Duration(10.0));
		listener.lookupTransform("/world","/vacuum_gripper_link",ros::Time(0), gripper_transform);


		ros::spinOnce();
		loop_rate.sleep();


	}
}




