

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Empty.h"
#include <osrf_gear/VacuumGripperState.h>
#include <tf/transform_listener.h>
#include <osrf_gear/VacuumGripperControl.h>

tf::StampedTransform gripper_transform;
geometry_msgs::PoseStamped gripper_target;
osrf_gear::VacuumGripperControl srv;


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

void move_armCallback(const geometry_msgs::PoseStamped msg)
{
  
  //ROS_INFO_STREAM("!! End effector link: " << group->getCurrentPose(group->getEndEffectorLink()));
  ROS_INFO_STREAM("!! Moving to: " << msg);
  group->setPoseTarget(msg);

  moveit::planning_interface::MoveGroup::Plan my_plan;
  group->setPlanningTime(1.0);
  if(group->plan(my_plan))

  //ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  //if(success)
  group->move();
  
}

void generate_gripper_target()
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

void grab(const std_msgs::Empty msg)
{
	grabbing = true;
	ROS_INFO_STREAM("grabbing");
	ROS_INFO_STREAM("location x: " << gripper_transform.getOrigin().x());
	ROS_INFO_STREAM("location y: " << gripper_transform.getOrigin().y());
	ROS_INFO_STREAM("location z: " << gripper_transform.getOrigin().z());
  generate_gripper_target();
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


    if(grabbing && attached){
    	group->stop();
    	grabbing = false;
    	ROS_INFO_STREAM("grabbing success");
    }
	
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_arm");
  ros:: NodeHandle n;
  tf::TransformListener listener;
  client = n.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/gripper/control");
  
  //need another spinner to keep the node alive for robot state errors
  ros::AsyncSpinner spinner(1);
  spinner.start();

  group = new moveit::planning_interface::MoveGroupInterface("manipulator");
  group->startStateMonitor();
  //subscribe to /move_arm which is a PoseStamped msg
  ros::Subscriber sub = n.subscribe("move_arm", 1000, move_armCallback);
  ros::Subscriber grab_sub = n.subscribe("grab",1000, grab);
  ros::Subscriber gripper_sub = n.subscribe("ariac/gripper/state", 1000, gripper_callback);
  

  
  group->setGoalTolerance(0.005);



   
   
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




