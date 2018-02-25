

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include "geometry_msgs/PoseStamped.h"


/*
This mode takes in object world locations and moves the manipulator to that location

rostopic pub -1 /move_arm geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "world"}, 
pose: {position: {x: -0.3, y: 0.43, z: 0.9}, orientation: {w:  0.707, x: 0.0, y: 0.707, z: 0}}}'


*/


moveit::planning_interface::MoveGroup *group;

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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_arm");
  ros:: NodeHandle n;
  
  //need another spinner to keep the node alive for robot state errors
  ros::AsyncSpinner spinner(1);
  spinner.start();

  group = new moveit::planning_interface::MoveGroup("manipulator");
  //subscribe to /move_arm which is a PoseStamped msg
  ros::Subscriber sub = n.subscribe("move_arm", 1000, move_armCallback);
  

  group->startStateMonitor();
  group->setGoalTolerance(0.04);

   
   
   //keep looping
  ros::Rate loop_rate(100);
   while (ros::ok())
  {
  	ros::spinOnce();
  	loop_rate.sleep();

  
   }
}




