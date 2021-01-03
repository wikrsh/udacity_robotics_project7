#define _USE_MATH_DEFINES
#include <cmath>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct GoalPose {
  double x;
  double y;
  double yaw;
};

const GoalPose pickup_pose = { 6.9, 0.4, 0 };
const GoalPose dropoff_pose = { -4.2, -2.2, M_PI };

move_base_msgs::MoveBaseGoal create_goal_msg(const GoalPose& pose) {
  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = pose.x;
  goal.target_pose.pose.position.y = pose.y;
  goal.target_pose.pose.orientation.z = sin(pose.yaw / 2.0);
  goal.target_pose.pose.orientation.w = cos(pose.yaw / 2.0);

  return goal;
}

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal pickup_goal = create_goal_msg(pickup_pose);

  // Send the pickup position and orientation for the robot to reach
  ROS_INFO("Sending pickup goal");
  ac.sendGoal(pickup_goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Arrived at pickup position");
  } else {
    ROS_INFO("Failed to move to pickup position for some reason");
  }

  // Wait 5 sec
  ROS_INFO("Waiting 5 sec");
  ros::Duration(5.0).sleep();

  move_base_msgs::MoveBaseGoal dropoff_goal = create_goal_msg(dropoff_pose);

  // Send the dropoff position and orientation for the robot to reach
  ROS_INFO("Sending dropoff goal");
  ac.sendGoal(dropoff_goal);
  
  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Arrived at dropoff position");
  } else {
    ROS_INFO("Failed to move to dropoff position for some reason");
  }

  return 0;
}
