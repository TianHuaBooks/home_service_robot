#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


// set goal
void set_goal(double x, double y, double z, double w, move_base_msgs::MoveBaseGoal& goal) ;

// send goal to move robot to a goal
void send_goal( MoveBaseClient& ac, move_base_msgs::MoveBaseGoal& goal) ;

// set goal
void set_goal(double x, double y, double z, double w, 
  	move_base_msgs::MoveBaseGoal& goal) 
{
  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  //goal.target_pose.pose.orientation.z = z;
  goal.target_pose.pose.orientation.w = w;
}

// send goal to move robot to a goal
void send_goal( MoveBaseClient& ac, move_base_msgs::MoveBaseGoal& goal) 
{
   // Send the goal position and orientation for the robot to reach
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved to a goal");
  else
    ROS_INFO("The base failed to move forward to a goal for some reason");
}

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal1;

   // Send the goal position and orientation for the robot to reach
  set_goal(2.5, 2.0, 0.6, 1.0, goal1) ;
  ROS_INFO("Sending goal 1");
  send_goal(ac, goal1);

  // pause for 5 sec
  ros::Duration(5.0).sleep();

  move_base_msgs::MoveBaseGoal goal2;
  set_goal(-5.0, 3.0, 0.7, 1.0, goal2) ;
  ROS_INFO("Sending goal 2");
  send_goal(ac, goal2);

  return 0;
}
