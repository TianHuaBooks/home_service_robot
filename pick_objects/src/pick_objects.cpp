#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


// set goal
void set_goal(double x, double y, double w, move_base_msgs::MoveBaseGoal& goal) ;

// send goal to move robot to 
bool send_goal( MoveBaseClient& ac, move_base_msgs::MoveBaseGoal& goal) ;

// set goal
void set_goal(geometry_msgs::Pose& pose, move_base_msgs::MoveBaseGoal& goal) 
{
  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = pose.position.x;
  goal.target_pose.pose.position.y = pose.position.y;
  goal.target_pose.pose.orientation.w = pose.orientation.w;
}

// send goal to move robot to 
bool send_goal( MoveBaseClient& ac, move_base_msgs::MoveBaseGoal& goal) 
{
  bool ret = false;
   // Send the goal position and orientation for the robot to reach
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, the base moved to a goal");
    ret = true;
  } else
    ROS_INFO("The base failed to move forward to a goal for some reason");

  return ret;
}

void set_pose(geometry_msgs::Pose& pose, double x, double y, double w) {
	pose.position.x = x;
	pose.position.y = y;
  	pose.orientation.w = w;
}

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  // create a publisher
  ros::NodeHandle n;
  //ros::Rate r(1);
  ros::Publisher goal_pub = n.advertise<geometry_msgs::Pose>("/mygoal", 1);

  while (goal_pub.getNumSubscribers() < 1)
  {
      if (!ros::ok())
        return 0;
      ROS_WARN_ONCE("Please create a subscriber to mygoal!");
      sleep(1);
  }

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal_pickup;

  // Send the pickup goal position and orientation for the robot to reach
  geometry_msgs::Pose pick_up_pose;
  set_pose(pick_up_pose, 3.5, 3.5, 1.0);
  set_goal(pick_up_pose, goal_pickup) ;
  goal_pub.publish(pick_up_pose);
  ROS_INFO("Sending pick up goal ...");
  send_goal(ac, goal_pickup);

  // pause for 5 sec
  ros::Duration(5.0).sleep();

  // Send the dropby goal position and orientation for the robot to reach
  move_base_msgs::MoveBaseGoal goal_drop;
  geometry_msgs::Pose drop_pose;
  set_pose(drop_pose, 3.0, -0.5, -1.0);
  set_goal(drop_pose, goal_drop) ;
  goal_pub.publish(drop_pose);
  ROS_INFO("Sending drop goal ...");
  if (send_goal(ac, goal_drop)) {
      // reach drop goal
      goal_pub.publish(drop_pose);
  }

  return 0;
}
