#include <ros/ros.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
double poseAMCLx, poseAMCLy, poseAMCLz;
struct DestinationCoords {
  double positionX;
  double positionY;
  double positionZ;
  double orientationX;
  double orientationY;
  double orientationW;
  double orientationZ;
};

void PoseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL);
void InitPosition(MoveBaseClient& ac, move_base_msgs::MoveBaseGoal& goal);
void MoveToDestination(MoveBaseClient& ac, move_base_msgs::MoveBaseGoal& goal, DestinationCoords destinationCoords);
void PrintMenu(std::string& dest);

int main(int argc, char **argv) {
  // AMCL_POSE topic
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("amcl_pose", 1000, PoseAMCLCallback);

  // Navigation action
  ros::init(argc, argv, "simple_navigation_goals");
  MoveBaseClient ac("move_base", true); // true -> don't need ros::spin()
  while(!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";

  // Initializing Position(Location) of TurtleBot
  // InitPosition(ac, goal);

  // If 0 is entered, terminate the program
  while (ros::ok()) {
    std::string dest;
    PrintMenu(dest);

    DestinationCoords destinationCoords;
    switch (atoi(dest.c_str())) {
      case 211:
        destinationCoords = { 10.3, 4.7, 0.0, 0.0, 0.0, -0.16, 1.0 };
        MoveToDestination(ac, goal, destinationCoords);
        std::cout << "The turtlebot has arrived in classroom 211" << std::endl;
        break;
      case 212:
        destinationCoords = { 6.88031060825, -3.10083584264, 0.0, 0.0, 0.0, -0.14043795689, 1.0 };
        MoveToDestination(ac, goal, destinationCoords);
        std::cout << "The turtlebot has arrived in classroom 212" << std::endl;
        break;
      case 213:
        destinationCoords = { 5.23840945737, -5.68014422117, 0.0, 0.0, 0.0, -0.16956420134, 0.985519143205 };
        MoveToDestination(ac, goal, destinationCoords);
        std::cout << "The turtlebot has arrived in classroom 213" << std::endl;
        break;
      case 0:
        std::cout << "Bye" << std::endl;
        return 0;
      default:
        std::cout << "The command does not exist. Enter a correct command." << std::endl;
        continue;
    }
  }

  return 0;
}

void PoseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL) {
  poseAMCLx = msgAMCL->pose.pose.position.x;
  poseAMCLy = msgAMCL->pose.pose.position.y;
  poseAMCLz = msgAMCL->pose.pose.position.z;
}

void PrintMenu(std::string& dest) {
  std::cout << "\nWhere do you want to go? (Enter '0' if you want to quit)" << std::endl;
  std::cout << " == List ==" << std::endl;
  std::cout << " [" << std::endl;
  std::cout << "     211" << std::endl;
  std::cout << "     212" << std::endl;
  std::cout << "     213" << std::endl;
  std::cout << "          ]" << std::endl;
  std::cin >> dest;
}

void InitPosition(MoveBaseClient& ac, move_base_msgs::MoveBaseGoal& goal) {
  ros::spinOnce();

  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = poseAMCLx;
  goal.target_pose.pose.position.y = poseAMCLy + 0.5;
  goal.target_pose.pose.position.z = poseAMCLz;

  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.w = -0.15;
  goal.target_pose.pose.orientation.z = 1.0;
  ac.sendGoal(goal);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Initializing is completed");
  else
    ROS_INFO("Failed to initialize");
}

void MoveToDestination(MoveBaseClient& ac, move_base_msgs::MoveBaseGoal& goal, DestinationCoords destinationCoords) {
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = destinationCoords.positionX;
  goal.target_pose.pose.position.y = destinationCoords.positionY;
  goal.target_pose.pose.position.z = destinationCoords.positionZ;

  goal.target_pose.pose.orientation.x = destinationCoords.orientationX;
  goal.target_pose.pose.orientation.y = destinationCoords.orientationY;
  goal.target_pose.pose.orientation.w = destinationCoords.orientationW;
  goal.target_pose.pose.orientation.z = destinationCoords.orientationZ;
  ac.sendGoal(goal);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Reached the goal");
  else
    ROS_INFO("Failed to reach the goal");
}
