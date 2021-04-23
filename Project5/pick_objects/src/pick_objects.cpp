#include "ros/ros.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <add_markers/ModifyMarker.h>

// Define a client for to send goal requests to the move_base server through a
// SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
  MoveBaseClient;

class PickObjects {
public:

  PickObjects(float pickup_x,
              float pickup_y,
              float dropoff_x,
              float dropoff_y,
              bool  use_markers)
  {
    // Define the pickup location
    pickup_ = get_goal((float)pickup_x,
                       (float)pickup_y);

    // Define the dropoff location
    dropoff_ = get_goal((float)dropoff_x,
                        (float)dropoff_y);

    // Get the object that can command the robot to move to certain locations
    // tell the action client that we want to spin a thread by default
    ac_ = new MoveBaseClient("move_base", true);

    // Wait 5 sec for move_base action server to come up
    while (!ac_->waitForServer(ros::Duration(5.0))) {
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    // Define a client service capable of requesting services from add_markers
    ROS_INFO("Will publish markers on /add_markers/modify_marker");
    marker_client_ = n_.serviceClient<add_markers::ModifyMarker>(
      "/add_markers/modify_marker");

    use_markers_ = use_markers;

    if (use_markers_) {
      // Wait for the marker service to be up and running
      while (!marker_client_.exists()) {
        if (!ros::ok())
        {
          return;
        }
        ROS_WARN_ONCE(
          "Waiting on /add_markers/modify_marker service to be available...");
        sleep(1);
      }
      ROS_INFO("/add_markers/modify_marker is available!");
    }

    ROS_INFO("Starting the pickup/dropoff sequence");
    perform_sequence();
    ROS_INFO("Sequence complete");
  }

  /**
   * Performs the following sequence:
   * Show the pickup marker
   * Move to the pickup location
   * Pickup the object
   * Hide the pickup marker
   * Move to the dropoff location
   * Show the dropoff marker
   */
  bool perform_sequence() {
    // Show the box at the pickup location
    ROS_INFO("Box is at the pickup location");
    action_marker_in_rviz(pickup_, PICKUP_ID,
                          ADD_MARKER);

    // Move to the pickup location
    ROS_INFO("Moving to pickup location....");

    if (!move_to_goal(pickup_)) {
      return 1;
    }

    ROS_INFO("Reached the pickup location! Picking up the box.");

    // Picking up the box takes some time...
    ros::Duration(5).sleep();

    // Hide the box from pickup location
    action_marker_in_rviz(pickup_, PICKUP_ID,
                          DELETE_MARKER);

    // Move to dropoff
    ROS_INFO("Moving to dropoff location with the box...");

    if (!move_to_goal(dropoff_)) {
      return 1;
    }

    ROS_INFO("Reached the dropoff location! Dropping off the object.");

    // Show the box at the dropoff location
    action_marker_in_rviz(dropoff_, DROPOFF_ID,
                          ADD_MARKER);
    return 0;
  }

private:

  // Some unique ids for the visualization of each box
  uint32_t PICKUP_ID  = 0;
  uint32_t DROPOFF_ID = 1;

  uint32_t ADD_MARKER    = 0; // visualization_msgs::Marker::ADD
  uint32_t DELETE_MARKER = 2; // visualization_msgs::Marker::DELETE

  // If true, will use RVIZ markers
  bool use_markers_;

  MoveBaseClient *ac_;
  ros::NodeHandle n_;
  ros::ServiceClient marker_client_;
  move_base_msgs::MoveBaseGoal pickup_;
  move_base_msgs::MoveBaseGoal dropoff_;

  /**
   *   Show or hide a marker in rviz
   */
  void action_marker_in_rviz(move_base_msgs::MoveBaseGoal goal,
                             uint32_t                     id,
                             uint32_t                     action) {
    if (!use_markers_) {
      return;
    }

    float x = goal.target_pose.pose.position.x;
    float y = goal.target_pose.pose.position.y;
    float z = goal.target_pose.pose.position.z;

    ROS_INFO("Creating a marker to x=%1.2f, y=%1.2f, id=%d, action=%d",
             x,
             y,
             id,
             action);


    add_markers::ModifyMarker srv;
    srv.request.position.x = x;
    srv.request.position.y = y;
    srv.request.position.z = z;
    srv.request.scale      = 0.3;
    srv.request.id         = id;
    srv.request.action     = action;

    // Call the add_markers service and pass the requested marker action
    if (!marker_client_.call(srv)) ROS_ERROR(
        "Failed to call service add_markers");
  }

  /**
   *   Defines a goal location
   */
  move_base_msgs::MoveBaseGoal get_goal(float x, float y) {
    move_base_msgs::MoveBaseGoal goal;

    // set up the frame parameters
    goal.target_pose.header.frame_id = "map"; // Use the map as a frame
                                              // reference
    goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x    = x;
    goal.target_pose.pose.position.y    = y;
    goal.target_pose.pose.orientation.w = 1.0;
    return goal;
  }

  /**
   *   Moves the robot to the specified goal
   */
  bool move_to_goal(move_base_msgs::MoveBaseGoal goal) {
    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending goal to x=%1.2f, y=%1.2f",
             goal.target_pose.pose.position.x,
             goal.target_pose.pose.position.y
             );

    ac_->sendGoal(goal);

    // Wait an infinite time for the results
    ac_->waitForResult();

    // Check if the robot reached its goal
    if (ac_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, the base reached %1.2f, %1.2f",
               goal.target_pose.pose.position.x,
               goal.target_pose.pose.position.y);
      return true;
    }
    else {
      ROS_ERROR(
        "The base failed to move to %1.2f, %1.2f for some reason",
        goal.target_pose.pose.position.x,
        goal.target_pose.pose.position.y
        );
      return false;
    }
  }
}; // End of class PickObjects

int main(int argc, char **argv)
{
  // Initiate ROS
  ros::init(argc, argv, "pick_objects");

  ros::NodeHandle nh("~");

  // Parse args
  double pickup_x;
  double pickup_y;

  nh.getParam("pickup_x", pickup_x);
  nh.getParam("pickup_y", pickup_y);

  double dropoff_x;
  double dropoff_y;
  nh.getParam("dropoff_x",   dropoff_x);
  nh.getParam("dropoff_y",   dropoff_y);

  bool use_markers;
  nh.getParam("use_markers", use_markers);

  // Create an object of class PickObjects that will take care of everything
  PickObjects PickObjectsObject(pickup_x,
                                pickup_y,
                                dropoff_x,
                                dropoff_y,
                                use_markers);

  ros::spin();

  return 0;
}
