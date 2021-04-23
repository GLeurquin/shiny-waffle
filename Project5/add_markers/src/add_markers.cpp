#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include "add_markers/ModifyMarker.h"

class AddMarkersService {
public:

  AddMarkersService()
  {
    // Advertise a service to change markers in rviz
    ROS_INFO("Offering modify marker service on /add_markers/modify_marker");
    service_ = n_.advertiseService("/add_markers/modify_marker",
                                   &AddMarkersService::handle_modify_marker_callback,
                                   this);

    // We are going to publish messages to the visualization_marker service of
    // rviz
    marker_pub_ = n_.advertise<visualization_msgs::Marker>(
      "visualization_marker",
      1);

    // Wait for marker to be ready
    while (marker_pub_.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    ROS_INFO("Ready to display markers");
  }

private:

  ros::NodeHandle n_;
  ros::ServiceServer service_;
  ros::Publisher marker_pub_;

  /**
   * Creates a ros marker
   */
  visualization_msgs::Marker get_marker(float         x,
                                        float         y,
                                        float         z,
                                        float         scale,
                                        ros::Duration duration,
                                        uint32_t      id,
                                        uint32_t      action
                                        ) {
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on
    // these.
    marker.header.frame_id = "/map"; // Use the map as a frame reference
    marker.header.stamp    = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique
    // ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = id;

    // Set the marker type. This is a CUBE
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3
    // (DELETEALL)
    marker.action = action;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the
    // frame/time specified in the header
    marker.pose.position.x    = x;
    marker.pose.position.y    = y;
    marker.pose.position.z    = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = duration;

    return marker;
  }

  // This callback function executes whenever a add_markers service is requested
  bool handle_modify_marker_callback(add_markers::ModifyMarker::Request & req,
                                     add_markers::ModifyMarker::Response& res)
  {
    ROS_INFO("ModifyMarkerRequest received - j1:%1.2f, j2:%1.2f",
             (float)req.position.x,
             (float)req.position.y);

    // Create the marker object
    visualization_msgs::Marker the_marker = get_marker(
      req.position.x,
      req.position.y,
      req.position.z,
      req.scale,
      ros::Duration(),
      req.id,
      req.action
      );

    // Publish it to rviz
    marker_pub_.publish(the_marker);

    // Return a response message
    res.msg_feedback = "Marker " +
                       std::to_string(req.id) + " published.";
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
  }
}; // End of class AddMarkersService

int main(int argc, char **argv)
{
  // Initiate ROS
  ros::init(argc, argv, "add_markers");

  // Create an object of class AddMarkersService that will take care of
  // everything
  AddMarkersService AddMarkersService;

  ros::spin();

  return 0;
}
