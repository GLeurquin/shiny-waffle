#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

class DriveBotService {
public:

  DriveBotService()
  {
    service_ = n_.advertiseService("/ball_chaser/command_robot",
                                   &DriveBotService::handle_drive_request_callback,
                                   this);
    motor_command_publisher_ = n_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ROS_INFO("Ready to send motor commands");
  }

private:

  ros::NodeHandle n_;
  ros::Publisher motor_command_publisher_;
  ros::ServiceServer service_;

  // This callback function executes whenever a safe_move service is requested
  bool handle_drive_request_callback(ball_chaser::DriveToTarget::Request & req,
                                     ball_chaser::DriveToTarget::Response& res)
  {
    ROS_INFO("DriveToTargetRequest received - j1:%1.2f, j2:%1.2f",
             (float)req.linear_x,
             (float)req.angular_z);

    // Create a motor_command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;

    // Set wheel velocities
    motor_command.linear.x  = req.linear_x;
    motor_command.angular.z = req.angular_z;

    // Publish angles to drive the robot
    motor_command_publisher_.publish(motor_command);

    // Return a response message
    res.msg_feedback = "Linear_x set: " +
                       std::to_string(motor_command.linear.x) +
                       ", angular_z set: " +
                       std::to_string(motor_command.angular.z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
  }
}; // End of class DriveBotService

int main(int argc, char **argv)
{
  // Initiate ROS
  ros::init(argc, argv, "drive_bot");

  // Create an object of class DriveBotService that will take care of
  // everything
  DriveBotService DriveBotService;

  ros::spin();

  return 0;
}
