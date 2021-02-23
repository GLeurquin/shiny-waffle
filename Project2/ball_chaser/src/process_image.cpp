#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

class ProcessImage {
public:

  ProcessImage()
  {
    // Define a client service capable of requesting services from safe_move
    client_ = n_.serviceClient<ball_chaser::DriveToTarget>(
      "/ball_chaser/command_robot");

    // Subscribe to rgb_camera/image_raw topic to read the image data inside the
    // process_image_callback function
    sub1_ = n_.subscribe("/camera/rgb/image_raw",
                         10,
                         &ProcessImage::process_image_callback, this);

    ROS_INFO("Ready to follow the ball");
  }

private:

  ros::NodeHandle n_;
  ros::Subscriber sub1_;
  ros::ServiceClient client_;

  // This function calls the command_robot service to drive the robot in the
  // specified direction
  void drive_robot(float lin_x, float ang_z)
  {
    ROS_INFO_STREAM("Moving the robot to " + std::to_string(
                      lin_x) + ", angz: " + std::to_string(ang_z));

    // Request centered joint angles [1.57, 1.57]
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x  = lin_x;
    srv.request.angular_z = ang_z;

    // Call the command_robot service and pass the requested joint angles
    if (!client_.call(srv)) ROS_ERROR("Failed to call service command_robot");
  }

  // This callback function continuously executes and reads the image data
  void process_image_callback(const sensor_msgs::Image img)
  {
    int   white_pixel = 255;
    float lin_x = 0, ang_z = 0;

    // Loop through each pixel in the image and check if its equal to the first
    // one
    for (uint32_t i = 0; i < img.height * img.step; i += 3) {
      if ((img.data[i] == white_pixel) && (img.data[i + 1] == white_pixel) &&
          (img.data[i + 2] == white_pixel)) {
        uint32_t col = i % img.step;

        if (col < img.step * 0.25) {
          // Turn left
          ang_z = 0.5;
        } else if (col < img.step * 0.75) {
          // Go forward
          lin_x = 0.5;
        } else {
          // Turn Right
          ang_z = -0.5;
        }
        break;
      }
    }
    drive_robot(lin_x, ang_z);
  }
}; // End of class ProcessImage

int main(int argc, char **argv)
{
  // Initiate ROS
  ros::init(argc, argv, "process_image");

  // Create an object of class ProcessImage that will take care of everything
  ProcessImage ProcessImageObject;

  ros::spin();

  return 0;
}
