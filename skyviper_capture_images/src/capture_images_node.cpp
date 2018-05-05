#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <tf/tf.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include "opencv2/core/core.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
using namespace cv;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_(nh_);
  image_transport::Publisher image_pub_ = it_.advertise("/skyviper/camera", 1);
 
  
  VideoCapture cap("http://192.168.99.1/ajax/video.mjpg");

  if(!cap.isOpened())  // check succeeded
  {
     std::cout << "Cannot Open Video Device" << std::endl;
     return -1;
  }
  ros::Rate loop_rate(30);
  sensor_msgs::ImagePtr msg;
  cv::Mat image;
  while(ros::ok())
  {
   cap >> image; // get a new image from camera
   msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
   msg->header.stamp = ros::Time::now();
   image_pub_.publish(msg);
   ros::spinOnce();
   loop_rate.sleep();
  }
  // the camera will be deinitialized automatically in VideoCapture destructor
  return 0;
}


