#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/highgui.hpp>

using namespace cv;

class skyviperCameraDriver {
private:
  image_transport::Publisher image_pub_;
  VideoCapture capture_;
  sensor_msgs::ImagePtr msg_;
  Mat image_;
public:
  skyviperCameraDriver(image_transport::ImageTransport, VideoCapture);
  void imageReader();
  void imagePublisher();
};

skyviperCameraDriver::skyviperCameraDriver(image_transport::ImageTransport it_,
                                           VideoCapture cap_) {
  capture_ = cap_;
  image_pub_ = it_.advertise("/skyviper/camera", 1);
}

void skyviperCameraDriver::imageReader() {
  capture_ >> image_; 
  msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_).toImageMsg();
  msg_->header.stamp = ros::Time::now();
}

void skyviperCameraDriver::imagePublisher() {
  image_pub_.publish(msg_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh); 

  VideoCapture cap("http://192.168.99.1/ajax/video.mjpg");
  if(!cap.isOpened()) {
     std::cout << "Cannot Open Video Device" << std::endl;
     return -1;
  }

  skyviperCameraDriver obj(it, cap);
  ros::Rate loop_rate(30);

  while (ros::ok()) {
    obj.imageReader();
    obj.imagePublisher();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


