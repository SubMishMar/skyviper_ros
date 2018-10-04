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
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>


class PoseEstimator {
private:
	ros::NodeHandle nh_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	ros::Publisher pose_pub_;
	ros::Publisher path_pub_;

	cv_bridge::CvImagePtr cv_ptr_;
	ros::Time time_stamp_;

	cv::Mat cameraMatrix_, distCoeffs_;
	cv::Mat image_;
public:
	PoseEstimator(ros::NodeHandle, image_transport::ImageTransport);
	void paramReader();
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	void estimatePose();
};

void PoseEstimator::estimatePose() {
	
}

void PoseEstimator::paramReader() {
	std::string config_file;
	if(!nh_.getParam("config_file", config_file)) {
		ROS_ERROR("Config File Address Not Loaded");
	} 
	cv::FileStorage fs(config_file, cv::FileStorage::READ);
	if(!fs.isOpened()) {
		ROS_ERROR("Failed to open config file");
	}
    fs["intrinsics"] >> cameraMatrix_;
    fs["distortion_coefficients"] >> distCoeffs_;
}

void PoseEstimator::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  	try {
    	cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    	time_stamp_ = msg->header.stamp;
    	cv::waitKey(30);
    } catch (cv_bridge::Exception& e) {
    	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}

	if(cv_ptr_) {
		image_ = cv_ptr_->image;
		estimatePose();
	} else {
		ROS_WARN("Image not received");
	}
}

PoseEstimator::PoseEstimator(ros::NodeHandle nh, image_transport::ImageTransport it):nh_(nh) { 
	image_sub_ = it.subscribe("/skyviper/camera", 1, &PoseEstimator::imageCallback, this);
	image_pub_ = it.advertise("/skyviper/camera/markers", 1);
	pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose", 100);
	path_pub_ = nh_.advertise<nav_msgs::Path>("/skyviper/path", 100);
	paramReader();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  PoseEstimator obj(nh, it);
  ros::spin();
  return 0;
}


