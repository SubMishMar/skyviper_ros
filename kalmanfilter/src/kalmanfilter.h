#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <tf/tf.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// OpenCV libraries
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/tracking.hpp>
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

class kalmanFilter {
private:
	ros::NodeHandle nh_;
	ros::Subscriber pose_in_sub_;
	ros::Publisher pose_out_pub_;
    geometry_msgs::PoseStamped pose_6d_;

	// Kalman Filter
	bool use_KF_;
	cv::KalmanFilter KF_;
	int nStates_, nMeasurements_, nInputs_;
	double dt_;

	// pose variables
	cv::Mat translation_measured_;
	tf::Quaternion quaternion_measured_;
	cv::Mat translation_estimated_;
	tf::Quaternion quaternion_estimated_;

public:
	//Constructor
	kalmanFilter(ros::NodeHandle);

	// Pose callback and publishers
	void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
	void publishPose(cv::Mat, tf::Quaternion);
	
	//Kalman Filter
	void initKalmanFilter(cv::KalmanFilter &KF, int nStates, 
						  int nMeasurements, int nInputs, double dt);
	void fillMeasurements( cv::Mat &measurements, 
						   const cv::Mat &translation_measured, 
	                       const tf::Quaternion &quat_measured);
	void updateKalmanFilter( cv::KalmanFilter &KF, cv::Mat &measurement,
	                         cv::Mat &translation_estimated, 
	                         tf::Quaternion &quaternion_estimated );
	void updateKalmanFilter( cv::KalmanFilter &KF,
	                         cv::Mat &translation_estimated, 
							 tf::Quaternion &quaternion_estimated );
};