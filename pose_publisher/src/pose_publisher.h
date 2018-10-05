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
#include <nav_msgs/Path.h>

#include <tuple>

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

	cv::Ptr<cv::aruco::Dictionary> dictionary_;
	std::vector<int> ids_;
	std::vector<std::vector<cv::Point2f> > corners_;
	std::vector< cv::Vec3d > rvecs_, tvecs_;
	double c_x, c_y; // image center

	double dx, dy;
    int n_x;

    
     
    geometry_msgs::PoseStamped pose_6d_;
    nav_msgs::Path path_6d_;
    sensor_msgs::ImagePtr msg_;

    bool show_image_;
    bool initializing_;

	// Kalman Filter
	cv::KalmanFilter KF_;
	int nStates_, nMeasurements_, nInputs_;
	double dt_;

	// initial position
	double x_init_, y_init_, z_init_;

	// pose variables
	cv::Mat translation_estimated_;
	cv::Mat translation_measured_;
	tf::Quaternion quaternion_estimated_;

	// timing amd counters for tests and debugging
	ros::Time begin_;
	int count_;
public:
	PoseEstimator(ros::NodeHandle, image_transport::ImageTransport);
	void paramReader();
	void imageCallback(const sensor_msgs::ImageConstPtr& msg);
	void estimatePose();
	void arucoFactory();
	void drawOnImage(float, cv::Point2f);
	std::tuple<float, int, std::vector<cv::Point2f>> findMinDist(std::vector<std::vector<cv::Point2f>>);
	cv::Mat getGlobalTransformation(cv::Vec3d, cv::Vec3d, int);
	std::tuple<cv::Point3d, tf::Quaternion> getTQ(cv::Mat);
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

