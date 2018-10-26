#include "kalmanfilter.h"

kalmanFilter::kalmanFilter(ros::NodeHandle nh) {
	nh_ = nh;
	pose_in_sub_ = nh_.subscribe("/global_pose", 10, &kalmanFilter::poseCallback, this);
	pose_out_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
    nStates_ = 18;
    nMeasurements_ = 6;
    nInputs_ = 0;
    dt_ = 0.1;
	initKalmanFilter(KF_, nStates_, nMeasurements_, nInputs_, dt_);
}

void kalmanFilter::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  translation_measured_ = translation_estimated_ = cv::Mat::zeros(cv::Size(3, 1), CV_64FC1);
  translation_measured_.at<double>(0) = msg->pose.position.x;
  translation_measured_.at<double>(1) = msg->pose.position.y;
  translation_measured_.at<double>(2) = msg->pose.position.z;
  cv::Mat measurements(6, 1, CV_64F);

  quaternion_measured_ = tf::Quaternion(msg->pose.orientation.x, 
                                        msg->pose.orientation.y, 
                                        msg->pose.orientation.z, 
                                        msg->pose.orientation.w);
  
  fillMeasurements(measurements, translation_measured_, quaternion_measured_);
  updateKalmanFilter(KF_, 
	                   measurements,
                     translation_estimated_, 
                     quaternion_estimated_);
  publishPose(translation_estimated_, quaternion_estimated_);
}

void kalmanFilter::initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt) {
  KF.init(nStates, nMeasurements, nInputs, CV_64F);                 // init Kalman Filter
  cv::setIdentity(KF.processNoiseCov, cv::Scalar::all(1e-5));       // set process noise
  cv::setIdentity(KF.measurementNoiseCov, cv::Scalar::all(1e-2));   // set measurement noise
  cv::setIdentity(KF.errorCovPost, cv::Scalar::all(1));             // error covariance
                 /* DYNAMIC MODEL */
  //  [1 0 0 dt  0  0 dt2   0   0 0 0 0  0  0  0   0   0   0]
  //  [0 1 0  0 dt  0   0 dt2   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 1  0  0 dt   0   0 dt2 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  1  0  0  dt   0   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  1  0   0  dt   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  1   0   0  dt 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   1   0   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   0   1   0 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   0   0   1 0 0 0  0  0  0   0   0   0]
  //  [0 0 0  0  0  0   0   0   0 1 0 0 dt  0  0 dt2   0   0]
  //  [0 0 0  0  0  0   0   0   0 0 1 0  0 dt  0   0 dt2   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 1  0  0 dt   0   0 dt2]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  1  0  0  dt   0   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  1  0   0  dt   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  1   0   0  dt]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   1   0   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   1   0]
  //  [0 0 0  0  0  0   0   0   0 0 0 0  0  0  0   0   0   1]

  // position
  KF.transitionMatrix.at<double>(0,3) = dt;
  KF.transitionMatrix.at<double>(1,4) = dt;
  KF.transitionMatrix.at<double>(2,5) = dt;
  KF.transitionMatrix.at<double>(3,6) = dt;
  KF.transitionMatrix.at<double>(4,7) = dt;
  KF.transitionMatrix.at<double>(5,8) = dt;
  KF.transitionMatrix.at<double>(0,6) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(1,7) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(2,8) = 0.5*pow(dt,2);

  // orientation
  KF.transitionMatrix.at<double>(9,12) = dt;
  KF.transitionMatrix.at<double>(10,13) = dt;
  KF.transitionMatrix.at<double>(11,14) = dt;
  KF.transitionMatrix.at<double>(12,15) = dt;
  KF.transitionMatrix.at<double>(13,16) = dt;
  KF.transitionMatrix.at<double>(14,17) = dt;
  KF.transitionMatrix.at<double>(9,15) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(10,16) = 0.5*pow(dt,2);
  KF.transitionMatrix.at<double>(11,17) = 0.5*pow(dt,2);

       /* MEASUREMENT MODEL */
  //  [1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0]

  KF.measurementMatrix.at<double>(0,0) = 1;  // x
  KF.measurementMatrix.at<double>(1,1) = 1;  // y
  KF.measurementMatrix.at<double>(2,2) = 1;  // z
  KF.measurementMatrix.at<double>(3,9) = 1;  // roll
  KF.measurementMatrix.at<double>(4,10) = 1; // pitch
  KF.measurementMatrix.at<double>(5,11) = 1; // yaw
}

void kalmanFilter::fillMeasurements( cv::Mat &measurements,
                   const cv::Mat &translation_measured, const tf::Quaternion &quat_measured) {
    double roll, pitch, yaw;
    tf::Matrix3x3(quat_measured).getRPY(roll, pitch, yaw);
    // Set measurement to predict
    measurements.at<double>(0) = translation_measured.at<double>(0); // x
    measurements.at<double>(1) = translation_measured.at<double>(1); // y
    measurements.at<double>(2) = translation_measured.at<double>(2); // z
    measurements.at<double>(3) = roll;      // roll
    measurements.at<double>(4) = pitch;      // pitch
    measurements.at<double>(5) = yaw;      // yaw
}


void kalmanFilter::updateKalmanFilter(cv::KalmanFilter &KF, 
	                                    cv::Mat &measurement,
                                      cv::Mat &translation_estimated, 
                                      tf::Quaternion &quaternion_estimated ) {
    // First predict, to update the internal statePre variable
    cv::Mat prediction = KF.predict();
    // The "correct" phase that is going to use the predicted value and our measurement
    cv::Mat estimated = KF.correct(measurement);
    // Estimated translation
    translation_estimated.at<double>(0) = estimated.at<double>(0);
    translation_estimated.at<double>(1) = estimated.at<double>(1);
    translation_estimated.at<double>(2) = estimated.at<double>(2);
    // Estimated euler angles
    double roll, pitch, yaw;
    roll = estimated.at<double>(9);
    pitch = estimated.at<double>(10);
    yaw = estimated.at<double>(11);
    // Convert euler angles to quaternions
    quaternion_estimated.setRPY(roll, pitch, yaw);
}

void kalmanFilter::publishPose(cv::Mat translation_estimated, 
							                 tf::Quaternion quaternion_estimated) {
    tf::Vector3 globalTranslation_rh(translation_estimated.at<double>(0),
                                     translation_estimated.at<double>(1),
                                     translation_estimated.at<double>(2));
    quaternion_estimated.normalize();
    pose_6d_.header.frame_id = "map";
    pose_6d_.header.stamp = ros::Time::now();		   
    pose_6d_.pose.position.x = translation_estimated.at<double>(0);
    pose_6d_.pose.position.y = translation_estimated.at<double>(1);
    pose_6d_.pose.position.z = translation_estimated.at<double>(2);
    pose_6d_.pose.orientation.x = quaternion_estimated[0];
    pose_6d_.pose.orientation.y = quaternion_estimated[1];
    pose_6d_.pose.orientation.z = quaternion_estimated[2];
  	pose_6d_.pose.orientation.w = quaternion_estimated[3]; 
  	pose_out_pub_.publish(pose_6d_);
}