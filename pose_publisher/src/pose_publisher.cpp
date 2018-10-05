#include "pose_publisher.h"
PoseEstimator::PoseEstimator(ros::NodeHandle nh, image_transport::ImageTransport it):nh_(nh) { 
	image_sub_ = it.subscribe("/skyviper/camera", 10, &PoseEstimator::imageCallback, this);
	image_pub_ = it.advertise("/skyviper/camera/markers", 1);
	pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose", 100);
	path_pub_ = nh_.advertise<nav_msgs::Path>("/skyviper/path", 100);
	dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
	dx = 0.2159;
    dy = 0.2794;
    n_x = 10;

    nStates_ = 18;
    nMeasurements_ = 6;
    nInputs_ = 0;
    dt_ = 0.1;

    initKalmanFilter(KF_, nStates_, nMeasurements_, nInputs_, dt_);
	translation_measured_ = translation_estimated_ = 
	cv::Mat::zeros(cv::Size(3, 1), CV_64FC1);
    x_init_ = 4*dx;
    y_init_ = 2*dy;
    z_init_ = 0;

    count_ = 0;
    show_image_ = false;
    initializing_ = true;
    if(show_image_)
		cv::namedWindow("Display window");
	paramReader();
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
  		c_y = (image_.rows - 1)/2;
        c_x = (image_.cols - 1)/2;
		estimatePose();
	} else {
		ROS_WARN("Image not received");
	}
	arucoFactory();
}

void PoseEstimator::arucoFactory() {
	estimatePose();
	if(show_image_) {
		cv::imshow("Display window", image_);
		cv::waitKey(30);		
	}
	msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_).toImageMsg();
	msg_->header.stamp = time_stamp_;
	image_pub_.publish(msg_);
}

void PoseEstimator::estimatePose() {
	cv::aruco::detectMarkers(image_, dictionary_, corners_, ids_);
	if(ids_.size() > 0) {
	    cv::aruco::drawDetectedMarkers(image_, corners_, ids_);
	    cv::aruco::estimatePoseSingleMarkers(corners_, 0.141, cameraMatrix_, distCoeffs_, rvecs_, tvecs_);
		
	    std::tuple<float, int, std::vector<cv::Point2f>> minDistAndIdx = findMinDist(corners_);
	    float min_dist = std::get< 0 >(minDistAndIdx);
	    int min_dist_idx = std::get< 1 >(minDistAndIdx);
	    std::vector<cv::Point2f> centroids = std::get< 2 >(minDistAndIdx);
		drawOnImage(min_dist, centroids[min_dist_idx]);
		cv::Mat T_WC = getGlobalTransformation(rvecs_[min_dist_idx], 
			tvecs_[min_dist_idx], ids_[min_dist_idx]);
		std::tuple<cv::Point3d, tf::Quaternion> TQ = getTQ(T_WC);
		cv::Point3d translation = std::get< 0 >(TQ);
		tf::Quaternion quatn = std::get< 1 >(TQ);


        double px, py, pz;
        px = translation.x;
        py = translation.y;
        pz = translation.z;

    	translation_measured_.at<double>(0) = px;
    	translation_measured_.at<double>(1) = py;
		translation_measured_.at<double>(2) = pz; 
		cv::Mat measurements(6, 1, CV_64F);
		fillMeasurements(measurements, translation_measured_, quatn);
	    // Instantiate estimated translation and rotation
	    cv::Mat rotation_estimated(3, 3, CV_64F);
        updateKalmanFilter( KF_, 
        	                measurements,
                            translation_estimated_, 
                            quaternion_estimated_);
       	quaternion_estimated_.normalize(); 

        publishPose(translation_estimated_, quaternion_estimated_);

		initializing_ = false;
		count_ = 0;
	} else {
		if(!initializing_) {
			std::cout << count_ << std::endl;
			if(count_ <= 10) {
				updateKalmanFilter(KF_, translation_estimated_, quaternion_estimated_); 
				publishPose(translation_estimated_, quaternion_estimated_);
			}
			count_ ++;
		} else {
			ROS_INFO("Initializing");
		    translation_estimated_.at<double>(0) = x_init_;
		    translation_estimated_.at<double>(1) = y_init_;
		    translation_estimated_.at<double>(2) = z_init_;
		    tf::Quaternion quatn;
		    quaternion_estimated_[0] = 0.0; 
		    quaternion_estimated_[1] = 0.0; 
		    quaternion_estimated_[2] = 0.0; 
		    quaternion_estimated_[3] = 1.0;		
			publishPose(translation_estimated_, quaternion_estimated_);
		}
	}
}

void PoseEstimator::drawOnImage(float min_dist, cv::Point2f center) {
	char str[200];
	sprintf(str, "%f", min_dist);
	cv::putText(image_, str, center, cv::FONT_HERSHEY_SIMPLEX, 1, 
		cv::Scalar(0,0,255,255));	
	cv::circle(image_, center, 25, cv::Scalar(0, 0, 255), 2, 8, 0);
}

std::tuple<float, int, std::vector<cv::Point2f>> PoseEstimator::findMinDist(std::vector<std::vector<cv::Point2f> > corners) {
	std::vector<double> dists;
	std::vector<cv::Point2f> centroids;
	for (size_t i = 0; i < corners.size(); i++) {
		float sum_x = 0;
		float sum_y = 0;
		for (size_t j = 0; j < corners[i].size(); j++) {
			sum_x += corners[i][j].x;
			sum_y += corners[i][j].y;
		}
		cv::Point2f centroid(sum_x/4.0, sum_y/4.0);
		centroids.push_back(centroid);
		double dist = sqrt((centroid.x - c_x)*(centroid.x - c_x) 
			+ (centroid.y - c_y)*(centroid.y - c_y));
		dists.push_back(dist);
	}
	float min_dist = *std::min_element(dists.begin(), dists.end());
	int min_dist_idx = std::min_element(dists.begin(), dists.end()) - dists.begin();

	std::tuple<float, int, std::vector<cv::Point2f>> result(min_dist, min_dist_idx, centroids);

	return result;
}

cv::Mat PoseEstimator::getGlobalTransformation(cv::Vec3d rvec, cv::Vec3d tvec, int id) {
	cv::Mat R_CW, Rt_CW, T_CW, T_WC;
	cv::Rodrigues(rvec, R_CW, cv::noArray());
	cv::hconcat(R_CW, tvec, Rt_CW);
	double data[] = {0, 0, 0, 1}; 
	cv::Mat last_row = cv::Mat(1,4, CV_64F, data);
	cv::vconcat(Rt_CW, last_row, T_CW);

	int quotient = (id - 1)/ n_x;
	int remainder = (id - 1) % n_x;

    double data_Rpremul[4][4] = {{1, 0, 0, remainder*dx}, 
                                 {0, 1, 0, quotient*dy}, 
                                 {0, 0, 1, 0}, 
								 {0, 0, 0, 1}};
	cv::Mat Tpremul = cv::Mat(4, 4, CV_64F, data_Rpremul);
	double data_R[4][4] = {{0, -1, 0, 0}, {-1, 0, 0, 0}, {0, 0, -1, 0}, {0, 0, 0, 1}};
    cv::Mat transformation = cv::Mat(4, 4, CV_64F, data_R);
	T_WC = Tpremul*T_CW.inv()*transformation;
	return T_WC;
}

std::tuple<cv::Point3d, tf::Quaternion> PoseEstimator::getTQ(cv::Mat T) {
	cv::Point3d translation;
	translation.x = T.at<double>(0, 3);
	translation.y = T.at<double>(1, 3);
	translation.z = T.at<double>(2, 3);

	tf::Matrix3x3 Rotn;
    Rotn.setValue(T.at<double>(0,0), T.at<double>(0,1), T.at<double>(0,2),
    	          T.at<double>(1,0), T.at<double>(1,1), T.at<double>(1,2),
				  T.at<double>(2,0), T.at<double>(2,1), T.at<double>(2,2));
    tf::Quaternion quatn;
    Rotn.getRotation(quatn);
    quatn.normalize();

    std::tuple<cv::Point3d, tf::Quaternion> result(translation, quatn);
    return result;
}

void PoseEstimator::publishPose(cv::Mat translation_estimated, 
								tf::Quaternion quaternion_estimated) {
    tf::Vector3 globalTranslation_rh(translation_estimated.at<double>(0),
                                     translation_estimated.at<double>(1),
                                     translation_estimated.at<double>(2));
    time_stamp_ = ros::Time::now();
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(globalTranslation_rh);
    transform.setRotation(quaternion_estimated);
    br.sendTransform(tf::StampedTransform(transform, time_stamp_, "map", "base_link"));
    pose_6d_.header.frame_id = path_6d_.header.frame_id = "map";
    pose_6d_.header.stamp = path_6d_.header.stamp = time_stamp_;		   
    pose_6d_.pose.position.x = translation_estimated.at<double>(0);
    pose_6d_.pose.position.y = translation_estimated.at<double>(1);
    pose_6d_.pose.position.z = translation_estimated.at<double>(2);
    pose_6d_.pose.orientation.x = quaternion_estimated[0];
    pose_6d_.pose.orientation.y = quaternion_estimated[1];
    pose_6d_.pose.orientation.z = quaternion_estimated[2];
	pose_6d_.pose.orientation.w = quaternion_estimated[3]; 
	pose_pub_.publish(pose_6d_);
	path_6d_.poses.push_back(pose_6d_);
	path_pub_.publish(path_6d_);
}

void PoseEstimator::initKalmanFilter(cv::KalmanFilter &KF, int nStates, int nMeasurements, int nInputs, double dt) {
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

void PoseEstimator::fillMeasurements( cv::Mat &measurements,
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


void PoseEstimator::updateKalmanFilter(cv::KalmanFilter &KF, 
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

void PoseEstimator::updateKalmanFilter(cv::KalmanFilter &KF,
                                       cv::Mat &translation_estimated, 
                                       tf::Quaternion &quaternion_estimated ) {
    // First predict, to update the internal statePre variable
    cv::Mat prediction = KF.predict();

    // Estimated translation
    translation_estimated.at<double>(0) = prediction.at<double>(0);
    translation_estimated.at<double>(1) = prediction.at<double>(1);
    translation_estimated.at<double>(2) = prediction.at<double>(2);
    // Estimated euler angles
    double roll, pitch, yaw;
    roll = prediction.at<double>(9);
    pitch = prediction.at<double>(10);
    yaw = prediction.at<double>(11);
    // Convert euler angles to quaternions
    quaternion_estimated.setRPY(roll, pitch, yaw);
}

