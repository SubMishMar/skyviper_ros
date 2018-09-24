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



using namespace std;



using namespace cv;
cv_bridge::CvImagePtr cv_ptr;
ros::Time time_stamp;
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    time_stamp = msg->header.stamp;
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_(nh_);
  image_transport::Subscriber image_sub_ = it_.subscribe("/skyviper/camera", 1, imageCallback);
  image_transport::Publisher image_pub_ = it_.advertise("/skyviper/camera/markers", 1);
  //ros::Publisher pub = nh_.advertise<geometry_msgs::PoseStamped>("/skyviper/pose",1000);
  //ros::Publisher pub_avg_pose = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",1000);
  ros::Publisher pub_avg_pose = nh_.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose",1000);
  ros::Publisher path_publisher = nh_.advertise<nav_msgs::Path>( "/skyviper/path", 1000);


  std::string config_file;
  if(!nh_.getParam("config_file", config_file))
  {
 	 ROS_ERROR("Config File Address Not Loaded");
  }
  cv::FileStorage fs(config_file, cv::FileStorage::READ);
  if (!fs.isOpened())
  {
   std::cerr << "Failed to open config file"<< std::endl;
   return 1;
  }
 
  cv::Mat cameraMatrix, distCoeffs;
  cv::Mat r_cw, T_cw, T_wc_transformed, rt_cw;
  cv::Mat image;
  fs["intrinsics"] >> cameraMatrix;
  fs["distortion_coefficients"] >> distCoeffs;

  ros::Rate loop_rate(30);
  sensor_msgs::ImagePtr msg;

  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

  std::vector< cv::Vec3d > rvecs, tvecs;
  std::vector<int> ids;
  std::vector<double> dists;
  std::vector<std::vector<cv::Point2f> > corners;


  //Clock Variables
  clock_t begin, end;
  double dt;
    
  double data[] = {0, 0, 0, 1}; 
  double data_R[4][4] = {{0, -1, 0, 0}, {-1, 0, 0, 0}, {0, 0, -1, 0}, {0, 0, 0, 1}};
  cv::Mat last_row = cv::Mat(1,4, CV_64F, data);
  cv::Mat transformation = cv::Mat(4, 4, CV_64F, data_R);

  double c_y, c_x;
  double mean_x, mean_y, d_fm_ctr;
  
  tf::Matrix3x3 Rotn;
  tf::Quaternion Quatn(0, 0, 0, 1);
  geometry_msgs::PoseStamped pose_6d;

  nav_msgs::Path path_6d;
  
  
  double n_rows = 10;
  double n_cols = 10;
  double dx = 8.5*0.0254;
  double dy = 11*0.0254;
  int row, col;
  double sum_x, sum_y, sum_z, alpha_x, alpha_y, alpha_z, count;
  sum_x = sum_y = sum_z = 0;
  alpha_x = alpha_y = alpha_z = 0.8;
  count = 0;
  bool initializing = true;
  while(ros::ok())
  {
   if(cv_ptr)
   {
   c_y = (cv_ptr->image.rows - 1)/2;
   c_x = (cv_ptr->image.cols - 1)/2;
   cv::aruco::detectMarkers(cv_ptr->image, dictionary, corners, ids);

   // if at least one marker detected
   if (ids.size() > 0)
   {
        ROS_INFO_STREAM("Seeing at least one marker");
        cv::aruco::drawDetectedMarkers(cv_ptr->image, corners, ids);
        cv::aruco::estimatePoseSingleMarkers(corners, 0.141, cameraMatrix, distCoeffs, rvecs, tvecs);
        for(int i=0; i<ids.size(); i++)
        {
         
         mean_x = mean_y = 0;
         for(int j =0; j < 4; j++)
         {
          mean_x = mean_x + corners[i][j].x;
          mean_y = mean_y + corners[i][j].y;
         }
         mean_x = mean_x/4;
         mean_y = mean_y/4;

         
         d_fm_ctr = sqrt((mean_x - c_x)*(mean_x - c_x) + (mean_y - c_y)*(mean_y - c_y));
         //std::cout << d_fm_ctr_2 << std::endl;
         dists.push_back(d_fm_ctr * norm(tvecs[i])); // dist metric = distance from center x dist from camera.
        }

        int hi, lo;    
        hi = lo = 0;   
     
        for (int i = 0;i < dists.size();i++) 
        {
         if(dists[i] < dists[lo]) 
         {
          lo = i;
         }
         else if(dists[i] > dists[hi]) 
         {
          hi = i;
         }
        }
        dists.clear();
        //std::cout << corners[lo] << std::endl;
        cv::aruco::drawAxis(cv_ptr->image, cameraMatrix, distCoeffs, rvecs[lo], tvecs[lo], 0.1);

        cv::Rodrigues(rvecs[lo], r_cw, cv::noArray());
        cv::hconcat(r_cw, tvecs[lo], rt_cw);
        cv::vconcat(rt_cw, last_row, T_cw);
        row = ceil(ids[lo]/n_cols);
        col = ids[lo] - (row - 1)*n_cols;
        row = row - 1;
        col = col - 1;
        /*
        double data_Rpremul[4][4] = {{1, 0, 0, col*dx + 0.037 + 0.0705}, 
                        {0, 1, 0, row*dy + 0.074 + 0.0705}, 
                        {0, 0, 1, 0}, 
                        {0, 0, 0, 1}};
        */

        // data_Rpremul used to convert from ENU to NED
        double data_Rpremul[4][4] = {{1, 0,  0, 0}, 
                                     {0, 1,  0, 0}, 
                                     {0, 0,  1, 0}, 
                                     {0, 0,  0, 1}};
        cv::Mat Tpremul = cv::Mat(4, 4, CV_64F, data_Rpremul);
        T_wc_transformed = Tpremul*T_cw.inv()*transformation;

        Rotn.setValue(T_wc_transformed.at<double>(0,0), T_wc_transformed.at<double>(0,1), T_wc_transformed.at<double>(0,2),
        	          T_wc_transformed.at<double>(1,0), T_wc_transformed.at<double>(1,1), T_wc_transformed.at<double>(1,2),
        	          T_wc_transformed.at<double>(2,0), T_wc_transformed.at<double>(2,1), T_wc_transformed.at<double>(2,2));
        
        if(count == 0)
        {
         sum_x = T_wc_transformed.at<double>(0,3);
         sum_y = T_wc_transformed.at<double>(1,3);
         sum_z = T_wc_transformed.at<double>(2,3);
         count = count + 1;
        }
        else
        {
         sum_x = alpha_x*T_wc_transformed.at<double>(0,3) + (1-alpha_x)*sum_x;
         sum_y = alpha_y*T_wc_transformed.at<double>(1,3) + (1-alpha_y)*sum_y;
         sum_z = alpha_z*T_wc_transformed.at<double>(2,3) + (1-alpha_z)*sum_z;
         count = count + 1;
        }
        Rotn.getRotation(Quatn);
        Quatn.normalize();
        double roll, pitch, yaw;
        tf::Matrix3x3(Quatn).getRPY(roll, pitch, yaw);
        //std::cout << roll*180/M_PI << "," << pitch*180/M_PI << "," << yaw*180/M_PI << std::endl;
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Vector3 globalTranslation_rh(sum_x,
                                         sum_y,
                                         sum_z);
        transform.setOrigin(globalTranslation_rh);
        transform.setRotation(Quatn);
        br.sendTransform(tf::StampedTransform(transform, time_stamp, "map", "base_link"));
        pose_6d.header.frame_id = path_6d.header.frame_id = "map";
        pose_6d.header.stamp = path_6d.header.stamp = time_stamp;
     
        pose_6d.pose.position.x = sum_x;
        pose_6d.pose.position.y = sum_y;
        pose_6d.pose.position.z = sum_z;
        pose_6d.pose.orientation.x = Quatn[0];
        pose_6d.pose.orientation.y = Quatn[1];
        pose_6d.pose.orientation.z = Quatn[2];
        pose_6d.pose.orientation.w = Quatn[3];  


        pub_avg_pose.publish(pose_6d);
        path_6d.poses.push_back(pose_6d);
        path_publisher.publish(path_6d);
        initializing = false;
   }
   else {
   	if(initializing) {
   		ROS_INFO("Initializing");
        Quatn.normalize();
        double roll, pitch, yaw;
        tf::Matrix3x3(Quatn).getRPY(roll, pitch, yaw);
        //std::cout << roll*180/M_PI << "," << pitch*180/M_PI << "," << yaw*180/M_PI << std::endl;
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        tf::Vector3 globalTranslation_rh(sum_x,
                                         sum_y,
                                         sum_z);
        transform.setOrigin(globalTranslation_rh);
        transform.setRotation(Quatn);
        br.sendTransform(tf::StampedTransform(transform, time_stamp, "map", "base_link"));
        pose_6d.header.frame_id = path_6d.header.frame_id = "map";
        pose_6d.header.stamp = path_6d.header.stamp = time_stamp;
     
        pose_6d.pose.position.x = sum_x;
        pose_6d.pose.position.y = sum_y;
        pose_6d.pose.position.z = sum_z;
        pose_6d.pose.orientation.x = Quatn[0];
        pose_6d.pose.orientation.y = Quatn[1];
        pose_6d.pose.orientation.z = Quatn[2];
        pose_6d.pose.orientation.w = Quatn[3];  


        pub_avg_pose.publish(pose_6d);
        path_6d.poses.push_back(pose_6d);
        path_publisher.publish(path_6d);   	
   	} else {

   			  ROS_INFO("Can't see marker for unknown reasons");
	        Rotn.getRotation(Quatn);
	        Quatn.normalize();
	        double roll, pitch, yaw;
	        tf::Matrix3x3(Quatn).getRPY(roll, pitch, yaw);
	        //std::cout << roll*180/M_PI << "," << pitch*180/M_PI << "," << yaw*180/M_PI << std::endl;
	        static tf::TransformBroadcaster br;
	        tf::Transform transform;
	        tf::Vector3 globalTranslation_rh(sum_x,
	                                         sum_y,
	                                         sum_z);
	        transform.setOrigin(globalTranslation_rh);
	        transform.setRotation(Quatn);
	        br.sendTransform(tf::StampedTransform(transform, time_stamp, "map", "base_link"));
	        pose_6d.header.frame_id = path_6d.header.frame_id = "map";
	        pose_6d.header.stamp = path_6d.header.stamp = time_stamp;
	     
	        pose_6d.pose.position.x = sum_x;
	        pose_6d.pose.position.y = sum_y;
	        pose_6d.pose.position.z = sum_z;
	        pose_6d.pose.orientation.x = Quatn[0];
	        pose_6d.pose.orientation.y = Quatn[1];
	        pose_6d.pose.orientation.z = Quatn[2];
	        pose_6d.pose.orientation.w = Quatn[3];

	        pub_avg_pose.publish(pose_6d);
	        path_6d.poses.push_back(pose_6d);
	        path_publisher.publish(path_6d); 

   	}
   }
   msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_ptr->image).toImageMsg();
   msg->header.stamp = time_stamp;
   image_pub_.publish(msg);
   }
   else
   {
    ROS_INFO("Waiting for Images");
   }
   ros::spinOnce();
   loop_rate.sleep();
  }
  return 0;
}


