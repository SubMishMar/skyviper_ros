#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <tf/tf.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Path.h>

using namespace std;
nav_msgs::Path path_6d;
geometry_msgs::PoseStamped pose_6d;
void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  double x = msg->pose.position.x;
  double y = msg->pose.position.y;
  double z = msg->pose.position.z;
  double qx = msg->pose.orientation.x;
  double qy = msg->pose.orientation.y;
  double qz = msg->pose.orientation.z;
  double qw = msg->pose.orientation.w;

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Vector3 globalTranslation_rh(x,
                                   y,
                                   z);
  transform.setOrigin(globalTranslation_rh);
  tf::Quaternion Quatn(qx, qy, qz, qw);
  transform.setRotation(Quatn);
  br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "map", "local_position"));
  pose_6d.header.frame_id = path_6d.header.frame_id = "map";
  pose_6d.header.stamp = path_6d.header.stamp = msg->header.stamp;

  pose_6d.pose.position.x = x;
  pose_6d.pose.position.y = y;
  pose_6d.pose.position.z = z;
  pose_6d.pose.orientation.x = qx;
  pose_6d.pose.orientation.y = qy;
  pose_6d.pose.orientation.z = qz;
  pose_6d.pose.orientation.w = qw;  
  path_6d.poses.push_back(pose_6d);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "local_position_publisher");
  ros::NodeHandle nh_;
  ros::Subscriber sub = nh_.subscribe("/mavros/local_position/pose", 100, local_position_cb);
  ros::Publisher path_publisher = nh_.advertise<nav_msgs::Path>("/local_position/path", 1000);

  ros::Rate loop_rate(30);
  while(ros::ok()) {
  	path_publisher.publish(path_6d);
  	ros::spinOnce();
  	loop_rate.sleep();
  }
  ros::spin();
  return 0;
}


