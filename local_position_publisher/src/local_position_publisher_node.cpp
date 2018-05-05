#include <ros/ros.h>
#include <iostream>
#include <math.h>
#include <tf/tf.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

using namespace std;

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
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "Quadrotor_pose_FCU"));
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "local_position_publisher");
  ros::NodeHandle nh_;
  ros::Subscriber sub = nh_.subscribe("/mavros/local_position/pose", 100, local_position_cb);
  ros::spin();
  return 0;
}


