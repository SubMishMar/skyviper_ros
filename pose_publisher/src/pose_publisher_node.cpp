

#include "pose_publisher.h"
int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  PoseEstimator obj(nh, it);
  ros::spin();
  return 0;
}


