#include "kalmanfilter.h"
int main(int argc, char** argv)
{
  ros::init(argc, argv, "kf");
  ros::NodeHandle nh;
  kalmanFilter obj(nh);
  ros::spin();
  return 0;
}
