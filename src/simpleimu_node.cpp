#include "simpleimu_driver/simpleimu.h"
#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simpleimu");

  Simpleimu imu;

  ros::spin();
}
