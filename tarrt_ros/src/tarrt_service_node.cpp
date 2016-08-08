#include <ros/ros.h>
#include "tarrt_ros/tarrt_service.h"

int main( int argc, char** argv ) {
  ros::init( argc, argv, "tarrt" );
  TARRTService tarrt_service;
  ros::spin();
  return 0;
}
