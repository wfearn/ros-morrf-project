#include <ros/ros.h>
#include "morrf_ros/morrf_service.h"

int main( int argc, char** argv ) {
  ros::init( argc, argv, "morrf" );
  MORRFService morrf;  
  ros::spin();
  return 0;
}
