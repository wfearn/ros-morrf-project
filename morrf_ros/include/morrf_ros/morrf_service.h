#ifndef MORRF_SERVICE_H_
#define MORRF_SERVICE_H_

#include <ros/ros.h>
#include "morrf_ros/morrf_mopp.h"

class MORRFService {
public:
  MORRFService();
  virtual ~MORRFService();

  bool get_multi_obj_paths( morrf_ros::morrf_mopp::Request& req,
                            morrf_ros::morrf_mopp::Response& res); 
 
  ros::NodeHandle     m_nh;
  ros::ServiceServer  m_mopp_srv;
};

#endif // MORRF_SERVICE_H_
