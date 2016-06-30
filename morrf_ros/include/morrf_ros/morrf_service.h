#ifndef MORRF_SERVICE_H_
#define MORRF_SERVICE_H_

#include <ros/ros.h>
#include "morrf_ros/morrf_initialize.h"
#include "morrf_ros/morrf_continue.h"
#include "morrf/morrf.h"

class MORRFService {
public:
  MORRFService();
  virtual ~MORRFService();

  MORRF* morrf;

  bool get_multi_obj_paths( morrf_ros::morrf_initialize::Request& req,
                            morrf_ros::morrf_initialize::Response& res);

  bool continuation( morrf_ros::morrf_continue::Request& req,
                     morrf_ros::morrf_continue::Response& res);

  void delete_morrf();

  std::vector<COST_FUNC_PTR> funcs;
  std::vector<int**> fitnessDistributions;

  ros::NodeHandle m_nh;
  ros::NodeHandle m_cont;

  ros::ServiceServer m_mopp_srv;
  ros::ServiceServer morrf_continue;
};

#endif // MORRF_SERVICE_H_
