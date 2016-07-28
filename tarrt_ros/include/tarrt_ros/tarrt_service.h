#ifndef TARRT_SERVICE_H_
#define TARRT_SERVICE_H_

#include <ros/ros.h>
#include "tarrt_ros/tarrt_initialize.h"
#include "tarrt_ros/tarrt_continue.h"
#include "tpp/tarrt/mlrrtstar.h"

class TARRTService {
public:
  TARRTService();
  virtual ~TARRTService();

  mlrrts::MLRRTstar*           mp_tarrt;
  homotopy::ReferenceFrameSet* mp_reference_frame_set;

  bool get_paths( tarrt_ros::tarrt_initialize::Request& req,
                  tarrt_ros::tarrt_initialize::Response& res);

  bool refine_paths( tarrt_ros::tarrt_continue::Request& req,
                     tarrt_ros::tarrt_continue::Response& res);

  void delete_tarrt();

  mlrrts::COST_FUNC_PTR func;
  double**              fitness_distribution;

  ros::NodeHandle m_nh;
  ros::NodeHandle m_cont;

  ros::ServiceServer m_tarrt_init_srv;
  ros::ServiceServer m_tarrt_cont_srv;
};

#endif // TARRT_SERVICE_H_
