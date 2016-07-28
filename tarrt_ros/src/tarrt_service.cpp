#include <iostream>
#include "tpp/homotopy/img_load_util.h"
#include "tarrt_ros/tarrt_service.h"

using namespace std;
using namespace homotopy;
using namespace mlrrts;

#define TARRT_INIT_SERVICE_NAME "/tarrt/get_paths"
#define TARRT_CONT_SERVICE_NAME "/tarrt/refine_paths"

static double calc_dist( POS2D pos_a, POS2D pos_b, double** pp_distribution, void* tree ) {
  double dist = 0.0;
  if (pos_a == pos_b) {
    return dist;
  }
  double delta_x = fabs(pos_a[0]-pos_b[0]);
  double delta_y = fabs(pos_a[1]-pos_b[1]);
  dist = sqrt(delta_x*delta_x+delta_y*delta_y);

  if(dist < 0.0) {
    std::cout << "Dist negative " << dist << std::endl;
  }
  return dist;
}

static double calc_cost( POS2D pos_a, POS2D pos_b, double** pp_distribution, void* tree ) {
  double cost = 0.0;
  MLRRTstar* rrts = (MLRRTstar*)tree;
  if ( pos_a == pos_b ) {
    return cost;
  }
  if( pp_distribution == NULL ) {
    return cost;
  }

  float x1 = pos_a[0];
  float y1 = pos_a[1];
  float x2 = pos_b[0];
  float y2 = pos_b[1];

  const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
  if (steep) {
    std::swap(x1, y1);
    std::swap(x2, y2);
  }

  if (x1 > x2) {
    std::swap(x1, x2);
    std::swap(y1, y2);
  }

  const float dx = x2 - x1;
  const float dy = fabs(y2 - y1);

  float error = dx / 2.0f;
  const int ystep = (y1 < y2) ? 1 : -1;
  int y = (int)y1;

  const int maxX = (int)x2;

  for(int x=(int)x1; x<maxX; x++) {
    if(steep) {
      if (y>=0 && y<rrts->get_sampling_width() && x>=0 && x<rrts->get_sampling_height()) {
        cost += pp_distribution[y][x];
      }
    }
    else {
      if (x>=0 && x<rrts->get_sampling_width() && y>=0 && y<rrts->get_sampling_height()) {
        cost += pp_distribution[x][y];
      }
    }

    error -= dy;
    if(error < 0) {
      y += ystep;
      error += dx;
    }
  }
  return cost;
}

TARRTService::TARRTService() {
  mp_tarrt = NULL;
  m_tarrt_init_srv = m_nh.advertiseService( TARRT_INIT_SERVICE_NAME, &TARRTService::get_paths, this);
  m_tarrt_cont_srv = m_cont.advertiseService( TARRT_CONT_SERVICE_NAME, &TARRTService::refine_paths, this);
}

TARRTService::~TARRTService() {

}

bool TARRTService::get_paths( tarrt_ros::tarrt_initialize::Request& req,
                              tarrt_ros::tarrt_initialize::Response& res) {

  std::cout << "Starting TARRT..." << std::endl;
  delete_tarrt();

  int** pp_obstacle = new int*[req.init.map.width];
  for(unsigned int w=0; w < req.init.map.width; w++) {

    pp_obstacle[w] = new int[req.init.map.height];
    for(unsigned int h=0; h < req.init.map.height; h++) {
      pp_obstacle[w][h] = req.init.map.int_array[w+req.init.map.width*h];
    }
  }

  std::vector< std::vector<Point2D> > obstacles;
  load_map_info( pp_obstacle, req.init.map.width, req.init.map.height, obstacles );
  std::cout << "NUM OF OBS = " << obstacles.size() << std::endl;

  grammar_type_t grammar_type = STRING_GRAMMAR_TYPE;
  mp_reference_frame_set = new ReferenceFrameSet();
  mp_reference_frame_set->init(req.init.width, req.init.height, obstacles);
  if(req.init.sketched_topology.size() > 0) {
    std::vector< Point2D > ref_points;
    for( unsigned int i = 0; i < req.init.sketched_topology.size(); i ++ ) {
      geometry_msgs::Point p = req.init.sketched_topology[i];
      ref_points.push_back( Point2D( p.x, p.y ) );
    }
    if( mp_reference_frame_set ) {
      mp_reference_frame_set->import_string_constraint( ref_points, grammar_type );
    }
  }

  mp_tarrt = new MLRRTstar(req.init.width, req.init.height, req.init.segment_length);
  mp_tarrt->set_reference_frames( mp_reference_frame_set );
  POS2D start(req.init.start.x, req.init.start.y);
  POS2D goal(req.init.goal.x, req.init.goal.y);
  mp_tarrt->init(start, goal, func, fitness_distribution, grammar_type );  

  while(mp_tarrt->get_current_iteration() <= req.init.number_of_iterations) {
    mp_tarrt->extend();
  }

  std::vector<Path*> paths = mp_tarrt->get_paths();

  for(unsigned int i=0; i<paths.size(); i++) {
    Path* p = paths[i];
    if(p) {
      tarrt_ros::single_objective_path pp;
      for(unsigned int j=0; j<p->m_way_points.size(); j++) {
        geometry_msgs::Pose2D point;
        point.x = p->m_way_points[j][0];
        point.y = p->m_way_points[j][1];
        pp.waypoints.push_back(point); 
      }
      res.paths.push_back(pp);
    }
  }

  for(unsigned int w=0; w < req.init.map.width; w++) {
    delete [] pp_obstacle[w];
    pp_obstacle[w] = NULL;
  }
  delete [] pp_obstacle;

  std::cout << "TARRT finished!" << std::endl;

  return true;
}

bool TARRTService::refine_paths( tarrt_ros::tarrt_continue::Request& req,
                                 tarrt_ros::tarrt_continue::Response& res) {
  
  std::cout << "Refine paths of TARRT ... " << std::endl;
  
  if( mp_tarrt ) {
    unsigned int new_iterations = mp_tarrt->get_current_iteration() + req.iterations;
    while(mp_tarrt->get_current_iteration() <= new_iterations) {
      mp_tarrt->extend();
    }

    std::vector<Path*> paths = mp_tarrt->get_paths();

    for(unsigned int i=0; i<paths.size(); i++) {
      Path* p = paths[i];
      if(p) {
        tarrt_ros::single_objective_path pp;
        for(unsigned int j=0; j<p->m_way_points.size(); j++) {
          geometry_msgs::Pose2D point;
          point.x = p->m_way_points[j][0];
          point.y = p->m_way_points[j][1];
          pp.waypoints.push_back(point); 
        }
        res.paths.push_back(pp);
      }
    }
  }
 
  std::cout << "Refine paths of TARRT finished " << std::endl;
 
  return true;
}

void TARRTService::delete_tarrt() {

  if(mp_tarrt) {
    delete mp_tarrt;
    mp_tarrt = NULL;
  }
  if(mp_reference_frame_set) {
    delete mp_reference_frame_set;
    mp_reference_frame_set = NULL;
  }
  func = NULL;
  if(fitness_distribution) {
    unsigned int array_size = sizeof(fitness_distribution)/sizeof(int*);

    for(unsigned int w=0; w < array_size; w++) {
      delete [] fitness_distribution[w];
    }

    delete [] fitness_distribution;
    fitness_distribution = NULL;
  }
}
