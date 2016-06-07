#include <iostream>
#include "morrf/morrf.h"
#include "morrf_ros/morrf_service.h"

using namespace std;

#define MORRF_SERVICE_NAME "/morrf/get_multi_obj_paths"


static double calcDist(POS2D pos_a, POS2D pos_b, int** distribution, void* tree) {
  double dist = 0.0;
  if (pos_a == pos_b) {
    return dist;
  }
  double delta_x = fabs(pos_a[0]-pos_b[0]);
  double delta_y = fabs(pos_a[1]-pos_b[1]);
  dist = sqrt(delta_x*delta_x+delta_y*delta_y);
 
  return dist;
}

static double calcCost(POS2D pos_a, POS2D pos_b, int** pp_distribution, void* tree) {
  MORRF* morrf = (MORRF*)tree;
  double cost = 0.0;
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
      if( y>=0 && y<morrf->get_sampling_width() && x>=0 && x<morrf->get_sampling_height() ) {
        double dist = static_cast<double>( pp_distribution[y][x] );
        cost += dist / 255.0;
      }
    }
    else {
      if( x>=0 && x<morrf->get_sampling_width() && y>=0 && y<morrf->get_sampling_height() ) {
        double dist = static_cast<double>( pp_distribution[x][y] );
        cost += dist / 255.0;
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

MORRFService::MORRFService() {

  m_mopp_srv = m_nh.advertiseService( MORRF_SERVICE_NAME, &MORRFService::get_multi_obj_paths, this);
}

MORRFService::~MORRFService() {
}
  

bool MORRFService::get_multi_obj_paths( morrf_ros::morrf_mopp::Request& req,
                                        morrf_ros::morrf_mopp::Response& res) {
  std::cout << "---------------------------------" << std::endl;
  std::cout << "MORRFService::get_multi_obj_paths" << std::endl;
  std::cout << "SERVICE RECEIVED" << std::endl;
  //std::cout << req.init << std::endl;
  std::cout << "size:" << req.init.width << "*" << req.init.height << std::endl;
  std::cout << "num of obj: " << req.init.objective_number << " , num of sub: " << req.init.number_of_trees;
  std::cout << "segment: " << req.init.segment_length << " method:" << req.init.method_type << std::endl;
  std::cout << "start x:" << req.init.start.x << " y:" << req.init.start.y << std::endl;
  std::cout << "goal x:" << req.init.goal.x << " y:" << req.init.goal.y << std::endl;
  std::cout << "num of iteration:" << req.init.number_of_iterations << std::endl;
  std::cout << "---------------------------------" << std::endl;

  std::vector<COST_FUNC_PTR> funcs;
  std::vector<int**> fitnessDistributions;

  int** pp_obstacle = new int*[req.init.map.width];
  for(unsigned int w=0; w < req.init.map.width; w++) {
    pp_obstacle[w] = new int[req.init.map.height];
    for(unsigned int h=0; h < req.init.map.height; h++) {
      pp_obstacle[w][h] = req.init.map.int_array[w*req.init.map.height+h];
    }
  }
  MORRF morrf(req.init.width, req.init.height, req.init.objective_number, req.init.number_of_trees,
              req.init.segment_length, (MORRF::MORRF_TYPE)req.init.method_type);

  if(req.init.minimum_distance_enabled == true) {
    funcs.push_back(calcDist);
    fitnessDistributions.push_back(NULL);
  }

  for(unsigned int i=0; i < req.init.cost_maps.size(); i++) {
    morrf_ros::int16_image img = req.init.cost_maps[i];
    int** p_costmap = new int*[img.width];
    for(unsigned int w=0; w < img.width; w++) {
      p_costmap[w] = new int[img.height];
      for(unsigned int h=0; h < img.height; h++) {
        p_costmap[w][h] = img.int_array[w*img.height+h];
      }
    }
    funcs.push_back(calcCost);
    fitnessDistributions.push_back(p_costmap);
  }

  morrf.add_funcs(funcs, fitnessDistributions);
  POS2D start(req.init.start.x, req.init.start.y);
  POS2D goal(req.init.goal.x, req.init.goal.y);
  morrf.init(start, goal);
  morrf.load_map(pp_obstacle);
  
  /*
  std::cout << "dump map info " << std::endl;
  morrf.dump_map_info("./test_obs.txt");
  */
  std::cout << "start planning" << std::endl;
  while(morrf.get_current_iteration() <= req.init.number_of_iterations) {
    morrf.extend();
    std::cout << "Iteration " << morrf.get_current_iteration() << std::endl;
  }

  std::vector<Path*> paths = morrf.get_paths();
  for(unsigned int i=0; i < paths.size(); i++) {
    morrf_ros::multi_objective_path pp;
    Path* p = paths[i];
    for(unsigned int k=0; k < p->m_objective_num; k++ ) {
      std_msgs::Float64 val;
      val.data = p->m_cost[k];
      pp.cost.push_back(val);
    }
    for(unsigned int j=0; j < p->m_waypoints.size(); j++) {
      geometry_msgs::Pose2D point;
      point.x = p->m_waypoints[j][0];
      point.y = p->m_waypoints[j][1];
      pp.waypoints.push_back(point);
    }
    res.paths.push_back(pp);
  }
  std::cout << "Path exported " << std::endl;

  for(unsigned int w=0; w < req.init.map.width; w++) {
    delete [] pp_obstacle[w];
    pp_obstacle[w] = NULL;
  }
  delete [] pp_obstacle;

  funcs.clear();
  for(unsigned int i=0; i < fitnessDistributions.size(); i++) {
    if(fitnessDistributions[i] != NULL) {
      for(unsigned int w=0; w < req.init.width; w++) {
        delete [] fitnessDistributions[i][w];
      }
      delete [] fitnessDistributions[i];
      fitnessDistributions[i] = NULL;
    }
  }
  fitnessDistributions.clear();

  return true;
}



