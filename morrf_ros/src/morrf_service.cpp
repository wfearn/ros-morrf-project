#include <iostream>
#include "morrf/morrf.h"
#include "morrf_ros/morrf_service.h"
#include "std_msgs/Int64.h"

using namespace std;

#define MORRF_SERVICE_NAME "/morrf/get_multi_obj_paths"
#define MORRF_CONTINUE_SERVICE "/morrf/continue"


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
    morrf_continue = m_cont.advertiseService( MORRF_CONTINUE_SERVICE, &MORRFService::continuation, this);

    morrf_progress = p_pub.advertise<std_msgs::Float64>("morrf_status", 10000);

    morrf = NULL;
}

MORRFService::~MORRFService() {
}

bool MORRFService::continuation( morrf_ros::morrf_continue::Request& req,
                                 morrf_ros::morrf_continue::Response& res) {

    std::cout << "Continuing iterations on MORRF..." << std::endl;

    if(morrf != NULL) {

        int new_iterations = morrf->get_current_iteration() + req.iterations;
        float max_itr = req.iterations;
        int current_itr = 1;

        int i = req.iterations / 10;


        while(morrf->get_current_iteration() <= new_iterations) {
            morrf->extend();

            if (current_itr % i == 0) {

                std_msgs::Float64 p;
                p.data = ((current_itr / max_itr) * 100);

                morrf_progress.publish(p);
            }

            current_itr++;
        }

    std::cout << "MORRF iterations completed..." << std::endl;

    std::vector<Path*> paths = morrf->get_paths(false, false);

        for(unsigned int i=0; i < paths.size(); i++) {
            Path* p = paths[i];
            if(p) {

                morrf_ros::multi_objective_path pp;
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
        }

       std::cout << "MORRF finished!" << std::endl;

       return true;
    }

    return false;
}


bool MORRFService::get_multi_obj_paths( morrf_ros::morrf_initialize::Request& req,
                                        morrf_ros::morrf_initialize::Response& res) {

    std::cout << "Starting MORRF..." << std::endl;

    int** pp_obstacle = new int*[req.init.map.width];
    for(unsigned int w=0; w < req.init.map.width; w++) {

        pp_obstacle[w] = new int[req.init.map.height];

        for(unsigned int h=0; h < req.init.map.height; h++) {

            pp_obstacle[w][h] = req.init.map.int_array[w+req.init.map.width*h];
        }
    }

    this->delete_morrf();

    morrf = new MORRF(req.init.width, req.init.height, req.init.objective_number, req.init.number_of_trees,
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
                p_costmap[w][h] = img.int_array[w + img.width*h];
            }
        }

        funcs.push_back(calcCost);
        fitnessDistributions.push_back(p_costmap);
    }

    morrf->add_funcs(funcs, fitnessDistributions);
    POS2D start(req.init.start.x, req.init.start.y);
    POS2D goal(req.init.goal.x, req.init.goal.y);
    morrf->set_sparsity_k(2);
    std::vector< std::vector<float> > weights;
    morrf->init(start, goal, weights);
    morrf->load_map(pp_obstacle);
    morrf->set_boundary_intersection_penalty(req.init.boundary_intersection_penalty);

    std::cout << "Starting MORRF iterations..." << std::endl;

    while(morrf->get_current_iteration() <= req.init.number_of_iterations) {

      int i = req.init.number_of_iterations / 10;

      if(morrf->get_current_iteration() % i == 0) {


          std_msgs::Float64 p;

          float current_itr = morrf->get_current_iteration();
          float max_itr = req.init.number_of_iterations;

          p.data = ((current_itr / max_itr) * 75) + 25;
          morrf_progress.publish(p);
      }

      morrf->extend();
    }

    std::cout << "MORRF iterations completed..." << std::endl;

    std::vector<Path*> paths = morrf->get_paths(false, false);

    std::cout << "Paths obtained!" << std::endl;

    for(unsigned int i=0; i < paths.size(); i++) {

        Path* p = paths[i];

        if(p) {
            morrf_ros::multi_objective_path pp;
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
                //std::cout << "(" << point.x << ", " << point.y << ") "; 
            }
            //std::cout << std::endl;

            res.path_array.paths.push_back(pp);
        }
    }

    for(unsigned int w=0; w < req.init.map.width; w++) {
        delete [] pp_obstacle[w];
        pp_obstacle[w] = NULL;
    }

    delete [] pp_obstacle;

    std::cout << "MORRF finished!" << std::endl;

    return true;
}

void MORRFService::delete_morrf() {

    if(morrf != NULL) {
        delete morrf;
        morrf = NULL;

    }

    funcs.clear();

    for(unsigned int i = 0; i < fitnessDistributions.size(); i++) {
        if(fitnessDistributions[i] != NULL) {

            unsigned int array_size = sizeof(fitnessDistributions[i])/sizeof(int*);

        for(unsigned int w=0; w < array_size; w++) {
            delete [] fitnessDistributions[i][w];
        }

        delete [] fitnessDistributions[i];
        fitnessDistributions[i] = NULL;

       }

    }

    fitnessDistributions.clear();
}
