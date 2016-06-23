#include "ros/ros.h"
#include "commander/get_cost_map.h"
#include "morrf_ros/int16_image.h"

#include <vector>
#include <list>

#include "point.h"
#include "generator.h"
#include "morrf_boundary_finder.h"
#include "array_converter.h"

bool generate_costmap(commander::get_cost_map::Request &req,
                         commander::get_cost_map::Response &res) {

            Generator generator = Generator();

            ROS_INFO("[Listener] I received image, bool, bool\n");

            morrf_ros::int16_image bound;
            find_boundaries(req.map, bound);

            print_array_image(bound, "/home/tkatuka/Pictures/test_boundary.png");

            list<Point> enemy_points;

            for(int i = 0; i < req.enemyPts.size(); i++) {

                Point pos(req.enemyPts[i].x, req.enemyPts[i].y);
                enemy_points.push_back(pos);
            }


            if(req.stealth == 1) {

                std::cout << "Stealthy active " << std::endl;

                morrf_ros::int16_image cost;
                commander::outputVals ov;

                generator.probOfSeenByEnemy(enemy_points, req.map, bound, cost, ov);

                print_array_image(cost, "/home/tkatuka/Pictures/stealth_cost.png");
		            write_output_vals(ov, "/home/tkatuka/Pictures/stealth.txt");
            }

            if(req.safe == 1) {
                std::cout << "Safety active " << std::endl;

		            morrf_ros::int16_image cost;
                commander::outputVals ov;

                generator.probOfBeingNearToObstacle(enemy_points, req.map, bound, cost, ov);
                print_array_image(cost, "/home/tkatuka/Pictures/safe_cost.png");
                write_output_vals(ov, "/home/tkatuka/Pictures/safe.txt");
            }
            return true;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "costmap_subscriber");

    ros::NodeHandle n;

    ros::ServiceServer s = n.advertiseService("/morrf/get_cost_map", generate_costmap);

    ros::spin();

    return 0;
}
