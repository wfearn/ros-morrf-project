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
            std::cout << "Recieved image: " << req.map.name << std::endl;

            morrf_ros::int16_image bound;

            find_boundaries(req.map, bound);

            print_array_image(bound, "/home/wfearn/Pictures/test_boundary.png");

            list<Point> enemy_points;

            for(int i = 0; i < req.enemyPts.size(); i++) {
                Point pos(req.enemyPts[i].x, req.enemyPts[i].y);
                //pos.x = (int) req.enemyPts[i].x;
                //pos.y = (int) req.enemyPts[i].y;
                enemy_points.push_back(pos);
            }


            //req.map // is of type int16_image, needs to be converted to QImage
            //
            if(req.stealth == 1) {
                std::cout << "Stealth active " << std::endl;

                morrf_ros::int16_image cost;
                commander::outputVals ov;

                generator.probOfSeenByEnemy(enemy_points, req.map, bound, cost, ov);

                print_array_image(cost, "/home/wfearn/Pictures/stealth_cost.png");
                //res.cost_maps.push_back(probOfSeenByEnemy());
            }

            if(req.safe == 1) {
                std::cout << "Safety active " << std::endl;
                // morrf_ros::int16_image resultImage;
                // commander::outputVals resultValsl;
                // generator.probOfSeenByEnemy(world_solids, boun, resultImage, resultVals);
                //res.cost_maps.push_back(probOfBeingNearToObstacle());
                //res.cost_values.push_back
            }

            //list<Point> enemies;

            //generator();

            return true;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "costmap_subscriber");

    //reference to new node
    ros::NodeHandle n;

    ros::ServiceServer s = n.advertiseService("/morrf/get_cost_map", generate_costmap);

    ros::spin();

    return 0;
}
