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

            print_array_image(bound, "/home/tkatuka/Pictures/test_boundary.png");

            list<Point> enemy_points;

            cout << "enemy points" << endl;

            for(int i = 0; i < req.enemyPts.size(); i++) {

                Point pos(req.enemyPts[i].x, req.enemyPts[i].y);
                cout << pos.x << ", " << pos.y << endl;

                enemy_points.push_back(pos);
            }


            //req.map // is of type int16_image, needs to be converted to QImage
            if(req.stealth == 1) {
                std::cout << "Stealth active " << std::endl;

                morrf_ros::int16_image cost;
                commander::outputVals ov;

                generator.probOfSeenByEnemy(enemy_points, req.map, bound, cost, ov);

                print_array_image(cost, "/home/tkatuka/Pictures/stealth_cost.png");
		write_output_vals(ov, "/home/tkatuka/Pictures/stealth.txt");
                //res.cost_maps.push_back(probOfSeenByEnemy());
            }

            if(req.safe == 1) {
                std::cout << "Safety active " << std::endl;
                
		morrf_ros::int16_image cost;
                commander::outputVals ov;

                generator.probOfBeingNearToObstacle(enemy_points, req.map, bound, cost, ov);
                cout << "function completed " << endl; 
		print_array_image(cost, "/home/tkatuka/Pictures/safe_cost.png");
                write_output_vals(ov, "/home/tkatuka/Pictures/safe.txt");
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
