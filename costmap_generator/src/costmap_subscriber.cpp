#include "ros/ros.h"
#include "commander/get_cost_map.h"
#include "morrf_ros/int16_image.h"

#include <vector>
#include <list>

#include "point.h"
#include "generator.h"
#include "morrf_boundary_finder.h"
#include "array_converter.h"

using namespace std;

bool generate_costmap(commander::get_cost_map::Request &req,
                         commander::get_cost_map::Response &res) {

        Generator generator = Generator();

        morrf_ros::int16_image bound;
        find_boundaries(req.map, bound);

        res.boundary_image = bound;

        list<Point> enemy_points;

        for(int i = 0; i < req.enemyPts.size(); i++) {

            Point pos(req.enemyPts[i].x, req.enemyPts[i].y);
            enemy_points.push_back(pos);

        }

        if(req.stealth == 1) {

            std::cout << "Generating stealth costmap..." << endl;

            morrf_ros::int16_image cost;
            commander::outputVals ov;

            generator.probOfSeenByEnemy(enemy_points, req.map, bound, cost, ov);

            res.cost_maps.push_back(cost);
            res.cost_values.push_back(ov);

        }

        if(req.safe == 1) {

            std::cout << "Generating safety costmap..." << endl;

            morrf_ros::int16_image cost;
            commander::outputVals ov;

            generator.probOfBeingNearToObstacle(enemy_points, req.map, bound, cost, ov);

            print_array_image(cost, "/home/robotron5000/Documents/old_safety.png");
            res.cost_maps.push_back(cost);
            res.cost_values.push_back(ov);

        }

        std::cout << "Costmaps generated!" << endl;

        /*Meher's Parameters
        1. Map Scores FileName
        2. Map PathPts FileName
        3. obs Img - req.map
        4. obs boundary Img - bound
        5. Stealth Values - cost_values
        6. Safe values - cost_values
        7. Start Pt - morrf_ros::morrf_init.start
        8. Goal - morrf_ros::morrf_init.goal
        9. enemyPts - req.enemyPts
        */
        return true;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "costmap_subscriber");

    ros::NodeHandle n;

    ros::ServiceServer s = n.advertiseService("/morrf/get_cost_map", generate_costmap);

    ros::spin();

    return 0;
}
