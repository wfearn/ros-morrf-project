#include "ros/ros.h"
#include "commander/get_cost_map.h"
#include "morrf_ros/int16_image.h"
#include <vector>
#include "point.h"
#include "generator.h"

bool generate_costmap(commander::get_cost_map::Request &req,
                         commander::get_cost_map::Response &res) {

            //Generator generator;

            ROS_INFO("[Listener] I received image, bool, bool\n");
            std::cout << "Recieved image: " << req.map.name << std::endl;

            //req.map // is of type int16_image, needs to be converted to QImage
            //
            if(req.stealth == 1)
                std::cout << "Stealth active " << std::endl;
                //res.cost_maps.push_back(probOfSeenByEnemy());

            if(req.safe == 1)
                std::cout << "Safety active " << std::endl;
                // morrf_ros::int16_image resultImage;
                // commander::outputVals resultValsl;
                // generator.probOfSeenByEnemy(world_solids, boun, resultImage, resultVals);
                //res.cost_maps.push_back(probOfBeingNearToObstacle());
                //res.cost_values.push_back

            //list<Point> enemies;

            for(int i = 0; i < req.enemyPts.size(); i++) {
                //Point pos;
                //pos.x = (int) req.enemyPts[i].x;
                //pos.y = (int) req.enemyPts[i].y;
                ////pause
            }

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
