#include "ros/ros.h"
#include "commander/get_cost_map.h"
#include "morrf_ros/int16_image.h"
#include <vector>
#include "point.h"
#include "generator.h"

void generate_costmap(costmap_generator::get_cost_map::Request &req,
								 costmap_generator::get_cost_map::Response &res) {

			Generator generator;
			ROS_INFO("[Listener] I received image, bool, bool\n");
			req.map // is of type int16_image, needs to be converted to QImage
			if(req.stealth == 1)
				res.cost_maps.push_back(probOfSeenByEnemy());
			if(req.safe == 1)
				res.cost_maps.push_back(probOfBeingNearToObstacle());
			list<Point> enemies;
			for(int i = 0; i < req.enemyPts.size(); i++) {
				Point pos;
				pos.x = (int) req.enemyPts[i].x;
				pos.y = (int) req.enemyPts[i].y;
				//pause
			}
			generator()

}


int main(int argc, char **argv) {

	ros::init(argc, argv, "costmap_subscriber");

	//reference to new node
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/morrf/get_info", 1000, generate_costmap);

	ros::spin();

	return 0;
}
