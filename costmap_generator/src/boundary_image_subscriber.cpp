#include "ros/ros.h"
#include "commander/test_boundary_img.h"
#include "morrf_ros/int16_image.h"
#include "morrf_boundary_finder.h"
#include <vector>
#include "point.h"

using namespace std;


bool get_boundary(commander::test_boundary_img::Request &req,
                  commander::test_boundary_img::Response &res) {

    //vector< vector<Point> > obstacles;

    find_boundaries(req.map, res.boundary_image);

    cout << "Boundary image received: " << res.boundary_image.name << endl;

    return true;
}

int main(int argc, char **argv) {

    ros::init(argc, argv, "find_boundary_image_server");
    ros::NodeHandle n;

    ros::ServiceServer s = n.advertiseService("/morrf/get_boundary_image", get_boundary);
    ROS_INFO("Ready to receive image.");
    ros::spin();

    return 0;
}
