#include <iostream>
#include <string>
#include "img_load_util.h"
#include "point.h"
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

const string filename = "/home/wfearn/catkin_ws/src/ros-morrf-project/costmap_generator/data/world_solids.png";
const string new_img_filename = "/home/wfearn/catkin_ws/src/ros-morrf-project/costmap_generator/data/new.png";

int main() {

    cout << "Starting image boundary process" << endl;

    Point p(2, 5);

    vector< vector<Point> > obstacles;

    int width = 0;
    int height = 0;

    load_map_info(filename, height, width, obstacles);

    cout << "Boundaries found" << endl;

    cv::Mat img(width, height, CV_8UC3, cv::Scalar(255, 255, 255));


    for(int i = 0; i < obstacles.size(); i++) {

        for(int j = 0; j < obstacles[i].size(); j++) {

            cv::Vec3b& bgr = img.at<cv::Vec3b>(obstacles[i][j].y, obstacles[i][j].x);
            bgr[0] = 0;
            bgr[1] = 0;
            bgr[2] = 0;
        }
    }

    vector<int> params;

    imwrite(new_img_filename, img, params);


    return 0;
}
