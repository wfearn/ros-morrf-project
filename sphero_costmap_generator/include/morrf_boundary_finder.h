#include "ros/ros.h"
#include <string>
#include <vector>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "point.h"
#include "morrf_ros/int16_image.h"
#include "commander/get_cost_map.h"
#include "array_converter.h"

bool find_boundaries( morrf_ros::int16_image &img,  morrf_ros::int16_image &boundary_image) {

    std::vector< std::vector<Point> > obstacles;

    int size = img.width * img.height;

    obstacles.clear();

    cv::Mat src = get_cv_image(img);

    cv::threshold(src, src, 200, 255, cv::THRESH_BINARY_INV);

    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;

    cv::findContours( src, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE );

    for( unsigned int i=0; i<contours.size(); i++ ) {

        std::vector<Point> cont;

        for( std::vector<cv::Point>::iterator it=contours[i].begin(); it!=contours[i].end(); it++ ) {

           cv::Point p = (cv::Point)(*it);
           Point ip(p.x, p.y);
           cont.push_back(ip);

        }

        obstacles.push_back(cont);
    }

    boundary_image.name = "Boundary Image";
    boundary_image.width = img.width;
    boundary_image.height = img.height;

    for(unsigned int i=0; i<img.width*img.height; i++) {
      boundary_image.int_array.push_back(255);
    }

  //Changing boundary points from obstacles vector to black pixels
    for(int i = 0; i < obstacles.size(); i++) {
        for(int j = 0; j < obstacles[i].size(); j++) {

            int index = (obstacles[i][j].y * img.width) + obstacles[i][j].x;

            boundary_image.int_array[index] = 0;
        }
    }

    return true;
}
