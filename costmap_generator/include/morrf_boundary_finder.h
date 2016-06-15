#include "ros/ros.h"
#include <string>
#include <vector>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "point.h"
#include "morrf_ros/int16_image.h"
#include "commander/get_cost_map.h"

bool find_boundaries( morrf_ros::int16_image &img,  morrf_ros::int16_image &boundary_image) {

    std::vector< std::vector<Point> > obstacles;

    std::cout << "find boundaries activated!" << std::endl;
    std::cout << "Image name is " << img.name << std::endl;

    //Multiplying int16_image has a single value representing a pixel
    int size = img.width * img.height;

    std::cout << "size is " << size << std::endl;

    uchar *img_array = new uchar[size];

    std::cout << "Converting image array into mat usable array" << std::endl;

    for(int i = 0; i < img.int_array.size(); i+=3) {

        img_array[i] = img.int_array[i];
    }

    obstacles.clear();

    std::cout << "Creating mat object" << std::endl;

    cv::Mat src(img.width, img.height, CV_8UC1, img_array);

    //src = cv::imread(filename,  CV_LOAD_IMAGE_GRAYSCALE);

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
    boundary_image.int_array = std::vector<int16_t>(img.width*img.height, 255);

    std::cout << "Changing boundary pixels black" << std::endl;

    for(int i = 0; i < obstacles.size(); i++) {
        for(int j = 0; j < obstacles[i].size(); j++) {

            //Appears that obstacles is indexed column major, so indexing is done accordingly
            int index = (obstacles[i][j].y * img.width) + obstacles[i][j].x;

            boundary_image.int_array[index] = 0;
        }
    }

    std::cout << "Successfully made it out" << std::endl;
    std::cout << "image name is " << boundary_image.name << std::endl;

    return true;
}
