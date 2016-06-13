#include "ros/ros.h"
#include <string>
#include <vector>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "point.h"
#include "morrf_ros/int16_image.h"
#include "commander/get_cost_map.h"

morrf_ros::int16_image find_boundaries( morrf_ros::int16_image img,  std::vector< std::vector<Point> >& obstacles ) {

    std::cout << "find boundaries activated!" << std::endl;

    //Multiplying int16_image has a single value representing a pixel
    int size = img.width * img.height;

    uchar *img_array = new uchar[size];

    for(int i = 0; i < img.int_array.size(); i+=3) {

        img_array[i] = img.int_array[i];
    }

    obstacles.clear();

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

    morrf_ros::int16_image im;
    im.name = "Boundary Image";
    im.width = img.width;
    im.height = img.height;

    /* 
    for(int i = 0; i < size; i++) {

        im.int_array.push_back(255);

    } */
    im.int_array = std::vector<int16_t>(img.width*img.height, 255);

    for(int i = 0; i < obstacles.size(); i++) {
        for(int j = 0; j < obstacles[i].size(); j++) {

            //Appears that obstacles is indexed column major, so indexing is done accordingly
            int index = (j * img.width) + i;
            im.int_array[index] = 0;

        }
    }

    return im;
}
