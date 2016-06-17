#include "ros/ros.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include "point.h"
#include "morrf_ros/int16_image.h"
#include <string>

cv::Mat get_cv_image(morrf_ros::int16_image img) {

    cv::Mat image(img.height, img.width, CV_8UC1, cv::Scalar(255));

    for(int i = 0; i < img.height; i++) {
        for(int j = 0; j < img.width; j++) {

            int index = i * img.width + j;
            image.at<uchar>(i, j) = img.int_array[index];
        }
    }

    return image;
}

void print_array_image(morrf_ros::int16_image img, std::string filename) {

    imwrite(filename, get_cv_image(img));
}

morrf_ros::int16_image get_int_array(cv::Mat img) {
    morrf_ros::int16_image image;

    for(int i = 0; i < img.rows; i++) {
        for(int j = 0; j < img.cols; j++) {

            image.int_array.push_back(img.at<uchar>(i, j));
        }
    }

}
