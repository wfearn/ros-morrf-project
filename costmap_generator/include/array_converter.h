#pragma once

#include "ros/ros.h"
#include "morrf_ros/int16_image.h"
#include "commander/outputVals.h"
#include "commander/point_cost.h"

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>

#include "point.h"
#include <string>
#include <iostream>
#include <fstream>

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
    image.name = "OpenCVImage";
    image.width = img.cols;
    image.height = img.rows;

    for(int i = 0; i < image.height; i++) {
        for(int j = 0; j < image.width; j++) {

            image.int_array.push_back(img.at<uchar>(i, j));
        }
    }

    return image;
}

void write_output_vals(commander::outputVals ov, string filename) {

    std::ofstream f;

    std::cout << "Printing output vals " << ov.name << std::endl;

    f.open(filename);

    for(int i = 0; i < ov.vals.size(); i++) {
        f << ov.vals[i].position.x << " " << ov.vals[i].position.y << "\t" << ov.vals[i].cost << std::endl;
    }

    f.close();
}
