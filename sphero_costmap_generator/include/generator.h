﻿#ifndef GENERATOR_H
#define GENERATOR_H
#include <iostream>
#include "KDTree2D.h"
#include "ros/ros.h"
#include "morrf_ros/int16_image.h"
#include "commander/outputVals.h"
#include "commander/point_cost.h"
#include <list>
#include <vector>
#include <cmath>
#include <set>
#include <string>
#include <sstream>
#include "point.h"
#include <QString>
#include <QPixmap>
#include <QImage>
#include <QColor>
#include <QPoint>
#include <QPainter>
#include <fstream>
#include <QFileDialog>
#include <QDebug>

//using namespace std;

const double MIN_GRAYSCALE_VALUE = 10;
const double MAX_GRAYSCALE_VALUE = 255;
const QColor WHITE = QColor(Qt::white);
const QColor BLACK = QColor(Qt::black);
// const double delta = 5.0;


class Generator
{

    private:
        double diag_distance;
        double slope;
        double y_intercept;
        double delta;
        double maxProbOfSeenVal;
        double minProbOfSeenVal;
        double minProbOfNearness;
        double maxProbOfNearness;
        int width;
        int height;

	KDTree2D* kd_obs;
        std::list<Point> obsBoundaryPts;
        std::set<std::string> allObsPts, enemyPtsToIgnore;

        void getAllObsPts(morrf_ros::int16_image img);
        void getImgBoundaryPts(morrf_ros::int16_image img);
        void getEnemyPtsToIgnore(std::list<Point> enemyPts);
	void getObsBoundaryPts_(morrf_ros::int16_image img);	

        bool isOnLineSegment(Point a, Point b, Point c);
        void writeImage(morrf_ros::int16_image &cost_map, commander::outputVals &ov, std::vector<std::vector<double> > imgProbVals);
        void writeSafeImage(morrf_ros::int16_image &cost_map, commander::outputVals &ov, std::vector<std::vector<double> > imgProbVals);
        void resize(std::vector<std::vector<double> > & array);
        std::vector<double> setEnemyProbValues(bool isBlocked, double distance);
        bool isBlocked(Point imgPt, Point enemyPt);
        double getNearObsValue(Point pt);
        void clear();
	void populateStealthProbVals(std::vector<std::vector<double > > &imgProbVals, std::list<Point> enemyPts);
	void populateSafeProbVals(std::vector<std::vector<double> > &imgProbVals);

    public:
        Generator();
        void probOfSeenByEnemy(std::list<Point> enemyPts, morrf_ros::int16_image worldImg, morrf_ros::int16_image boundaryImg, morrf_ros::int16_image &cost_map, commander::outputVals &ov);
        void probOfBeingNearToObstacle(std::list<Point> enemyPts, morrf_ros::int16_image worldImg, morrf_ros::int16_image boundaryImg, morrf_ros::int16_image &cost_map, commander::outputVals &ov);

};

#endif // GENERATOR_H
