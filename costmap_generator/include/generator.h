#ifndef GENERATOR_H
#define GENERATOR_H
#include <iostream>
#include <list>
#include <vector>
#include <math.h>
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

using namespace std;

const double MIN_GRAYSCALE_VALUE = 10;
const double MAX_GRAYSCALE_VALUE = 255;
const QColor WHITE = QColor(Qt::white);
const QColor BLACK = QColor(Qt::black);

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

    list<Point> obsBoundaryPts;
    set<string> allObsPts, enemyPtsToIgnore;

    void getAllObsPts(QImage image);
    void getEnemyPtsToIgnore(list<Point> enemyPts);
    bool isOnLineSegment(Point a, Point b, Point c);
    void writeImage(string file, string output, vector<vector<double> > imgProbVals);
    void writeSafeImage(string file, string output, vector<vector<double> > imgProbVals);
    void resize(vector<vector<double>> & array);
    vector<double> setEnemyProbArray(int i, bool blocked, double distance);
    bool isBlocked(Point imgPt, Point enemyPt);
    double getNearObsValue(Point pt);
    void clear();

public:
    Generator();
    void probOfSeenByEnemy(double delta, list<Point> enemyPts, string world_boundaries, string world_solids, string imageFile, string outputFile);
    void getImgBoundaryPts(QImage image);
    void probOfBeingNearToObstacle(double delta, string world_boundaries, string world_solids, list<Point> enemyPts, string safeObjImgFilename, string result);

};

#endif // GENERATOR_H
