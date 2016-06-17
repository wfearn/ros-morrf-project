#include <QCoreApplication>
#include "generator.h"
#include "img_load_util.h"
#include "array_converter.h"
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ctime>

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    // Generator generator;
    // double delta = 5;
    uint start = clock();

    // list<Point> enemyPts;
    // Point enemy1(115, 100);
    // Point enemy2(380, 255);
    //
    // enemyPts.push_back(enemy1);
    // enemyPts.push_back(enemy2);
    //
    cout << "It started." << endl;

    string obsSolidImg = "/home/tkatuka/catkin_ws/src/ros-morrf-project/costmap_generator/data/world_solids.png";
    string obsBoundaryImg = "/home/tkatuka/catkin_ws/src/ros-morrf-project/costmap_generator/data/world_boundaries.png";

    int width = 0;
    int height = 0;

    // load_map_info(obsSolidImg, width, height, obstacles);

    cv::Mat img = cv::imread(obsSolidImg, 0);

    morrf_ros::int16_image resultImage = get_int_array(img);

    print_array_image(resultImage, "/home/tkatuka/catkin_ws/src/ros-morrf-project/costmap_generator/data/test.png");

    return 0;

  //   vector<vector<Point> > obstacles;
   //
  //   for(int i = 0; i < obstacles.size(); i++) {
   //
  //       for(int j = 0; j < obstacles[i].size(); j++) {
   //
  //           cv::Vec3b& bgr = img.at<cv::Vec3b>( obstacles[i][j].y, obstacles[i][j].x);
  //           bgr[0] = 0;
  //           bgr[1] = 0;
  //           bgr[2] = 0;
  //       }
  //   }
   //
  //   vector<int> params;
   //
  //   imwrite(obsBoundaryImg, img, params);
   //
   //
  //   string stealthOutputText = "/home/tkatuka/catkin_ws/src/ros-morrf-project/costmap_generator/data/stealthOutputVals.txt";
  //   string stealthOutputImage = "/home/tkatuka/catkin_ws/src/ros-morrf-project/costmap_generator/data/stealthCostMap.png";
  //   generator.probOfSeenByEnemy(delta, enemyPts, obsBoundaryImg, obsSolidImg, stealthOutputImage, stealthOutputText);
   //
  //  string safeOutputText = "/home/tkatuka/catkin_ws/src/ros-morrf-project/costmap_generator/data/safeOutputVals.txt";
  //  string safeOutputImage = "/home/tkatuka/catkin_ws/src/ros-morrf-project/costmap_generator/data/safeCostMap.png";
  //  generator.probOfBeingNearToObstacle(delta, obsBoundaryImg, obsSolidImg, enemyPts, safeOutputImage, safeOutputText);
   //
  //   uint finish = clock();
   //
  //   double milliseconds = finish - start;
  //   cout << "Elapsed time: " << milliseconds << "ms" << endl;
  //   return 0;
}
