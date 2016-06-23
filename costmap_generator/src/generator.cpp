#include "generator.h"

Generator::Generator()
{
    diag_distance = 0;
    slope = 0;
    y_intercept = 0;
    delta = 5.0;
    maxProbOfSeenVal = 0;
    minProbOfSeenVal = 0;
    minProbOfNearness = 0;
    maxProbOfNearness = 0;
    width = 0;
    height = 0;
}

bool Generator::isOnLineSegment(Point a, Point b, Point c){

  double AB, AC, CB;
  AB = AC = CB = 0.0;

  AB = sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
  AC = sqrt((c.x - a.x) * (c.x - a.x) + (c.y - a.y) * (c.y - a.y));
  CB = sqrt((b.x - c.x) * (b.x - c.x) + (b.y - c.y) * (b.y - c.y));

  double sumWithR = AC + CB;

  if(abs((sumWithR - AB)) < 0.01)
    return true;
  return false;
}

void Generator::probOfSeenByEnemy(list<Point> enemyPts, morrf_ros::int16_image worldImg, morrf_ros::int16_image boundaryImg, morrf_ros::int16_image &cost_map, commander::outputVals &ov){

    width = worldImg.width; //width of the world_boundaries image
    height = worldImg.height; //height of the world_boundaries image

    diag_distance = sqrt(pow(width, 2) + pow(height, 2));
    slope = 1/(delta - diag_distance);
    y_intercept = (-1) * (diag_distance * slope);

    //getting all the points
    getAllObsPts(worldImg);
    getImgBoundaryPts(boundaryImg);
    getEnemyPtsToIgnore(enemyPts);

    minProbOfSeenVal = 2.0; // some reason for that
    maxProbOfSeenVal = 0.0;

    vector<vector<double> > imgProbVals;
    resize(imgProbVals);

    int numOfEnemies = enemyPts.size();
    //cout << "Enmies: " << numOfEnemies << endl;	
    //std::cout << "Entering first nested for loops" << std::endl;

    for(int y = 0; y < height; y++) {
        for(int x = 0; x < width; x++) {

            string s = to_string(x) + " " + to_string(y);
            if(allObsPts.count(s) || enemyPtsToIgnore.count(s))
            {
               continue;
            }

            double enemyProbArray[numOfEnemies][2];
            int i = 0;

            for(Point enemyPt : enemyPts) {

                bool blocked = isBlocked(Point(x, y), enemyPt);
                double distanceToEnemy = sqrt((x - enemyPt.x) * (x - enemyPt.x) + (y - enemyPt.y) * (y - enemyPt.y));

                vector<double> probability = setEnemyProbValues(blocked, distanceToEnemy);
                enemyProbArray[i][0] = probability.at(0);
                enemyProbArray[i][1] = probability.at(1);
                i++;
            }

            double allFalseProduct = 1;
            for(int i = 0; i < numOfEnemies; i++) {
                allFalseProduct *= enemyProbArray[i][1];
            }
            imgProbVals[y][x] = 1 - allFalseProduct;


            if(imgProbVals[y][x] > maxProbOfSeenVal)
                maxProbOfSeenVal = imgProbVals[y][x];
            if(imgProbVals[y][x] < minProbOfSeenVal)
                minProbOfSeenVal = imgProbVals[y][x];
        }
    }

    //std::cout << "Leaving nested for loops" << std::endl;
    writeImage(cost_map, ov, imgProbVals);
}


void Generator::writeImage(morrf_ros::int16_image &cost_map, commander::outputVals &ov, vector<vector<double> > imgProbVals) {

    double origRange = maxProbOfSeenVal - minProbOfSeenVal;
    double newRange = MAX_GRAYSCALE_VALUE - MIN_GRAYSCALE_VALUE;

    cost_map.name = "Stealthy Costmap Image";
    cost_map.width = width;
    cost_map.height = height;

    ov.name = "Stealth Text file";
   
	//now you have to fix the output vals file
    for(int y = 0; y < height; y++) {
        for(int x = 0; x < width; x++) {

            stringstream ss;
            ss << x << " " << y ;

            if(allObsPts.count(ss.str()) || enemyPtsToIgnore.count(ss.str())) {
                cost_map.int_array.push_back(255);
                continue;
            }
            imgProbVals[y][x] = (((imgProbVals[y][x] - minProbOfSeenVal) * newRange)/origRange) + MIN_GRAYSCALE_VALUE;

            cost_map.int_array.push_back(ceil(imgProbVals[y][x]));
           
	    commander::point_cost val;
 	    val.position.x = x;
	    val.position.y = y;
	    val.cost = ceil(imgProbVals[y][x]);
	    
	    ov.vals.push_back(val);		
	}
    }
}

void Generator::resize(vector<vector<double>> & array) {

    array.resize(height);
    for(int i = 0; i < height; i++) {
        array[i].resize(width);
    }
}

vector<double> Generator::setEnemyProbValues(bool blocked, double distance) {

    vector<double> arr;
    if(blocked) {
        arr.push_back(0);   //Probability of Seen
        arr.push_back(1);   //Prob of Unseen
    }
    else if(distance <= delta) {
        arr.push_back(1);
        arr.push_back(0);
    }
    else {
        double probability = (slope * distance) + y_intercept;
        arr.push_back(probability);
        arr.push_back(1 - probability);
    }
    return arr;
}

bool Generator::isBlocked(Point imgPt, Point enemyPt) {

    bool toReturn = false;
    for(Point obsPt : obsBoundaryPts) {
        if(isOnLineSegment(imgPt, enemyPt, obsPt)) {
            toReturn = true;
            break;
        }
    }
    return toReturn;
}

void Generator::getEnemyPtsToIgnore(list<Point> enemyPts) {

   for(Point p : enemyPts) {
       stringstream ss;
       ss << p.x << " " << p.y;
       enemyPtsToIgnore.insert(ss.str());
   }
}


void Generator::getAllObsPts(morrf_ros::int16_image img) {

    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {

          int index = i * width + j;

          int pixel_color  = img.int_array[index];

          if(pixel_color == 0) {

             //std::cout << "pixel is black: " << pixel_color << std::endl;

             stringstream ss;
             ss << j << " " << i;
             allObsPts.insert(ss.str());

          }
        }
    }
}

void Generator::getImgBoundaryPts(morrf_ros::int16_image img) {

    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {

            int index = i * width + j;
            int pixel_color  = img.int_array[index];
            if(pixel_color == 0) {
                obsBoundaryPts.push_back(Point(j,i));
            }
        }
    }
}

void Generator::clear() {

    obsBoundaryPts.clear();
    allObsPts.clear();
    enemyPtsToIgnore.clear();

    Generator();

}

void Generator::probOfBeingNearToObstacle(list<Point> enemyPts, morrf_ros::int16_image worldImg, morrf_ros::int16_image boundaryImg, morrf_ros::int16_image &cost_map, commander::outputVals &ov){

    // clear();

    // QImage worldBoundariesImage;
    // worldBoundariesImage.load(QString::fromStdString(world_boundaries));

    // QImage worldSolidsImage;
    // worldSolidsImage.load(QString::fromStdString(world_solids));

    // width = worldImg.width;
    // height = worldImg.height;

    // getImgBoundaryPts(worldImg);
    // getAllObsPts(worldSolidsImage);
    // getEnemyPtsToIgnore(enemyPts);

    // diag_distance = sqrt(pow(width, 2) + pow(height, 2));
    // double inv_diag_distance = 1 / diag_distance;

    // minProbOfNearness = 1;
    // maxProbOfNearness = inv_diag_distance;

    // vector<vector<double> > imgProbVals;
    // resize(imgProbVals);

    // for(int x = 0; x < width; x++) {
    //     for(int y = 0; y < height; y++) {

    //         stringstream ss;
    //         ss << x << " " << y;

    //         if(allObsPts.count(ss.str()) || enemyPtsToIgnore.count(ss.str()))
    //             continue;

    //         imgProbVals[y][x] = getNearObsValue(Point(x, y));

    //         if(imgProbVals[y][x] > maxProbOfNearness)
    //             maxProbOfNearness = imgProbVals[y][x];

    //         if(imgProbVals[y][x] < minProbOfNearness)
    //             minProbOfNearness = imgProbVals[y][x];
    //     }
    // }

    // writeSafeImage(imageFilename, outputFilename, imgProbVals);
}
//
double Generator::getNearObsValue(Point pt) {
//
//     double nearObsValue = 0;
//
//     for(Point obsPt : obsBoundaryPts) {
//
//         double distance = sqrt(pow(pt.x - obsPt.x, 2) + pow(pt.y - obsPt.y, 2));
//         double temp_inv_distance = 0;
//
//         if(distance <= delta)
//             temp_inv_distance = 1/delta;
//         else
//             temp_inv_distance = 1/distance;
//
//         if(temp_inv_distance > nearObsValue)
//             nearObsValue = temp_inv_distance;
//     }
//
//     return nearObsValue;
 }

void Generator::writeSafeImage(string imageFilename, string outputFilename, vector<vector<double> > imgProbVals) {

    //morrf_ros::int16_image im;
    //im.name = "safe cost map"
    //im.width = width;
    //im.height = height;
    //im.int_array = std::vector<int16_t> (img.width * img.height, 255);

    double origRange = maxProbOfNearness - minProbOfNearness;
    double newRange = MAX_GRAYSCALE_VALUE - MIN_GRAYSCALE_VALUE;

    QImage resultImage = QImage(width, height, QImage::Format_ARGB32);
    resultImage.fill(WHITE); //initialize the entire image with white

    ofstream out(outputFilename);

    for(int x = 0; x < width; x++) {
        for(int y = 0; y < height; y++) {

            stringstream ss;
            ss << x << " " << y;

            if(allObsPts.count(ss.str()) || enemyPtsToIgnore.count(ss.str())) {

                //cout << "The new range here is: " << (int) newRange << endl;
                QRgb value = qRgb((int)newRange, (int)newRange, (int)newRange);
                resultImage.setPixel(x, y, value);
                continue;
            }

            imgProbVals[y][x] = (((imgProbVals[y][x] - minProbOfNearness) * newRange)/origRange) + MIN_GRAYSCALE_VALUE;
            QRgb value = qRgb((int)imgProbVals[y][x], (int)imgProbVals[y][x], (int)imgProbVals[y][x]);
	    //im.int_array =
            resultImage.setPixel(x, y, value);

            out << ss.str() << "\t" << ceil(imgProbVals[y][x]);
            out << endl;
        }
    }

    out.close();
    resultImage.save(QString::fromStdString(imageFilename));
}
