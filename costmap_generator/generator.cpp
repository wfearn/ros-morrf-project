#include "generator.h"

Generator::Generator()
{

}

bool Generator::isOnLineSegment(Point a, Point b, Point c){

  double AB, AC, CB;
  AB = AC = CB = 0.0;

  AB = sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
  AC = sqrt((c.x - a.x) * (c.x - a.x) + (c.y - a.y) * (c.y - a.y));
  CB = sqrt((b.x - c.x) * (b.x - c.x) + (b.y - c.y) * (b.y - c.y));

  double sumWithR = AC + CB;

  if(abs((int) (sumWithR - AB)) < 0.01)
    return true;
  return false;
}

void Generator::probOfSeenByEnemy(double d, list<Point> enemyPts, string world_boundaries, string world_solids, string imageFile, string outputFile){

    QImage worldBoundariesImage;
    worldBoundariesImage.load(QString::fromStdString(world_boundaries));

    width = worldBoundariesImage.width(); //width of the world_boundaries image
    height = worldBoundariesImage.height(); //height of the world_boundaries image

    delta = d;
    diag_distance = sqrt(pow(width, 2) + pow(height, 2));
    slope = 1/(delta - diag_distance);
    y_intercept = (-1) * (diag_distance * slope);

    //getting all the points
    getImgBoundaryPts(worldBoundariesImage);

    QImage worldSolidsImage;
    worldSolidsImage.load(QString::fromStdString(world_solids));
    getAllObsPts(worldSolidsImage);

    getEnemyPtsToIgnore(enemyPts);

    minProbOfSeenVal = 2.0;
    maxProbOfSeenVal = 0.0;

    vector<vector<double> > imgProbVals;
    resize(imgProbVals);
    int numOfEnemies = enemyPts.size();

//    qDebug() << "width: " << width << endl;
    for(int x = 0; x < height; x++) {
        for(int y = 0; y < width; y++) {

            string s = to_string(x) + " " + to_string(y);
            if(allObsPts.count(s) || enemyPtsToIgnore.count(s))
                continue;

            double enemyProbArray[numOfEnemies][2];
            int i = 0;
            for(Point enemyPt : enemyPts) {

//                cout << "(" << x << ", " << y << ") at: " << i << endl;
                bool blocked = isBlocked(Point(x, y), enemyPt);
                double distanceToEnemy = sqrt((x - enemyPt.x) * (x - enemyPt.x) + (y - enemyPt.y) * (y - enemyPt.y));

                vector<double> probability = setEnemyProbArray(i, blocked, distanceToEnemy);
                enemyProbArray[i][0] = probability.at(0);
                enemyProbArray[i][1] = probability.at(1);
                i++;
            }

            double allFalseProduct = 1;
            for(int i = 0; i < numOfEnemies; i++) {
                allFalseProduct *= enemyProbArray[i][1];
            }
            imgProbVals[x][y] = 1 - allFalseProduct;
//            cout << imgProbVals[x][y] << endl;

            if(imgProbVals[x][y] > maxProbOfSeenVal)
                maxProbOfSeenVal = imgProbVals[x][y];
            if(imgProbVals[x][y] < minProbOfSeenVal)
                minProbOfSeenVal = imgProbVals[x][y];
        }
    }

//    qDebug() << imgProbVals.size() << endl;

    writeImage(outputFile, imageFile, imgProbVals);
}


void Generator::writeImage(string outputFile, string image, vector<vector<double>> imgProbVals) {

    cout << "Writing Image!" << endl;
    double origRange = maxProbOfSeenVal - minProbOfSeenVal;
    double newRange = MAX_GRAYSCALE_VALUE - MIN_GRAYSCALE_VALUE;

//    double ratio = newRange/origRange;

    QImage resultImage = QImage(width, height, QImage::Format_ARGB32);
    resultImage.fill(BLACK); //initialize the entire image with white

    ofstream out(outputFile);
    if(out.is_open())
        cout << "Opened output file!" << endl;
    else
        cout << "output file cannot be opened" << endl;

    for(int x = 0; x < height; x++) {
        for(int y = 0; y < width; y++) {

            stringstream ss;
            ss << x << " " << y;

            if(allObsPts.count(ss.str()) || enemyPtsToIgnore.count(ss.str()))
                continue;
            imgProbVals[x][y] = (((imgProbVals[x][y] - minProbOfSeenVal) * newRange)/origRange) + MIN_GRAYSCALE_VALUE;

            if(imgProbVals[x][y] < 255) {
                resultImage.setPixel(y, x, (int)imgProbVals[x][y]);

                out << ss.str() << "\t" << ceil(imgProbVals[x][y]);
                out << endl;
            }
        }
    }

    out.close();
    resultImage.save(QString::fromStdString(image));
}

void Generator::resize(vector<vector<double>> & array) {

    array.resize(height);
    for(int i = 0; i < height; i++) {
        array[i].resize(width);
    }
}

vector<double> Generator::setEnemyProbArray(int i, bool blocked, double distance) {

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


void Generator::getAllObsPts(QImage image) {

//    cout << "Getting all Obs Pts" << endl;
//    int width = image.width();
//    int height = image.height();

    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {

//            cout << "i,j: " << i << ", " << j << endl;
            QColor pixel_color = image.pixel(j, i);
            if(pixel_color == BLACK) {

                stringstream ss;
               ss << i << " " << j;
               allObsPts.insert(ss.str());

            }
        }
    }
}

void Generator::getImgBoundaryPts(QImage image) {

//    int width = image.width();
//    int height = image.height();

    for(int i = 0; i < height; i++) {
        for(int j = 0; j < width; j++) {

            QColor pixel_color = image.pixel(j, i);
            if(pixel_color == BLACK) {
                obsBoundaryPts.push_back(Point(i,j));
            }
        }
    }
}

