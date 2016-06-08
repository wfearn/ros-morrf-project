#include <QCoreApplication>
#include "generator.h"
#include <ctime>

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    Generator generator;
    double delta = 5;

    uint start = clock();

    string objValsFName = "/home/tkatuka/Documents/Qt Creator/CostmapGenerator/outputVal.txt";
    string resultImageFName = "/home/tkatuka/Documents/Qt Creator/CostmapGenerator/image.png";

    string obsBoundaryImg = "/home/tkatuka/Documents/Qt Creator/CostmapGenerator/world_boundaries.png";
    string obsSolidImg = "/home/tkatuka/Documents/Qt Creator/CostmapGenerator/world_solids.png";
//    ifstream in(obsBoundaryImg);
//    cout << in.is_open() << endl;

    list<Point> enemyPts;
    Point enemy1(115, 100);
    Point enemy2(380, 255);

    enemyPts.push_back(enemy1);
    enemyPts.push_back(enemy2);

    cout << "It started." << endl;
    generator.probOfSeenByEnemy(delta, enemyPts, obsBoundaryImg, obsSolidImg, resultImageFName, objValsFName);

    uint finish = clock();
    cout << "Time elapsed in milliseconds: " << finish - start << endl;
    return a.exec();
}
