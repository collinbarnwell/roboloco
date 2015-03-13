#include "raycast.hpp"

using namespace std;
using namespace pcl;


int main(int argc, char** argv) {

    PointXY erstart, erend, eestart, eeend, eye;

    erstart.x = .25;
    erstart.y = .5;

    erend.x = -.25;
    erend.y = 2.5;

    Line occluder(erstart, erend);

    eestart.x = -1.0;
    eestart.y = 2.0;
    eeend.x = 1.0;
    eeend.y = 2.0;

    Line occludee(eestart, eeend);

    vector<Line> lineList;

    eye.x = 0.0;
    eye.y = 0.0;


    TrimOcclusion(eye, occludee, occluder, lineList);

    cout << "Occluder=> ";
    occluder.print();
    cout << "Occludee=> ";
    occludee.print();

    cout << "Linelist size: " << lineList.size() << endl;

    if (lineList.size() > 0) {
        cout << "First line in lineList=> ";
        lineList.at(0).print();
    }

    cout << endl;

    return 0;
}