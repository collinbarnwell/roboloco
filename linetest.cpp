#include "raycast.hpp"

using namespace std;
using namespace pcl;


int main(int argc, char** argv) {

    PointXY erstart, erend, eestart, eeend, eye;
    vector<Line> lineList;

    erstart.x = 2;
    erstart.y = .5;
    erend.x = 3;
    erend.y = .5;

    Line occluder(erstart, erend);

    eestart.x = 6.0;
    eestart.y = 2.0;
    eeend.x = 0.0;
    eeend.y = 2.0;

    Line occludee(eestart, eeend);

    eye.x = 3.0;
    eye.y = 0.0;

    lineList.push_back(occludee);
    lineList.push_back(occluder);
    svgPrint(lineList, -5, eye);
    lineList.clear();

    TrimOcclusion(eye, occludee, occluder, lineList);

    lineList.push_back(occludee);
    lineList.push_back(occluder);

    svgPrint(lineList, -4, eye);

    ///////////////////////////////////////

    // lineList.clear();

    // erstart.x = 1;
    // erstart.y = 2;

    // erend.x = 0;
    // erend.y = 2;

    // occluder.setStart(erstart);
    // occluder.setEnd(erend);

    // eestart.x = 2.0;
    // eestart.y = 2.0;
    // eeend.x = 0.0;
    // eeend.y = 2.0;

    // Line occludee2(eestart, eeend);

    // eye.x = 2.0;
    // eye.y = 0.0;

    // lineList.push_back(occludee);
    // lineList.push_back(occluder);
    // svgPrint(lineList, -2, eye);
    // lineList.clear();

    // TrimOcclusion(eye, occludee2, occluder, lineList);

    // lineList.push_back(occludee);
    // lineList.push_back(occluder);

    // svgPrint(lineList, -3, eye);

    return 0;
}