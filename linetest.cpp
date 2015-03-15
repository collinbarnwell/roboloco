#include "raycast.hpp"

using namespace std;
using namespace pcl;


int main(int argc, char** argv) {

    PointXY erstart, erend, eestart, eeend, eye;
    vector<Line> lineList;

    // _____
    // ___
    // x

    erstart.x = 1;
    erstart.y = .5;

    erend.x = 0;
    erend.y = .5;

    Line occluder(erstart, erend);

    eestart.x = 0.0;
    eestart.y = 2.0;
    eeend.x = 2.0;
    eeend.y = 2.0;

    Line occludee(eestart, eeend);


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

    //    _____
    // ___
    //
    //    x

    lineList.clear();

    erstart.x = 1;
    erstart.y = 2;

    erend.x = 0;
    erend.y = 2;

    occluder.setStart(erstart);
    occluder.setEnd(erend);

    eestart.x = 0.0;
    eestart.y = 2.0;
    eeend.x = -2.0;
    eeend.y = 2.0;

    Line occludee2(eestart, eeend);

    eye.x = 0.0;
    eye.y = 0.0;


    TrimOcclusion(eye, occludee2, occluder, lineList);

    cout << "Occluder=> ";
    occluder.print();
    cout << "Occludee=> ";
    occludee2.print();

    cout << "Linelist size: " << lineList.size() << endl;

    if (lineList.size() > 0) {
        cout << "First line in lineList=> ";
        lineList.at(0).print();
    }

    cout << endl;

    return 0;
}