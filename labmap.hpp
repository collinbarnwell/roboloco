#include "lines.hpp"

using namespace std;
using namespace pcl;


Line createLine(float x1, float y1, float x2, float y2) {
    PointXY s, e;

    s.x = x1;
    s.y = y1;
    e.x = x2;
    e.y = y2;

    return Line(s, e);
}


vector<Line> makeMap() {
    vector<Line> map;

    // Origin at bathroom-most corner of room

    // Walls + stuff connected to walls
    map.push_back(createLine(0, .76, 0, 7.6));
    // map.push_back(createLine(0, 5.5, .76, 5.5));
    // map.push_back(createLine(.76, 5.5, .76, 7.6));
    map.push_back(createLine(0.0, 7.6, 10.2, 7.6)); // top wall
    map.push_back(createLine(10.2, 7.6, 10.2, -1.5));
    map.push_back(createLine(10.2, -1.5, 7.5, -1.5));
    map.push_back(createLine(7.5, -1.5, 7.5, 0.0));
    map.push_back(createLine(0, .76, 3.8, .76));
    map.push_back(createLine(3.8, .76, 3.8, 0));
    map.push_back(createLine(3.8, 0, 7.5, 0));

    // Left Desk
    map.push_back(createLine(2.3, 5.5, 3.8, 5.5));
    map.push_back(createLine(2.3, 5.5, 2.3, 7));
    map.push_back(createLine(2.3, 7, 3.8, 7));
    map.push_back(createLine(3.8, 7, 3.8, 5.5));

    // Rigt Desk
    map.push_back(createLine(5.3, 5.5, 6.8, 5.5));
    map.push_back(createLine(5.3, 5.5, 5.3, 7));
    map.push_back(createLine(5.3, 7, 6.8, 7));
    map.push_back(createLine(6.8, 7, 6.8, 5.5));

    // Left shelf
    map.push_back(createLine(.9, 4.9, 3, 4.9));
    map.push_back(createLine(.9, 4.6, 3, 4.6));
    map.push_back(createLine(.9, 4.6, .9, 4.9));
    map.push_back(createLine(3, 4.9, 3, 4.6));
    
    // Left shelf
    map.push_back(createLine(4, 4.9, 6.1, 4.9));
    map.push_back(createLine(4, 4.6, 6.1, 4.6));
    map.push_back(createLine(4, 4.6, 4, 4.9));
    map.push_back(createLine(6.1, 4.9, 6.1, 4.6));


    return map;
}
