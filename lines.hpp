#ifndef LINES_N_STUFF
#define LINES_N_STUFF

#include "basic_utilities.hpp"

using namespace std;
using namespace pcl;

class Line {
    public:
        Line(PointXY s, PointXY e);
        bool intersect(Line l, PointXY* intersection);
        bool intersectOutOfBound(Line l, PointXY* intersection);
        PointXY getStart();
        PointXY getEnd();
        void setStart(PointXY s);
        void setEnd(PointXY e);
        bool isZero();
        void set_to_zero();
        void print();
    private:
        PointXY start;
        PointXY end;
        bool is_zero;
};

Line::Line(PointXY s, PointXY e) {
    start = s;
    end = e;
    is_zero = false;
}

void Line::print() {
    cout << "start: " << start.x << "," << start.y 
        << "  end: " << end.x << "," << end.y 
        << "is_zero: " << is_zero << endl;
}

PointXY Line::getStart() {
    return start;
}

void Line::setStart(PointXY s) {
    start = s;
}

PointXY Line::getEnd() {
    return end;
}

void Line::setEnd(PointXY e) {
    end = e;
}

bool Line::isZero() {
    return is_zero;
}

void Line::set_to_zero() {
    is_zero = true;
}

bool Line::intersect(Line l, PointXY* intersection) {
    float slopel = (l.getEnd().y - l.getStart().y)/(l.getEnd().x - l.getStart().x);
    float slopeme = (l.getEnd().y - l.getStart().y)/(l.getEnd().x - l.getStart().x);
    
    float x = (-slopel*l.getStart().x + l.getStart().y - start.y + start.x * slopeme)/(slopeme - slopel);
    
    if (x > start.x && x < end.x && x > l.getStart().x && x < l.getEnd().x) 
    {
        PointXY p;
        p.x = x;
        p.y = slopeme*(x - start.x) + start.y;
        *intersection = p;
        return true;
    }
    else {
        return false;
    }
}

bool Line::intersectOutOfBound(Line l, PointXY* intersection) {
    float slopel = (l.getEnd().y - l.getStart().y)/(l.getEnd().x - l.getStart().x);
    float slopeme = (l.getEnd().y - l.getStart().y)/(l.getEnd().x - l.getStart().x);
    
    float x = (-slopel*l.getStart().x + l.getStart().y - start.y + start.x * slopeme)/(slopeme - slopel);
    
    if (x > start.x && x < end.x) 
    {
        PointXY p;
        p.x = x;
        p.y = slopeme*(x - start.x) + start.y;
        *intersection = p;
        return true;
    }
    else {
        return false;
    }
}



//******************
//******************
//******************

// class LineList {
//     public:
//         void addLine(Line l);
//         void removeLine(Line l);
//         void mergeList(LineList l);
//     private:
//         vector<Line> list;
// };


#endif