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
        << "  is_zero: " << is_zero << endl;
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
    float slopeme = (end.y - start.y)/(end.x - start.x);
    
    float x = (-slopel*l.getStart().x + l.getStart().y - start.y + start.x * slopeme)/(slopeme - slopel);
    
    bool within_l = ((x > l.getStart().x && x < l.getEnd().x) || (x < l.getStart().x && x > l.getEnd().x));
    bool within_self = ((x > start.x && x < end.x) || (x < start.x && x > end.x));

    if (within_self && within_l)
    {
        PointXY p;
        p.x = x;
        p.y = slopeme*(x - start.x) + start.y;
        *intersection = p;

        cout << "INTERSECTION DETECTED (self, l): " << endl;
        print();
        l.print();
        cout << "at: " << intersection->x << "," << intersection->y << endl << endl;

        return true;
    }
    else {
        return false;
    }
}

// test if intersects with l, even if intersection is outside of self
bool Line::intersectOutOfBound(Line l, PointXY* intersection) {
    float slopel = (l.getEnd().y - l.getStart().y)/(l.getEnd().x - l.getStart().x);
    float slopeme = (end.y - start.y)/(end.x - start.x);
    
    float x = (-slopel*l.getStart().x + l.getStart().y - start.y + start.x * slopeme)/(slopeme - slopel);
    
    bool within_l = ((x > l.getStart().x && x < l.getEnd().x) || (x < l.getStart().x && x > l.getEnd().x));
    bool not_within_self = !((x > start.x && x < end.x) || (x < start.x && x > end.x));

    if (within_l && not_within_self)
    {
        PointXY p;
        p.x = x;
        p.y = slopeme*(x - start.x) + start.y;
        *intersection = p;

        cout << "INTERSECTION (out of bound) DETECTED (self, l): " << endl;
        print();
        l.print();
        cout << "at: " << intersection->x << "," << intersection->y << endl << endl;

        return true;
    }
    else {
        return false;
    }
}


#endif
