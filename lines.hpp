#ifndef LINES_N_STUFF
#define LINES_N_STUFF

#define INF 16384

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
        float lengthSqd();
        void checkLength();
        bool angleAboveMax(PointXY p, float max);
    private:
        PointXY start;
        PointXY end;
        bool is_zero;
};

Line::Line(PointXY s, PointXY e) {
    start = s;
    end = e;
    is_zero = false;
    checkLength();
}

void Line::checkLength() {
    if (lengthSqd() <= .01*.01) {
        is_zero = true;
    }
}

bool Line::angleAboveMax(PointXY p, float max) {
    // calculate angle between (0, 0) to p (<PointXY>) and NORMAL of self (Line) IN RADIANS
    float lx = start.x - end.x;
    float ly = start.y - end.y;

    // magnitudes
    float ll = sqrt(lx*lx + ly*ly);
    float lp = sqrt(p.x*p.x + p.y*p.y);

    // dotp of normalized vectors
    float dotp = (lx*p.x + ly * p.y)/(lp*ll);
    
    // range of ang is [0, Pi]
    float ang = acos(dotp);

    // angle should be between Pi/4 + max and Pi/4 - max
    if ((ang <= PI/4 + max) && (ang >= PI/4 - max)) {
        return true;
    }
    else {
        return false;
    }
}

float Line::lengthSqd() {
    return (start.x-end.x)*(start.x-end.x) + (start.y-end.y)*(start.y-end.y);
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
    checkLength();
}

PointXY Line::getEnd() {
    return end;
}

void Line::setEnd(PointXY e) {
    end = e;
    checkLength();
}

bool Line::isZero() {
    return is_zero;
}

void Line::set_to_zero() {
    is_zero = true;
}

bool Line::intersect(Line l, PointXY* intersection) {
    if (l.isZero() || isZero()) return false;

    float slopeme, slopel;

    if ((l.getEnd().x - l.getStart().x) == 0) {
        slopel = (l.getEnd().y - l.getStart().y)*INF;
    } else {
        slopel = (l.getEnd().y - l.getStart().y)/(l.getEnd().x - l.getStart().x);
    }

    if ((end.x - start.x) == 0) {
        slopeme = (end.y - start.y)*INF;
    } else {
        slopeme = (end.y - start.y)/(end.x - start.x);
    }
    
    float x = (-slopel*l.getStart().x + l.getStart().y - start.y + start.x * slopeme)/(slopeme - slopel);

    bool within_l = ((x >= l.getStart().x && x <= l.getEnd().x) || (x <= l.getStart().x && x >= l.getEnd().x));
    bool within_self = (x >= start.x && x <= end.x) || (x <= start.x && x >= end.x);

    if (within_self && within_l)
    {
        PointXY p;
        p.x = x;
        p.y = slopeme*(x - start.x) + start.y;
        *intersection = p;

        // cout << "INTERSECTION DETECTED (self, l): " << endl;
        // print();
        // l.print();
        // cout << "at: " << intersection->x << "," << intersection->y << endl << endl;

        return true;
    }
    else {
        return false;
    }
}

// test if intersects with l, even if intersection is outside of self
bool Line::intersectOutOfBound(Line l, PointXY* intersection) {
    if (l.isZero() || isZero()) return false;

    float slopeme, slopel;

    if ((l.getEnd().x - l.getStart().x) == 0) {
        slopel = (l.getEnd().y - l.getStart().y)*INF;
    } else {
        slopel = (l.getEnd().y - l.getStart().y)/(l.getEnd().x - l.getStart().x);
    }

    if ((end.x - start.x) == 0) {
        slopeme = (end.y - start.y)*INF;
    } else {
        slopeme = (end.y - start.y)/(end.x - start.x);
    }

    float x = (-slopel*l.getStart().x + l.getStart().y - start.y + start.x * slopeme)/(slopeme - slopel);
    
    bool within_l = ((x >= l.getStart().x && x <= l.getEnd().x) || (x <= l.getStart().x && x >= l.getEnd().x));
    within_l = within_l || veryCloseTo(x, l.getStart().x) || veryCloseTo(x, l.getEnd().x);

    bool not_within_self = (x >= start.x && x >= end.x) || (x <= start.x && x <= end.x);
    not_within_self = not_within_self || veryCloseTo(x, end.x) || veryCloseTo(x, start.x);

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
