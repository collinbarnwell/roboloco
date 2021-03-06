#ifndef LINES_N_STUFF
#define LINES_N_STUFF

#include "basic_utilities.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;
using namespace pcl;


class Line {
    public:
        Line(PointXY s, PointXY e);
        bool intersect(Line l, PointXY &intersection);
        bool intersectOutOfBound(Line l, PointXY &intersection);
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
        friend bool operator==(Line l1, Line l2);
    private:
        PointXY start;
        PointXY end;
        bool is_zero;
};

bool operator==(Line l1, Line l2) {
    if ((l1.start == l2.start && l1.end == l2.end) ||
        (l1.start == l2.end && l1.end == l2.start))
        return true;
    return false;
}

Line::Line(PointXY s, PointXY e) {
    start = s;
    end = e;
    is_zero = false;
    checkLength();
}

void Line::checkLength() {
    // check to make sure not too small
    // if (lengthSqd() <= .01*.01) {
    //     is_zero = true;
    // }

    // // check to make sure its not vertical
    if (end.x != end.x || start.x != start.x 
        || end.y != end.y || start.y != start.y) {
        cout << "dis shit be nan!!" << endl;
        throw 1;
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

    if (dotp != dotp) {
        cout << "dotp is NaN in angleAboveMax" << endl;
        throw 1;
    }
    
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

bool Line::intersect(Line l, PointXY &intersection) 
{
    // p is self start
    // q is l start
    // s is q end - q start
    // r is p end - p start
    PointXY s, r, qminusp;
    s.x = l.getEnd().x - l.getStart().x;
    s.y = l.getEnd().y - l.getStart().y;

    r.x = end.x - start.x;
    r.y = end.y - start.y;

    // t = (l.getStart() − start) × s / (r × s)
    // u = (l.getStart() − start) × r / (r × s)
    qminusp.x = l.getStart().x - start.x;
    qminusp.y = l.getStart().y - start.y;

    float rxs = crossp( r, s );

    if (abs(rxs) < 1/INFINITY) {
        // If r × s = 0 and (q − p) × r = 0, then the two lines are collinear.)
        // OR
        // If r × s = 0 and (q − p) × r ≠ 0, 
        // then the two lines are parallel and non-intersecting.
        return false;
    }

    float t = crossp( qminusp, s ) / rxs;
    float u = crossp( qminusp, r ) / rxs;

    // behind start or within segment || outside of s
    if ( t > 1.0 || t < 0.0 || u > 1.0 || u < 0.0 ) {
        return false;
    }

    // if ( veryCloseTo(t, 1.0) || veryCloseTo(t, 0.0) ||
    //     veryCloseTo(u, 0.0) || veryCloseTo(u, 1.0) )
    //     return false;

    intersection.x = start.x + t*r.x;
    intersection.y = start.y + t*r.y;

    if (u != u || t != t)
        return false;

    if (intersection.x != intersection.x || intersection.y != intersection.y) {
        cout << "intersection be nan!" << endl;
        throw 1;
    }

    return true;
}


// help from: http://stackoverflow.com/questions/14307158
// /how-do-you-check-for-intersection-between-a-line-segment-and-a-line-ray-emanatin
// and 
// http://stackoverflow.com/questions/563198
// /how-do-you-detect-where-two-line-segments-intersect/565282#565282
bool Line::intersectOutOfBound(Line l, PointXY &intersection) 
// Tests if intersects with l, even if intersection is outside of self
{
    // want to find t & u for: (p)start + t*<r> = (q)l.getStart() + u*<s>
    PointXY s, r, qminusp;
    s.x = l.getEnd().x - l.getStart().x;
    s.y = l.getEnd().y - l.getStart().y;

    r.x = end.x - start.x;
    r.y = end.y - start.y;

    // t = (l.getStart() − start) × s / (r × s)
    // u = (l.getStart() − start) × r / (r × s)
    qminusp.x = l.getStart().x - start.x;
    qminusp.y = l.getStart().y - start.y;

    float rxs = crossp( r, s );

    if (abs(rxs) < 1/INFINITY) {
        // If r × s = 0 and (q − p) × r = 0, then the two lines are collinear.)
        // OR
        // If r × s = 0 and (q − p) × r ≠ 0, 
        // then the two lines are parallel and non-intersecting.
        return false;
    }

    float t = crossp( qminusp, s ) / rxs;
    float u = crossp( qminusp, r ) / rxs;

    // behind start or not within segment || outside of s

    if ( t <= 1.0 || u >= 1.0 || u <= 0.0 ) {
        return false;
    }

    // if ( veryCloseTo(t, 1.0) || veryCloseTo(u, 1.0) || veryCloseTo(u, 0.0) )
    //     return false;

    if (u != u || t != t) {
        // cout << "NAN" << endl;
        return false;
    }

    intersection.x = start.x + t*r.x;
    intersection.y = start.y + t*r.y;

    if (intersection.x != intersection.x || intersection.y != intersection.y) {
        cout << "intersection be nan!" << endl;
        throw 1;
    }

    return true;
}

void svgPrint(vector<Line> lines, int namenum, PointXY p)
{
    srand (time(NULL));
    int imgsize = 800;
    float k = float(imgsize)/12.5;

    stringstream oss;
    oss << "../plots/" << namenum << "plot.html";
    string filename = oss.str();

    ofstream f;
    f.open(filename.c_str());

    f << "<!DOCTYPE html><html><body><svg version=\"1.1\""
        << "style=\"background: black\" "
        << "baseProfile=\"full\" width=\"" 
        << imgsize << "\" height=\"" 
        << imgsize << "\" xmlns=\"http://www.w3.org/2000/svg\">\n\n";

    f << "<circle cx=\"" << int(5 + p.x * k) << "\" cy=\"" << 795 - int((p.y+1.5) * k)
        << "\" r=\"5\" fill=\"green\" />\n\n";

    int linesize = lines.size();
    for (int i = 0; i < linesize; i++)
    {
        if (lines[i].isZero())
            continue;

        int x1 = 5 + lines[i].getStart().x * k;
        int x2 = 5 + lines[i].getEnd().x * k;
        int y1 = 795 - (lines[i].getStart().y + 1.5) * k;
        int y2 = 795 - (lines[i].getEnd().y + 1.5) * k;

        int r = rand()%175;
        int g = rand()%175;
        int b = rand()%175;

        f << "<line x1=\"" << x1 << "\" y1=\"" << y1
            << "\" x2=\"" << x2 << "\" y2=\"" << y2
            << "\" style=\"stroke:rgb(" 
            << r << "," << g << "," << b 
            << ");stroke-width:3\" />\n";
    }

    f << "\n</svg></body></html>";

    f.close();
}


#endif
