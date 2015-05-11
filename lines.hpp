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

bool Line::intersect(Line l, PointXY &intersection) 
{
    PointXYZ s, r, pq;
    s.x = l.getEnd().x - l.getStart().x;
    s.y = l.getEnd().y - l.getStart().y;
    s.z = 0.0;

    r.x = end.x - start.x;
    r.y = end.y - start.y;
    r.z = 0.0;

    // t = (l.getStart() − start) × s / (r × s)
    // u = (l.getStart() − start) × r / (r × s)
    pq.x = start.x - l.getStart().x;
    pq.y = start.y - l.getStart().y;
    pq.z = 0.0;

    PointXYZ rxs = crossp( r, s );
    float t = crossp( pq, s ).z / rxs.z;
    float u = crossp( pq, r ).z / rxs.z;

    // behind start or within segment || outside of s
    if ( t > 1.0 || t < 0.0 || u > 1.0 || u < 0.0 )
        return false;

    intersection.x = start.x + t*r.x;
    intersection.y = start.y + t*r.y;

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
    PointXYZ s, r, pq;
    s.x = l.getEnd().x - l.getStart().x;
    s.y = l.getEnd().y - l.getStart().y;
    s.z = 0.0;

    r.x = end.x - start.x;
    r.y = end.y - start.y;
    r.z = 0.0;

    // t = (l.getStart() − start) × s / (r × s)
    // u = (l.getStart() − start) × r / (r × s)
    pq.x = start.x - l.getStart().x;
    pq.y = start.y - l.getStart().y;
    pq.z = 0.0;

    PointXYZ rxs = crossp( r, s );
    float t = crossp( pq, s ).z / rxs.z;
    float u = crossp( pq, r ).z / rxs.z;

    // behind start or within segment || outside of s
    if ( t <= 1.0 || u >= 1.0 || u <= 0.0 )
        return false;

    intersection.x = start.x + t*r.x;
    intersection.y = start.y + t*r.y;

    return true;
}

void svgPrint(vector<Line> lines, int namenum, PointXY p)
{
    p.x = 1.555;
    p.y = 3.851;

    srand (time(NULL));
    int imgsize = 800;
    float k = float(imgsize)/12.5;

    stringstream oss;
    oss << namenum << "plot.html";
    string filename = oss.str();

    ofstream f;
    f.open(filename.c_str());

    f << "<!DOCTYPE html><html><body><svg version=\"1.1\""
        << "style=\"background: black\" "
        << "baseProfile=\"full\" width=\"" 
        << imgsize << "\" height=\"" 
        << imgsize << "\" xmlns=\"http://www.w3.org/2000/svg\">\n\n";

    f << "<circle cx=\"" << int(p.x * k) << "\" cy=\"" << 800 - int(p.y * k)
        << "\" r=\"5\" fill=\"green\" />\n\n";

    int linesize = lines.size();
    for (int i = 0; i < linesize; i++)
    {

        int x1 = lines[i].getStart().x * k;
        int x2 = lines[i].getEnd().x * k;
        int y1 = 800 - (lines[i].getStart().y + 1.5) * k;
        int y2 = 800 - (lines[i].getEnd().y + 1.5) * k;

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
