#ifndef RAYCAST
#define RAYCAST

#include "lines.hpp"
#include <algorithm>

using namespace std;
using namespace pcl;


struct PointSorter {
    PointSorter(PointXY point) {
        p = point;
    }
    bool operator () (PointXY i, PointXY j) {
        float disti = (p.x-i.x)*(p.x-i.x) + (p.y-i.y)*(p.y-i.y);
        float distj = (p.x-j.x)*(p.x-j.x) + (p.y-j.y)*(p.y-j.y);
        return (disti > distj);
    }
    PointXY p;
};


void TrimOcclusion(PointXY x, Line &occludee, Line occluder, vector<Line> &lineList) {
    if (occluder.isZero()) return;

    vector<PointXY> pois;
    PointXY intersection, poi1, poi2, ignore;

    Line to_occludee_start(x, occludee.getStart());
    Line to_occludee_end(x, occludee.getEnd());
    Line to_occluder_start(x, occluder.getStart());
    Line to_occluder_end(x, occluder.getEnd());

    if (to_occludee_start.intersect(occluder, &ignore) && to_occludee_end.intersect(occluder, &ignore)) {
        // line is totally occluded
        occludee.set_to_zero();
        return;
    }

    if (occludee.intersect(occluder, &intersection)) {
        pois.push_back(intersection);
    }
    if (to_occluder_start.intersectOutOfBound(occludee, &poi1)) {
        pois.push_back(poi1);
    }
    if (to_occluder_end.intersectOutOfBound(occludee, &poi2)) {
        pois.push_back(poi2);
    }

    // if at least one endpoint is clear ^^
    // and there are no pois ^
    // occludee is not occluded
    if (pois.size() < 1) {
        return;
    }

    // 1) find unoccluded endpt, keep until closest poi/endpt 
    // 2) throw away until following poi/endpt
    // 3) repeat...
    if (to_occludee_start.intersect(occluder, &ignore)) {
        // start at end
        pois.push_back(occludee.getStart());
        PointSorter psort(occludee.getEnd());
        sort(pois.begin(), pois.end(), psort);

        occludee.setStart(pois.back());
        pois.pop_back();
    } else {
        // start at start
        pois.push_back(occludee.getEnd());
        PointSorter psort(occludee.getStart());
        sort(pois.begin(), pois.end(), psort);

        occludee.setEnd(pois.back());
        pois.pop_back();
    }

    // add possible disconnected segment to lineList
    if (pois.size() > 1) {
        PointXY new_line_start = pois.back();
        pois.pop_back();
        Line new_line(new_line_start, pois.back());
        lineList.push_back(new_line);
    }
    
    return;
}


void AnalyticRayCast(vector<Line> input, vector<Line> &output) {






    return;
}


#endif
