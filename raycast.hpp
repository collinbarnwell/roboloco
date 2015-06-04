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

    if (to_occludee_start.intersect(occluder, ignore) && to_occludee_end.intersect(occluder, ignore)) {
        // line is totally occluded`
        occludee.set_to_zero();
        return;
    }

    // if (occludee.intersect(occluder, intersection)) {
    //     pois.push_back(intersection);
    //     cout << "should not happen" << endl;
    //     throw 1;
    // }
    if (to_occluder_start.intersectOutOfBound(occludee, poi1)) {
        pois.push_back(poi1);
    }
    if (to_occluder_end.intersectOutOfBound(occludee, poi2)) {
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

    if (to_occludee_start.intersect(occluder, ignore)) 
    // start is occluded
    { 
        // start at end
        PointSorter psort(occludee.getEnd());
        sort(pois.begin(), pois.end(), psort);

        // for (int i = 0; i < pois.size(); i++) {
        //     cout << pois[i].x << ", " << pois[i].y << endl;
        // }

        occludee.setStart(pois.back());
        pois.pop_back();
    } 
    else 
    // start is not occluded
    {
        // add end to pois if also not occluded
        if (!to_occludee_end.intersect(occluder, ignore)) {
            pois.push_back(occludee.getEnd());
        }

        // start at start
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


vector<Line> AnalyticRayCast(PointXY x, vector<Line> map) {

    int map_size = map.size();

    vector<Line> L;

    for (int i = 0; i < map_size; i++)
    {
        Line currentLine = map[i];

        if (currentLine.isZero()) { continue; }

        for (int j = 0; j < map_size; j++)
        {
            if (j == i) { continue; }

            TrimOcclusion(x, currentLine, map[j], map);
        }

        if (currentLine.isZero()) { continue; }

        int lsize = L.size();
        for (int j = 0; j < lsize; j++) {
            TrimOcclusion(x, L[j], currentLine, map);
        }

        L.push_back(currentLine);
    }

    return L;
}


#endif
