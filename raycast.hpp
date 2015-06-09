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

void addLinesICanSee(PointXY x, vector<Line> &L, vector<Line> lines)
{
        for (int i = 0; i != lines.size(); i++)
        {
            Line l = lines[i];

            // find point half way between current and next
            PointXY midpoint, ignore;
            midpoint.x = (l.getStart().x + l.getEnd().x)/2.0;
            midpoint.y = (l.getStart().y + l.getEnd().y)/2.0;
            Line to_l = Line(x, midpoint);

            bool keep = true;
            for (int j = 0; j != L.size(); j++) 
            {
                PointXY ignore;
                if ( to_l.intersect(L[j], ignore) ) {
                    keep = false;
                    break;
                }
            }
            if (keep)
                L.push_back(l);
        }
        
        return;
}


void TrimOcclusion(PointXY x, Line &occludee, Line occluder, vector<Line> &lineList) {
    if (occludee.getStart().x == 0.0 && occludee.getStart().y == 0.76 
        && occludee.getEnd().x == 0.0 && occludee.getEnd().y == 5.5)
        cout << "trim occlusion is called !" << endl;

    if (occluder.isZero() || occludee.isZero())
        return;

    vector<PointXY> pois;
    PointXY poi1, poi2;

    Line to_occluder_start(x, occluder.getStart());
    Line to_occluder_end(x, occluder.getEnd());

    if (to_occluder_start.intersectOutOfBound(occludee, poi1))
        pois.push_back(poi1);

    if (to_occluder_end.intersectOutOfBound(occludee, poi2))
        pois.push_back(poi2);

    pois.push_back(occludee.getStart());
    pois.push_back(occludee.getEnd());

    PointSorter psort(occludee.getEnd());
    sort(pois.begin(), pois.end(), psort);

    PointXY current = pois.back();
    pois.pop_back();
    PointXY next = pois.back();
    pois.pop_back();

    // Loop through pois-separated segments
    bool first_keeper = true;
    while (true) {
        bool last_iteration = false;
        if (pois.size() == 0)
            last_iteration = true;

        // find point half way between current and next
        PointXY midpoint, ignore;
        midpoint.x = (current.x + next.x)/2.0;
        midpoint.y = (current.y + next.y)/2.0;
        Line l = Line(x, midpoint);

        if (first_keeper)
        {
            // check for occluder intersection in line to midpoint of occludee section
            if (l.intersect(occluder, ignore)) {
                if (last_iteration) // nothing is un-occluded
                    occludee.set_to_zero();
                // else segment has already been trimmed and nothing needs to be added
            }
            else {
                occludee.setStart(current);
                occludee.setEnd(next);
                first_keeper = false;
            }
        } 
        else 
        {
            if (!l.intersect(occluder, ignore)) {
                lineList.push_back(Line(current, next));
            }
            // else segment has already been trimmed, do nothing
        }

        if (last_iteration) {

            return;
        }

        current = next;
        next = pois.back();
        pois.pop_back();
    }
}


vector<Line> AnalyticRayCast(PointXY x, vector<Line> map) 
{
    vector<Line> Lprime(map);
    vector<Line> L;

    int same_count = 0;
    int count = 0;
    int prevsize = Lprime.size();
    while (Lprime.size() > 0)
    {
        count++;
        // cout << "i: " << Lprime.size() <<"-";
        Line li = Lprime.back();
        Lprime.pop_back();

        // hacky
        // if (Lprime.size() > 0 && li == Lprime.front())
        //     break;

        if (Lprime.size() == prevsize)
            same_count++;
        else {
            same_count = 0;
            prevsize = Lprime.size();
        }

        if (same_count > 10 || count > 200) {
            cout << "same_count: " << same_count << " count: " << count << endl;
            cout << "! - " << L.size() << ", " << Lprime.size() << endl;
            addLinesICanSee(x, L, Lprime);
            cout << "IM HERE " << x.x << "," << x.y << endl;
            return L;
        }

        int j = 0;
        while (j < L.size()) 
        {
            Line lj = L[j];

            if (!(li == lj))
                TrimOcclusion(x, li, lj, Lprime);
            j++;
        }

        if (li.isZero())
            continue;

        j = 0;
        while (j < Lprime.size())
        {
            Line lj = Lprime[j];

            if (!(lj == li))
                TrimOcclusion(x, li, lj, Lprime);
            j++;
        }

        if (li.isZero())
            continue;

        // j = 0;
        // while (true)
        // {
        //     if (j >= L.size())
        //         break;

        //     Line lj = L[j];
        //     TrimOcclusion(x, lj, li, Lprime);
        //     j++;
        // }

        // remove all zeroes
        int i = 0;
        while (i < Lprime.size()) {
            if (Lprime[i].isZero())
                Lprime.erase(Lprime.begin() + i);
            else
                i++;
        }

        L.push_back(li);
    }

    return L;
}


#endif
