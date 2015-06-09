#ifndef LOCALIZARD
#define LOCALIZARD

#include "definitions.hpp"
#include "raycast.hpp"
#include "movement_keeper.hpp"
#include "particle.hpp"
#include "new_samples.hpp"

using namespace std;
using namespace pcl;

// CGR /////////////////////////////////////

void motionEvolve(vector<Particle> &belief, MovementKeeper mk) 
{
    cout << "Motion evolve x: " << mk.getXPos() 
        << " Y: " << mk.getYPos() << "  Angle: " << mk.getAngle() << endl;
    int beliefsize = belief.size();
    for (int i = 0; i < beliefsize; i++) 
    {
        belief[i].moveP(mk.getXPos(), mk.getYPos(), mk.getAngle());
    }

    checkBounds(belief);
}

// convert normal from robot-space to map-space
PointXY convertNorm(PointXY p, float ang) {
    // rotate p by ang
    PointXY newp;
    newp.x = p.x * cos(ang) - p.y * sin(ang);
    newp.y = p.x * sin(ang) + p.y * cos(ang);
    if (newp.x != newp.x) {
        // this happened - normal returned by fspf must be vertical
        cout << "Error converting normal to mapspace" << endl;
        cout << ang << endl;
        throw 2;
    }
    return newp;
}

void CGRLocalize(vector<Particle> &belief, PointCloud<PointXYZ> cloud, 
                 PointCloud<PointXY> normals, vector<Line> map)
{
    int beliefsize = belief.size();
    svgPrint(map, -1, belief[0].getPos());
    float totalWeight = 0.0;

    for (int i = 0; i < beliefsize; i++)
    // iterating through particles in belief to calculate p - belief index is i
    {
        int pointsin = 0;
        int pointsout = 0;

        vector<Line> raycastMap = AnalyticRayCast(belief[i].getPos(), map);
        int raycastmapsize = raycastMap.size();

        svgPrint(raycastMap, i, belief[i].getPos());
        
        float obsLikelihood = 1.0;

        int cloudsize = cloud.size();
        for (int j = 0; j < cloudsize; j++) 
        // iterate through points in cloud - point (AND NORMAL) index is j
        {
            PointXY cloudpt;
            cloudpt.x = cloud[j].x + belief[i].getPos().x;
            cloudpt.y = cloud[j].y + belief[i].getPos().y;

            Line toCloudpt(belief[i].getPos(), cloudpt);

            PointXY intersection;

            int oldpointsin = pointsin;

            for (int k = 0; k < raycastmapsize; k++)
            // iterate through viewable lines raycasted map - line index is k
            {
                if (raycastMap[k].isZero())
                    continue;

                if (toCloudpt.intersectOutOfBound(raycastMap[k], intersection) || 
                    toCloudpt.intersect(raycastMap[k], intersection)) 
                {
                    Line wall = raycastMap[k];

                    // convert normal from robot-space to map-space
                    PointXY n = convertNorm(normals[j], belief[i].getAngle());

                    // calculate angle between map-space point normal n 
                    // (0, 0) to (<PointXY>) and wall normal (Line)
                    if (wall.angleAboveMax(n, MAX_NORMAL_DIFF)) {
                        float di2 = dst2dsqd(intersection, cloudpt);
                        obsLikelihood = obsLikelihood * exp(-di2/KONSTANT);
                        pointsin++;
                    } else {
                        cout << "n:" << endl;
                        cout << n.x << "," << n.y << endl;
                        cout << "Wall:" << endl;
                        wall.print();
                        throw 1;
                    }

                    // found corresponding wall; break out of loop
                    break;
                }
            }
            if (pointsin == oldpointsin)
                pointsout++;

        }
        // print pointsin, pointsout, obsLikelihood
        // pointsout should be low and only correspond to a few wall-less areas
        cout << i << " Obs: " << obsLikelihood << " In: " << pointsin << " Out: " 
        << pointsout << " Pos: " << belief[i].getPos().x << ", " 
        << belief[i].getPos().y << " - " << belief[i].getAngle()  << endl;

        belief[i].setWeight(obsLikelihood);
        totalWeight += obsLikelihood;
    }

    bubbleSort(belief); // lol

    particlePrint(belief, map);

    // Print best guess
    cout << "------------------------" << endl;
    cout << "Current Best Guess: " << endl;
    cout << "Position-x: " << belief[0].getPos().x << endl;
    cout << "Position-y: " << belief[0].getPos().y << endl;
    cout << "Angle: " << belief[0].getAngle() << endl;
    cout << "Weight: " << belief[0].getWeight() << endl;
    cout << "------------------------" << endl;

    cout << "Resampling..." << endl;

    belief = newSamples(belief, totalWeight);

    cout << "Finished CGR fxn" << endl;
}


#endif
