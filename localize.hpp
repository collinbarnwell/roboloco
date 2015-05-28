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
    return newp;
}

void CGRLocalize(vector<Particle> &belief, PointCloud<PointXYZ> cloud, PointCloud<PointXY> normals, vector<Line> map)
{
    int beliefsize = belief.size();
    svgPrint(map, -1, belief[0].getPos());
    float totalWeight = 0.0;

    for (int i = 0; i < beliefsize; i++)
    // iterating through particles in belief to calculate p - belief index is i
    {
        // int pointsin = 0;
        // int pointsout = 0;

        vector<Line> raycastMap = AnalyticRayCast(belief[i].getPos(), map);
        int raycastmapsize = raycastMap.size();

        // if (i < 100)
        //     svgPrint(raycastMap, i, belief[i].getPos());
        
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

            // int oldpointsin = pointsin;

            for (int k = 0; k < raycastmapsize; k++)
            // iterate through viewable lines raycasted map - line index is k
            {
                if (toCloudpt.intersectOutOfBound(raycastMap[k], intersection) || 
                    toCloudpt.intersect(raycastMap[k], intersection)) 
                {
                    Line wall = raycastMap[k];

                    // convert normal from robot-space to map-space
                    PointXY n = convertNorm(normals[j], belief[i].getAngle());

                    // calculate angle between map-space point normal n (0, 0) to (<PointXY>) and wall normal (Line)
                    
                    if (wall.angleAboveMax(n, MAX_NORMAL_DIFF)) {
                        float di2 = dst2dsqd(intersection, cloudpt);
                        obsLikelihood = obsLikelihood * exp(-di2/KONSTANT);
                        // pointsin++;
                    }

                    // found corresponding wall; break out of loop
                    break;
                }
            }
            // if (pointsin == oldpointsin) {
            //     pointsout++;
            // }
        }

        // cout << "B: " << obsLikelihood << ", IN: " << pointsin << ", OUT: " << pointsout << endl;

        belief[i].setWeight(obsLikelihood);

        // if (pointsin < 10)
        //     belief[i].setWeight(-1);

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

    /* TRASH ------------------>>>>>>

    // choose best KEEP_RATIO percent, create NEW_SAMPS new samples near each
    int keepers = KEEP_RATIO * beliefsize;

    for (int i = 0; i < keepers; i++) 
    {
        // create NEW_SAMPS - 1, new particles near each keeper + keep self
        // [k k ns ns ns ns space space space]

        for (int j = 0; j < NEW_SAMPS; j++) 
        {
            PointXY x;
            float angle = belief[i].getAngle() + fmod(rand(),ANGLE_VARIANCE) - (ANGLE_VARIANCE/2.0);
            x.x = belief[i].getPos().x + fmod(rand(),DIST_VARIANCE) - (DIST_VARIANCE/2.0);
            x.y = belief[i].getPos().y + fmod(rand(),DIST_VARIANCE) - (DIST_VARIANCE/2.0);

            belief[keepers + i*NEW_SAMPS + j] = Particle(x, angle);
            // last assigned is belief[keepers + (keepers-1)*NEW_SAMPS + (NEW_SAMPS-1) - 1]
            // = belief[keepers*NEW_SAMPS - 1]
        }
    }

    // Add in remaining percentage random particles
    for (int i = keepers*NEW_SAMPS; i < beliefsize; i++) {
        belief[i] = randomParticle();
    }

    checkBounds(belief);
    */
