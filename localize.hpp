#ifndef LOCALIZARD
#define LOCALIZARD

#include "raycast.hpp"
#include "movement_keeper.hpp"

#define MAX_NORMAL_DIFF 0.0872 // 0.0872 = 5 degrees
// #define SIGMA .02 // standard deviation of distance measurement
// #define DISCOUNT 10 // discounting factor f <- ??????????
#define KONSTANT .08 // 2 * SIGMA^2 * f

// Map boundaries
#define MAPMAXY 7.62
#define MAPMAXX 10.21

using namespace std;
using namespace pcl;

class Particle {
    public:
        Particle(PointXY pos, float angle);
        void setWeight(float w);
        float getWeight();
        PointXY getPos();
        float getAngle();
        void moveP(float x, float y, float ang);
    private:
        float weight;
        PointXY pos;
        float angle;
};

Particle::Particle(PointXY p, float a) {
    pos = p;
    angle = a;
}

void Particle::setWeight(float w) {
    weight = w;
}

float Particle::getWeight() {
    return weight;
}

PointXY Particle::getPos() {
    return pos;
}

float Particle::getAngle() {
    return angle;
}

void Particle::moveP(float x, float y, float ang) {
    angle += ang;
    pos.x += x;
    pos.y += y;
}

// CGR /////////////////////////////////////

void motionEvolve(vector<Particle> &belief, MovementKeeper mk) 
{
    for (int i = 0; i < belief.size(); i++) 
    {
        belief[i].moveP(mk.getXPos(), mk.getYPos(), mk.getAngle());
    }

    for (int i = belief.size() - 1; i >= 0; i--) 
    {
        float curx = belief[i].getPos().x;
        float cury = belief[i].getPos().y;
        if (curx > MAPMAXX || curx < 0 || cury > MAPMAXY || cury < 0) {
            belief.erase(belief.begin() + i);
        }
    }
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
    float maxp = 0;
    float sump = 0;

    for (int i = 0; i < belief.size(); i++)
    // iterating through particles in belief to calculate p - belief index is i
    {
        vector<Line> raycastMap;
        AnalyticRayCast(belief[i].getPos(), map, raycastMap);

        float obsLikelihood = 1.0;

        for (int j = 0; j < cloud.size(); j++) 
        // iterate through points in cloud - point (AND NORMAL) index is j
        {
            PointXY cloudpt;
            cloudpt.x = cloud[j].x + belief[i].getPos().x;
            cloudpt.y = cloud[j].y + belief[i].getPos().y;

            Line toCloudpt(belief[i].getPos(), cloudpt);

            PointXY intersection;

            // iterate through viewable lines raycasted map - line index is k
            for (int k = 0; k < raycastMap.size(); k++) 
            {
                if (toCloudpt.intersectOutOfBound(raycastMap[k], &intersection)) {
                    Line wall = raycastMap[k];

                    // convert normal from robot-space to map-space
                    PointXY n = convertNorm(normals[j], belief[i].getAngle());

                    // calculate angle between map-space point normal n (0, 0) to (<PointXY>) and wall normal (Line)
                    if (wall.angleAboveMax(n, MAX_NORMAL_DIFF)) {
                        float di2 = dst2dsqd(intersection, cloudpt);
                        obsLikelihood *= exp(-di2/KONSTANT);
                    }

                    // found corresponding wall; break out of loop
                    break;
                }

            }
        }

        belief[i].setWeight(obsLikelihood);

        // find maximum weigt
        if (obsLikelihood > maxp) {
            maxp = obsLikelihood;
        }

        // find avg weigt
        sump += obsLikelihood;
    }

    float avgp = sump/belief.size();

    // Pick only particles with highest P (based on maxp and avgp)


    // Add in some random particles


}


#endif
