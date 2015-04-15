#ifndef LOCALIZARD
#define LOCALIZARD

#include "raycast.hpp"
#include "movement_keeper.hpp"

#define MAX_NORMAL_DIFF 0.0872 // 0.0872 = 5 degrees
// #define SIGMA .02 // standard deviation of distance measurement
// #define DISCOUNT 10 // discounting factor f <- ??????????
#define KONSTANT .08 // 2 * SIGMA^2 * f
#define KEEP_RATIO .1
#define NEW_SAMPS 6

// Map boundaries
#define MAPMAXY 7.62
#define MAPMAXX 10.21

#define ANGLE_VARIANCE 0.17 // in rads = ~10 degrees
#define DIST_VARIANCE 0.3 // in meters

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

bool operator<(Particle p1, Particle p2) { 
     return (p1.getWeight() < p2.getWeight());
}

// CGR /////////////////////////////////////

Particle randomParticle() {
    PointXY pos;
    pos.x = fmod(rand(), MAPMAXX);
    pos.y = fmod(rand(), MAPMAXY);

    return Particle( pos, fmod(rand(),(2 * PI)) );
}

void checkBounds(vector<Particle> &belief) {
    for (int i = belief.size() - 1; i >= 0; i--) 
    {
        float curx = belief[i].getPos().x;
        float cury = belief[i].getPos().y;
        if (curx > MAPMAXX || curx < 0 || cury > MAPMAXY || cury < 0) {
            // if OB, replace wit random particle
            belief[i] = randomParticle();

            // belief.erase(belief.begin() + i);
        }
    }
}

void motionEvolve(vector<Particle> &belief, MovementKeeper mk) 
{
    for (int i = 0; i < belief.size(); i++) 
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
    // TODO: Move this to main after #definitions are abstracted
    if (KEEP_RATIO * (NEW_SAMPS + 1) > 1.0) {
        cout << "!! KEEP_RATIO is too big for NEW_SAMPS value !!" << endl;
        return;
    }

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
    }

    // Sort particles in descending order by weight
    sort(belief.end(), belief.begin());

    // Print best guess
    cout << "------------------------" << endl;
    cout << "Current Best Guess: " << endl;
    cout << "Position-x: " << belief[0].getPos().x << endl;
    cout << "Position-y: " << belief[0].getPos().y << endl;
    cout << "Position-y: " << belief[0].getPos().y << endl;
    cout << "Angle: " << belief[0].getAngle() << endl;
    cout << "------------------------" << endl;

    // choose best KEEP_RATIO percent, create NEW_SAMPS new samples near each
    int keepers = KEEP_RATIO * belief.size();

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
    for (int i = keepers*NEW_SAMPS; i < belief.size(); i++) {
        belief[i] = randomParticle();
    }

    checkBounds(belief);
}


#endif
