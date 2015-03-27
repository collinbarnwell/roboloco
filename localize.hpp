#ifndef LOCALIZARD
#define LOCALIZARD

#include "fspf.hpp"
#include "raycast.hpp"

#define MAX_NORMAL_DIFF 0.0872 // 0.0872 = 5 degrees
// #define SIGMA .02 // standard deviation of distance measurement
// #define DISCOUNT 10 // discounting factor f <- ??????????
#define KONSTANT .08 // 2 * SIGMA^2 * f

using namespace std;
using namespace pcl;

class Particle {
    public:
        Particle(pos, angle);
        void setWeight(float w);
        float getWeight();
        PointXY getPos();
        float getAngle();
    private:
        float weight;
        PointXY pos;
        float angle;
};

Particle::Particle(p, a) {
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

// CGR /////////////////////////////////////

void motionEvolve(vector<Particle> &belief) { // need to add other argument for accumulated commands

}

void CGRLocalize(vector<Particle> &belief, PointCloud<PointXYZ> cloud, unordered_map<PointXYZ, Normal> normals, vector<Line> map) {

    motionEvolve(belief);

    for (int i = 0; i < belief.size(); i++)
    // iterating through particles in belief
    {
        vector<Line> raycastMap;
        AnalyticRayCast(belief[i].pos, map, raycastMap)

        float obsLikelihood = 1.0;


        for (int j = 0; j < cloud.size(); j++) 
        // iterate through points in cloud
        {
            PointXY cloudpt;
            cloudpt.x = cloud[j].x + belief[i].pos.x;
            cloudpt.y = cloud[j].y + belief[i].pos.y;

            Line toCloudpt(belief[i].pos, cloudpt);
            Line wall;

            PointXY intersection;

            // iterate through viewable lines raycasted map
            for (int k = 0; k < raycastMap.size(); k++) 
            {
                if (toCloudpt.intersectOutOfBound(raycastMap[k], &intersection)) {
                    wall = raycastMap[k];

                    if ( angle < MAX_NORMAL_DIFF) {
                        float di2 = dst2dqd(intersection, cloudpt);
                        obsLikelihood *= exp(-di2/KONSTANT);
                    }

                    break;
                }

            }
        }

        // do more stuff with particle and obsLikelihood:

        
        // 

        
    }




}



// ---
// linear: 
//   x: 0.5
//   y: 0.0
//   z: 0.0
// angular: 
//   x: 0.0
//   y: 0.0
//   z: 0.168472617865



#endif