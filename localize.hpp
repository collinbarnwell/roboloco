#ifndef LOCALIZARD
#define LOCALIZARD

#include "fspf.hpp"
#include "raycast.hpp"


using namespace std;
using namespace pcl;

class Particle {
    public:
        Particle(pos, angle);
        void setWeight();
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

void CGRLocalize(vector<Particle> &belief, PointCloud<PointXYZ> cloud, unordered_map<PointXYZ, Normal> normals) {




}

float obsLikelihood(float angle, PointXY point) {

}



#endif
