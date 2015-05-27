#ifndef PARTICLE
#define PARTICLE

#include "lines.hpp"

using namespace std;


class Particle {
    public:
        Particle() {};
        Particle(PointXY pos, float angle);
        void setWeight(float w);
        float getWeight() const;
        PointXY getPos() const;
        float getAngle() const;
        void moveP(float x, float y, float ang);
        void print();
        // Note reversal: geq is opposite of < for descending order sort
        bool operator<(const Particle p) const { return weight >= p.getWeight(); }
    private:
        PointXY pos;
        float weight;
        float angle;
};

Particle::Particle(PointXY p, float a) {
    pos = p;
    angle = a;
}

void Particle::print() {
    cout << "Particle=>  pos: " << pos.x << "," << pos.y << " ;  angle:" << angle << endl;
}

void Particle::setWeight(float w) {
    weight = w;
}

float Particle::getWeight() const {
    return weight;
}

PointXY Particle::getPos() const {
    return pos;
}

float Particle::getAngle() const {
    return angle;
}

void Particle::moveP(float x, float y, float ang) {
    angle += ang;
    pos.x += x;
    pos.y += y;
}

void bubbleSort(vector<Particle> &arr) { // could do better
    int n = arr.size();
    bool swapped = true;
    int j = 0;
    Particle tmp;
    while (swapped) {
        swapped = false;
        j++;
        for (int i = 0; i < n - j; i++) {
            if (arr[i].getWeight() < arr[i + 1].getWeight()) {
                tmp = arr[i];
                arr[i] = arr[i + 1];
                arr[i + 1] = tmp;
                swapped = true;
            }
        }
    }
}

void particlePrint(vector<Particle> b, vector<Line> lines)
{
    srand (time(NULL));
    int imgsize = 800;
    float k = float(imgsize)/12.5;

    ofstream f;
    f.open("../plots/particles.html");

    f << "<!DOCTYPE html><html><body><svg version=\"1.1\""
        << "style=\"background: black\" "
        << "baseProfile=\"full\" width=\"" 
        << imgsize << "\" height=\"" 
        << imgsize << "\" xmlns=\"http://www.w3.org/2000/svg\">\n\n";

    for (int i = 0; i < b.size(); i++)
    {
        if (b[i].getWeight() < 0)
            continue;
        
        PointXY p = b[i].getPos();

        f << "<circle cx=\"" << int(5 + p.x * k) << "\" cy=\"" << 795 - int((p.y+1.5) * k)
            << "\" r=\"5\" fill=\"rgb(" 
            << int(255 - 255*i/b.size()) << ",0," // red
            << int(255*i/b.size()) // blue
            << "\" />\n\n";
    }

    int linesize = lines.size();
    for (int i = 0; i < linesize; i++)
    {
        if (lines[i].isZero())
            continue;

        int x1 = 5 + lines[i].getStart().x * k;
        int x2 = 5 + lines[i].getEnd().x * k;
        int y1 = 795 - (lines[i].getStart().y + 1.5) * k;
        int y2 = 795 - (lines[i].getEnd().y + 1.5) * k;

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
