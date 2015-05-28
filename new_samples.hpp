#ifndef NEWSAMPLES
#define NEWSAMPLES

#include "definitions.hpp"
#include "particle.hpp"

using namespace std;


vector<Particle> newSamples(vector<Particle> belief, float totalWeight)
// O(n^2); n = belief.size()
{
	totalWeight += totalWeight * RANDOM_PARTICLES;
	int beliefsize = belief.size();
	vector<Particle> new_belief(belief);

	for (int i = 0; i < beliefsize; i++)
	{
		float totalWeightSoFar = 0.0;
		bool randomize_particle = true;
		float random_number = fmod( rand(), totalWeight );

		for (int j = 0; j < beliefsize; j++)
		{
			totalWeightSoFar += belief[j].getWeight();

			if (random_number < totalWeightSoFar)
			{
				// create slightly modified version of belief[j]
	            PointXY x;
	            float angle = belief[j].getAngle() + fmod(rand(),ANGLE_VARIANCE) - (ANGLE_VARIANCE/2.0);
	            x.x = belief[j].getPos().x + fmod(rand(),DIST_VARIANCE) - (DIST_VARIANCE/2.0);
	            x.y = belief[j].getPos().y + fmod(rand(),DIST_VARIANCE) - (DIST_VARIANCE/2.0);
				new_belief[i] = Particle(x, angle);

				randomize_particle = false;
				break;
			}
		}

		if (randomize_particle) {
			new_belief[i] = randomParticle();
		}
	}

	checkBounds(new_belief);

	return new_belief;
}


#endif
