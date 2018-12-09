/*
 * ParticleFilter.h
 *
 *  Created on: 19 maj 2018
 *      Author: Admin
 */

#ifndef PARTICLEFILTER_PARTICLEFILTER_H_
#define PARTICLEFILTER_PARTICLEFILTER_H_

#include <stdlib.h>
#include <vector>
#include <numeric>

struct uniform_resampler
{
	inline std::vector<unsigned int> resampleIndexes(const std::vector<double> & particles, int nparticles=0) const;
};

std::vector<unsigned int> uniform_resampler::resampleIndexes(const std::vector<double>& particles, int nparticles) const
{
	double interval = std::accumulate(particles.begin(), particles.end(), 0.0) / particles.size();
	double target=interval*::drand48();

	int numOfParticles = 0;
	double cweight = 0;
	std::vector<unsigned int> indexes(particles.size());
	unsigned int index=0;

	for (auto l_it : particles)
	{
		cweight+= l_it;
		while (cweight > target)
		{
			indexes[numOfParticles++] = index;
			target += interval;
		}
		++index;
	}
	return indexes;
}


#endif /* PARTICLEFILTER_PARTICLEFILTER_H_ */
