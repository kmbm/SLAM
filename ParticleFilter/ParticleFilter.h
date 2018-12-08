/*
 * ParticleFilter.h
 *
 *  Created on: 19 maj 2018
 *      Author: Admin
 */

#ifndef PARTICLEFILTER_PARTICLEFILTER_H_
#define PARTICLEFILTER_PARTICLEFILTER_H_

#include <stdlib.h>
#include <sys/types.h>
#include <vector>
#include <utility>
#include <cmath>
#include <values.h>


template <class Particle, class Numeric>
struct uniform_resampler{
	std::vector<unsigned int> resampleIndexes(const std::vector<Particle> & particles, int nparticles=0) const;
};


template <class Particle, class Numeric>
std::vector<unsigned int> uniform_resampler<Particle, Numeric>::resampleIndexes(const std::vector<Particle>& particles, int nparticles) const{
	Numeric cweight=0;

	//compute the cumulative weights
	unsigned int n=0;
	for (typename std::vector<Particle>::const_iterator it=particles.begin(); it!=particles.end(); ++it){
		cweight+=(Numeric)*it;
		n++;
	}

	if (nparticles>0)
		n=nparticles;

	//compute the interval
	Numeric interval=cweight/n;

	//compute the initial target weight
	Numeric target=interval*::drand48();
	//compute the resampled indexes

	cweight=0;
	std::vector<unsigned int> indexes(n);
	n=0;
	unsigned int i=0;
	for (typename std::vector<Particle>::const_iterator it=particles.begin(); it!=particles.end(); ++it, ++i){
		cweight+=(Numeric)* it;
		while(cweight>target){
			indexes[n++]=i;
			target+=interval;
		}
	}
	return indexes;
}


#endif /* PARTICLEFILTER_PARTICLEFILTER_H_ */
