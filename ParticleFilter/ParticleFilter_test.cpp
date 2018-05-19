#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <ParticleFilter/ParticleFilter.h>
#include <RunConfig.h>

using namespace std;


struct Particle{
	double p;
	double w;
	Particle(){}
	Particle(double p, double w){this->p = p; this->w = w;}
	inline operator double() const {return w;}
	inline bool operator==(const Particle& particle){return abs(this->p - particle.p) < 0.0001 && abs(this->w - particle.w) < 0.0001;}
	inline void setWeight(double _w) {w=_w;}
};

ostream& printParticles(ostream& os, const vector<Particle>& p)
{
	for (vector<Particle>::const_iterator it=p.begin(); it!=p.end(); ++it) {
		os << it->p<< " " << it->w << endl;
	}
	return os;
}

struct EvolutionModel{
	Particle evolve(const Particle& p){
		Particle pn(p);
		pn.p+=.5*(drand48()-.5);
		return pn;
	}
};

struct QualificationModel{
	Particle evolve(const Particle& p){
		return p;
	}
};

struct LikelyhoodModel{
	double likelyhood(const Particle& p) const{
		double v = 1./(0.1+10*(p.p-2)*(p.p-2))+0.5/(0.1+10*(p.p-8)*(p.p-8));
		return v;
	}
};

#ifdef PARTICLE_FILTER_TEST
int main (unsigned int argc, const char * const * argv){
	int nparticles=10;

	vector<Particle> particles(nparticles);
	LikelyhoodModel likelyhoodModel;
	uniform_resampler<Particle, double> resampler;
	evolver <Particle, EvolutionModel> evolver;

	int particle_val = 0;
	for (vector<Particle>::iterator it=particles.begin(); it!=particles.end(); it++){
		it->w=1;
		it->p= particle_val;
		++particle_val;
	}
	vector<Particle> sirparticles(particles);
	vector<Particle> expectedparticles = {{2.14659, 3.17715},
										  {1.94418, 7.62552},
										  {2.1419, 3.31992},
										  {1.83089, 2.59214},
										  {2.17984, 2.3632},
										  {1.84015, 2.81416},
										  {7.82981, 1.28615},
										  {8.30454, 0.489166},
										  {8.132, 1.82587},
										  {7.72165, 0.57462}};

	int i = 0;
	while (i < 3){
	    ++i;
		vector<Particle> newgeneration;

		evolver.evolve(sirparticles);
		for (vector<Particle>::iterator it=sirparticles.begin(); it!=sirparticles.end(); it++){
			it->setWeight(likelyhoodModel.likelyhood(*it));
		}

		ofstream os("sir.dat");
		printParticles(os, sirparticles);
		os.close();

		if (i < 3)
		{
			newgeneration=resampler.resample(sirparticles);
			sirparticles=newgeneration;
		}
	}
	if(equal(sirparticles.begin(), sirparticles.end(), expectedparticles.begin()))
	{
		std::cout << "PASS" << std::endl;
	}
	else
	{
		std::cout << "FAIL" << std::endl;
	}
}

#endif

