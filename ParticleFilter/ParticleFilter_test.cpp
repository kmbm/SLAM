#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <ParticleFilter/ParticleFilter.h>
#include <RunConfig.h>

#include <Utils/Point.h>

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

void validate(std::vector<Particle> particles, std::vector<Particle> expectedParticles)
{
	if(equal(particles.begin(), particles.end(), expectedParticles.begin()))
	{
		std::cout << "PASS" << std::endl;
	}
	else
	{
		std::cout << "FAIL" << std::endl;
	}
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
	auxiliary_evolver <Particle, double, QualificationModel, EvolutionModel, LikelyhoodModel> auxevolver;
	evolver <Particle, EvolutionModel> evolver;

	int particle_val = 0;
	for (vector<Particle>::iterator it=particles.begin(); it!=particles.end(); it++){
		it->w=1;
		it->p= particle_val;
		++particle_val;
	}
	vector<Particle> sirparticles(particles);
	vector<Particle> auxparticles(particles);
	vector<Particle> expectedsirparticles = {{1.6218, 0.65467},
											 {1.92439, 6.36401},
											 {1.93078, 6.76207},
										     {1.834, 2.664},
										     {8.30738, 0.481052},
										     {7.98231, 4.851},
										     {8.23684, 0.759087},
										     {7.73726, 0.635689},
										     {8.26845, 0.611827},
										     {8.17425, 1.24143}};


	vector<Particle> expectedauxparticles = {{1.7211, 1.14042},
											 {2.20117, 1.98289},
											 {2.21673, 1.75671},
										   	 {1.9477, 7.85379},
											 {2.27017, 1.20648},
											 {1.98984, 9.89915},
											 {7.8323, 1.31446},
											 {7.79228, 0.943745},
											 {7.92298, 3.14116},
											 {7.76618, 0.776133}};

	int i = 0;
	while (i < 3)
	{
	    ++i;
		vector<Particle> newgeneration;

		evolver.evolve(sirparticles);
		for (vector<Particle>::iterator it=sirparticles.begin(); it!=sirparticles.end(); it++){
			it->setWeight(likelyhoodModel.likelyhood(*it));
		}
		ofstream os("sir.dat");
		printParticles(os, sirparticles);
		os.close();

		if (i == 3)
		{
			validate(sirparticles, expectedsirparticles);
		}

		newgeneration=resampler.resample(sirparticles);
		sirparticles=newgeneration;

		auxevolver.evolve(auxparticles);
		for (vector<Particle>::iterator it=auxparticles.begin(); it!=auxparticles.end(); it++){
			it->setWeight(likelyhoodModel.likelyhood(*it));
		}
		os.open("aux.dat");
		printParticles(os, auxparticles);
		os.close();

		if (i == 3)
		{
			validate(auxparticles, expectedauxparticles);
		}
		newgeneration=resampler.resample(auxparticles);
		auxparticles=newgeneration;
	}
}

#endif

