/*
 * GfsCarmen.cpp
 *
 *  Created on: 21 maj 2018
 *      Author: Admin
 */

#include "RunConfig.h"
#include <utils/commandline.h>
//#include <Carmen/CarmenWrapper.h>
#include <gridfastslam/gridslamprocessor.h>
//#include <utils/orientedboundingbox.h>
#include <configfile/configfile.h>
#include "Runner.h"

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#define DEBUG cout << __PRETTY_FUNCTION__

/*
Example file for interfacing carmen, and gfs.

if you want to look for a specific topic search for one of the following keywords in the file comments

KEYWORDS:
	CREATION
	INITIALIZATION
	SENSOR MAP
	BEST PARTICLE INDEX
	PARTICLE VECTOR
	PARTICLE TRAJECTORIES
	BEST MAP
 	BOUNDING BOX
*/

using namespace GMapping;
using namespace std;

#ifdef GFS_MAIN

int main(int argc, const char * const * argv){

	std::string outfilename="";
	double xmin=-100.;
	double ymin=-100.;
	double xmax=100.;
	double ymax=100.;
	double delta=0.05;

	//scan matching parameters
	double sigma=0.05;
	double maxrange=80.;
	double maxUrange=80.;
	double regscore=1e4;
	double lstep=.05;
	double astep=.05;
	int kernelSize=1;
	int iterations=5;
	double critscore=0.;
	double maxMove=1.;
	double lsigma=.075;
	double ogain=3;
	int lskip=0;

	//motion model parameters
	double srr=0.01, srt=0.01, str=0.01, stt=0.01;
	//particle parameters
	int particles=30;


	//gfs parameters
	double angularUpdate=0.5;
	double linearUpdate=1;
	double resampleThreshold=0.5;
	bool generateMap=true;


	//CarmenWrapper::initializeIPC(argv[0]);
	//CarmenWrapper::start(argv[0]);

/*	while (! CarmenWrapper::sensorMapComputed()){
		usleep(500000);
		cerr << "." << flush;
	}*/

	//CREATION

	GridSlamProcessor* processor=new GridSlamProcessor;

	//SENSOR MAP
	//loads from the carmen wrapper the laser and robot settings
	//SensorMap sensorMap=CarmenWrapper::sensorMap();
	cerr << "Connected " << endl;
	processor->setSensorMap();  //!!!!!!!!!!!!! - laser not set

	//set the command line parameters
	processor->setMatchingParameters(maxUrange, maxrange, sigma, kernelSize, lstep, astep, iterations, lsigma, ogain, lskip);
	processor->setMotionModelParameters(srr, srt, str, stt);
	processor->setUpdateDistances(linearUpdate, angularUpdate, resampleThreshold);
	//processor->setgenerateMap(true); //!!!!!!!!
	OrientedPoint initialPose(xmin+xmax/2, ymin+ymax/2, 0);


	//INITIALIZATION
	processor->init(particles, xmin, ymin, xmax, ymax, delta, initialPose);
	if (outfilename.length()>0)
		processor->outputStream().open(outfilename.c_str());

	bool running=true;

	GridSlamProcessor* ap, *copy=processor->clone();
	ap=processor; processor=copy; copy=ap;

	//this is the CORE LOOP;
	RangeReading rr(0,0);

	int p_poseAngle;
	auto l_systemRunner = std::make_unique<Runner>(p_poseAngle);
	std::thread GyroscopeThread(l_systemRunner->gyroscopeThread());
	double i=0;
	while (running){
		//while (CarmenWrapper::getReading(rr)){
		while (true){
			sleep(1);
			//RangeReading temp(0,0);
			//rr = temp;
			rr.setPose(OrientedPoint(i, 1.0, p_poseAngle));
			rr.resize(8);/*
			rr[0] = 1.0;
			rr[1] = 1.0;
			rr[2] = 4.0;
			rr[3] = 5.0;*/
			bool processed=processor->processScan(rr);
			i+=10;
			//this returns true when the algorithm effectively processes (the traveled path since the last processing is over a given threshold)
			if (processed){
				cerr << "PROCESSED" << endl;
				//for searching for the BEST PARTICLE INDEX
				//				unsigned int best_idx=processor->getBestParticleIndex();

				//if you want to access to the PARTICLE VECTOR
				const GridSlamProcessor::ParticleVector& particles = processor->getParticles();
				//remember to use a const reference, otherwise it copys the whole particles and maps

				//this is for recovering the tree of PARTICLE TRAJECTORIES (obtaining the ancestor of each particle)
				cerr << "Particle reproduction story begin" << endl;
				for (unsigned int i=0; i<particles.size(); i++){
					cerr << particles[i].previousIndex << "->"  << i << " ";
				}
				cerr << "Particle reproduction story end" << endl;
/*
				//then if you want to access the BEST MAP,
				//of course by copying it in a plain structure
				Map<double, DoubleArray2D, false>* mymap = processor->getParticles()[best_idx].map.toDoubleMap();
				//at this point mymap is yours. Can do what you want.

				double best_weight=particles[best_idx].weightSum;
				cerr << "Best Particle is " << best_idx << " with weight " << best_weight << endl;

*/
				cerr << __PRETTY_FUNCTION__  << "CLONING... " << endl;
				GridSlamProcessor* newProcessor=processor->clone();
				cerr << "DONE" << endl;
				cerr << __PRETTY_FUNCTION__  << "DELETING... " << endl;
				delete processor;
				cerr << "DONE" << endl;
				processor=newProcessor;
			}
		}
	}
	return 0;
}

#endif
