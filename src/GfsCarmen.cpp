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
#include "SystemParameters.h"

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

#define DEBUG cout << __PRETTY_FUNCTION__

using namespace GMapping;
using namespace std;

#ifdef GFS_MAIN

int main(int argc, const char * const * argv){



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

	//this is the CORE LOOP;
	RangeReading rr(0,0);

	std::shared_ptr<SensorsDataStorage> l_sensorsDataStorage;
	auto l_systemRunner = std::make_unique<Runner>(l_sensorsDataStorage);
	std::thread GyroscopeThread(l_systemRunner->gyroscopeThread());
	double i=0;
	while (running){
		//while (CarmenWrapper::getReading(rr)){
		while (true){
			sleep(1);
			//RangeReading temp(0,0);
			//rr = temp;
			//rr.setPose(OrientedPoint(i, 1.0, p_poseAngle));
			auto l_robotPose = l_sensorsDataStorage->getRobotPose();
			rr.setPose(OrientedPoint(l_robotPose.x, l_robotPose.y, l_robotPose.angle));
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
