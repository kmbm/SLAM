/*
 * GfsCarmen.cpp
 *
 *  Created on: 21 maj 2018
 *      Author: Admin
 */

#include "RunConfig.h"
#include <gridfastslam/gridslamprocessor.h>
#include <configfile/configfile.h>
#include "Runner.h"
#include "SystemParameters.h"
#include "MapLogger.h"

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>

using namespace GMapping;
using namespace std;

#ifdef GFS_MAIN

int main(int argc, const char * const * argv)
{
	GridSlamProcessor* processor=new GridSlamProcessor;

	const RangeSensor* rangeSensor = new RangeSensor("laser", LIDAR_NUM_OF_BEAMS, OrientedPoint(0.0, 0.0, 0.0), 0, 500);
	RangeReading rr(rangeSensor,0);

	processor->setMatchingParameters(maxUrange, maxrange, sigma, kernelSize, lstep, astep, iterations, lsigma, ogain, lskip);
	processor->setMotionModelParameters(srr, srt, str, stt);
	processor->setUpdateDistances(linearUpdate, angularUpdate, resampleThreshold);
	OrientedPoint initialPose(xmin+xmax/2, ymin+ymax/2, 0);

	processor->init(particles, xmin, ymin, xmax, ymax, delta, LIDAR_NUM_OF_BEAMS, initialPose);

	bool running=true;


	auto l_mapLogger = std::make_unique<MapLogger>();
	std::shared_ptr<SensorsDataStorage> l_sensorsDataStorage = std::make_shared<SensorsDataStorage>(LIDAR_NUM_OF_BEAMS);
	auto l_systemRunner = std::make_unique<Runner>(l_sensorsDataStorage);
	std::thread GyroscopeThread(l_systemRunner->gyroscopeThread());
	std::thread LidarThread(l_systemRunner->lidarScan());
	std::thread MapGeneratorThread(l_systemRunner->generateMap());

	double temp=0;
	sleep(2);
	double x=0.8, y=1;
	rr.setPose(OrientedPoint(x, y, 0));
	while (running)
	{
		const auto& l_robotPose = l_sensorsDataStorage->getRobotPose();
		x+=0.2;
		rr.setPose(OrientedPoint(x, y, l_robotPose->angle));
			//x+=0.2;// ++y;
			/*
		std::vector<double> vect1{5,9,9,9,9,9,9,9,9,9,6,9,9,9,9,9,9,9,9,9};
		std::vector<double> vect2{3,9,9,9,9,9,9,9,9,9,8,9,9,9,9,9,9,9,9,9};
		std::vector<double> vect3{1,9,9,9,9,9,9,9,9,9,10,9,9,9,9,9,9,9,9,9};*/

	/*	std::vector<double> vect1{.5,.9,.9,.9,.9,.9,.9,.9,.9,.9,.6,.9,.9,.9,.9,.9,.9,.9,.9,.9};
		std::vector<double> vect2{.3,.9,.9,.9,.9,.9,.9,.9,.9,.9,.8,.9,.9,.9,.9,.9,.9,.9,.9,.9};
		std::vector<double> vect3{.1,.9,.9,.9,.9,.9,.9,.9,.9,.9,1,.9,.9,.9,.9,.9,.9,.9,.9,.9};*/

		std::vector<LidarDataResponse> vect1{ LidarDataResponse(1, 0,    .5),
											   LidarDataResponse(1, 18,   .9),
											   LidarDataResponse(1, 36,   .9),
											   LidarDataResponse(1, 54,   .9),
											   LidarDataResponse(1, 72,   .9),
											   LidarDataResponse(1, 90,   .9),
											   LidarDataResponse(1, 108,  .9),
											   LidarDataResponse(1, 126,  .9),
											   LidarDataResponse(1, 144,  .9),
											   LidarDataResponse(1, 162,  .9),
											   LidarDataResponse(1, 180,  .6),
											   LidarDataResponse(1, 198,  .9),
											   LidarDataResponse(1, 216,  .9),
											   LidarDataResponse(1, 234,  .9),
											   LidarDataResponse(1, 252,  .9),
											   LidarDataResponse(1, 270,  .9),
											   LidarDataResponse(1, 288,  .9),
											   LidarDataResponse(1, 306,  .9),
											   LidarDataResponse(1, 324,  .9),
											   LidarDataResponse(1, 342,  .9) };

		std::vector<LidarDataResponse> vect2{ LidarDataResponse(1, 0,    .3),
											   LidarDataResponse(1, 18,   .9),
											   LidarDataResponse(1, 36,   .9),
											   LidarDataResponse(1, 54,   .9),
											   LidarDataResponse(1, 72,   .9),
											   LidarDataResponse(1, 90,   .9),
											   LidarDataResponse(1, 108,  .9),
											   LidarDataResponse(1, 126,  .9),
											   LidarDataResponse(1, 144,  .9),
											   LidarDataResponse(1, 162,  .9),
											   LidarDataResponse(1, 180,  .8),
											   LidarDataResponse(1, 198,  .9),
											   LidarDataResponse(1, 216,  .9),
											   LidarDataResponse(1, 234,  .9),
											   LidarDataResponse(1, 252,  .9),
											   LidarDataResponse(1, 270,  .9),
											   LidarDataResponse(1, 288,  .9),
											   LidarDataResponse(1, 306,  .9),
											   LidarDataResponse(1, 324,  .9),
											   LidarDataResponse(1, 342,  .9) };

		std::vector<LidarDataResponse> vect3{ LidarDataResponse(1, 0,    .1),
											   LidarDataResponse(1, 18,   .9),
											   LidarDataResponse(1, 36,   .9),
											   LidarDataResponse(1, 54,   .9),
											   LidarDataResponse(1, 72,   .9),
											   LidarDataResponse(1, 90,   .9),
											   LidarDataResponse(1, 108,  .9),
											   LidarDataResponse(1, 126,  .9),
											   LidarDataResponse(1, 144,  .9),
											   LidarDataResponse(1, 162,  .9),
											   LidarDataResponse(1, 180,  1),
											   LidarDataResponse(1, 198,  .9),
											   LidarDataResponse(1, 216,  .9),
											   LidarDataResponse(1, 234,  .9),
											   LidarDataResponse(1, 252,  .9),
											   LidarDataResponse(1, 270,  .9),
											   LidarDataResponse(1, 288,  .9),
											   LidarDataResponse(1, 306,  .9),
											   LidarDataResponse(1, 324,  .9),
											   LidarDataResponse(1, 342,  .9) };

		//std::vector<double> vect2{3,9,9,9,9,9,9,9,9,9,8,9,9,9,9,9,9,9,9,9};
		//std::vector<double> vect3{1,9,9,9,9,9,9,9,9,9,10,9,9,9,9,9,9,9,9,9}
		//rr.setScanReading(l_sensorsDataStorage->getLidarScan()->lidarFilteredReadings);
		if (temp == 0)
		{
			rr.setScanReading(vect1);
			//x=1.2;
		}
		else if(temp == 1)
		{
			rr.setScanReading(vect2);
			//x=1.4;
		}
		else
		{
			rr.setScanReading(vect3);
			//x=1.6;
		}
		bool processed=processor->processScan(rr);

		if (processed){
			++temp;
			if (temp == 3) running = false;
			cerr << "PROCESSED" << endl;

			const GridSlamProcessor::ParticleVector& particles = processor->getParticles();

			cerr << "Particle reproduction story begin" << endl;
			for (unsigned int i=0; i<particles.size(); i++){
				cerr << particles[i].previousIndex << "->"  << i << " ";
			}
			cerr << "Particle reproduction story end" << endl;

			if (running)
			{
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
	auto l_map = processor->getBestParticleMap();
	l_mapLogger->saveMap(l_map);

	GyroscopeThread.join();
	LidarThread.join();
	MapGeneratorThread.join();

	return 0;
}

#endif
