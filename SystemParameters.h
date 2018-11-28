/*
 * SystemParameters.h
 *
 *  Created on: Oct 3, 2018
 *      Author: Krzysiek
 */

#ifndef SRC_SYSTEMPARAMETERS_H_
#define SRC_SYSTEMPARAMETERS_H_

	std::string outfilename="";
	double xmin=-10.;
	double ymin=-10.;
	double xmax=10.;
	double ymax=10.;
	double delta=0.05;
	//double delta = 1;
	//scan matching parameters
	double sigma=0.05;
	//double sigma=1;
	double maxrange=8.;
	double maxUrange=8.;
	double regscore=1e4;
	double lstep=.05;
	double astep=.05;
	//double lstep=1;
	//double astep=1;
	int kernelSize=1;
	int iterations=5;
	double critscore=0.;
	double maxMove=0.2;
	double lsigma=.075;
	double ogain=3;
	int lskip=0;

	//motion model parameters
	double srr=0.01, srt=0.01, str=0.01, stt=0.01;
	//particle parameters
	int particles=30;


	//gfs parameters
	//double angularUpdate=0.5;
	//double linearUpdate=1;
	double angularUpdate=0.5;
	double linearUpdate=0.05;
	double resampleThreshold=0.5;
	bool generateMap=true;

	int LIDAR_NUM_OF_BEAMS = 20;




#endif /* SRC_SYSTEMPARAMETERS_H_ */
