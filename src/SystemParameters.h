/*
 * SystemParameters.h
 *
 *  Created on: Oct 3, 2018
 *      Author: Krzysiek
 */

#ifndef SRC_SYSTEMPARAMETERS_H_
#define SRC_SYSTEMPARAMETERS_H_

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




#endif /* SRC_SYSTEMPARAMETERS_H_ */
