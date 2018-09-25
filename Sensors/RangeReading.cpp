/*
 * RangeReading.cpp
 *
 *  Created on: 21 maj 2018
 *      Author: Admin
 */

#include <limits>
#include <iostream>
#include <assert.h>
#include <sys/types.h>
#include <values.h>
#include "RangeReading.h"

namespace GMapping{

using namespace std;

RangeReading::RangeReading(const RangeSensor* rs, double time)
	: m_rangeSensor(rs)
{}
//	SensorReading(rs,time){}

RangeReading::RangeReading(unsigned int n_beams, const double* d, const RangeSensor* rs, double time)
//	SensorReading(rs,time){
{
	assert(n_beams==rs->beams().size());
	resize(n_beams);
	for (unsigned int i=0; i<size(); i++)
		(*this)[i]=d[i];
}

RangeReading::~RangeReading(){
//	cerr << __PRETTY_FUNCTION__ << ": CAZZZZZZZZZZZZZZZZZZZZOOOOOOOOOOO" << endl;
}

unsigned int RangeReading::rawView(double* v, double density) const{
	if (density==0){
		for (unsigned int i=0; i<size(); i++)
			v[i]=(*this)[i];
	} else {
		Point lastPoint(0,0);
		uint suppressed=0;
		for (unsigned int i=0; i<size(); i++){
			//const RangeSensor* rs=dynamic_cast<const RangeSensor*>(getSensor());
			//assert(rs);
			Point lp(
				cos(m_rangeSensor->beams()[i].pose.theta)*(*this)[i],
				sin(m_rangeSensor->beams()[i].pose.theta)*(*this)[i]);
			Point dp=lastPoint-lp;
			double distance=sqrt(dp*dp);
			if (distance<density){
			  //				v[i]=MAXDOUBLE;
				v[i]=std::numeric_limits<double>::max();
				suppressed++;
			}
			else{
				lastPoint=lp;
				v[i]=(*this)[i];
			}
			//std::cerr<< __PRETTY_FUNCTION__ << std::endl;
			//std::cerr<< "suppressed " << suppressed <<"/"<<size() << std::endl;
		}
	}
	//	return size();
	return static_cast<unsigned int>(size());

};

unsigned int RangeReading::activeBeams(double density) const{
	if (density==0.)
		return size();
		int ab=0;
	Point lastPoint(0,0);
	uint suppressed=0;
	for (unsigned int i=0; i<size(); i++){
		//const RangeSensor* rs=dynamic_cast<const RangeSensor*>(getSensor());
		//assert(rs);
		Point lp(
			cos(m_rangeSensor->beams()[i].pose.theta)*(*this)[i],
			sin(m_rangeSensor->beams()[i].pose.theta)*(*this)[i]);
		Point dp=lastPoint-lp;
		double distance=sqrt(dp*dp);
		if (distance<density){
			suppressed++;
		}
		else{
			lastPoint=lp;
			ab++;
		}
		//std::cerr<< __PRETTY_FUNCTION__ << std::endl;
		//std::cerr<< "suppressed " << suppressed <<"/"<<size() << std::endl;
	}
	return ab;
}

std::vector<Point> RangeReading::cartesianForm(double maxRange) const{
	//const RangeSensor* rangeSensor=dynamic_cast<const RangeSensor*>(getSensor());
	assert(m_rangeSensor && m_rangeSensor->beams().size());
	//	uint m_beams=rangeSensor->beams().size();
	uint m_beams=static_cast<unsigned int>(m_rangeSensor->beams().size());
	std::vector<Point> cartesianPoints(m_beams);
	double px,py,ps,pc;
	px=m_rangeSensor->getPose().x;
	py=m_rangeSensor->getPose().y;
	ps=sin(m_rangeSensor->getPose().theta);
	pc=cos(m_rangeSensor->getPose().theta);
	for (unsigned int i=0; i<m_beams; i++){
		const double& rho=(*this)[i];
		const double& s=m_rangeSensor->beams()[i].s;
		const double& c=m_rangeSensor->beams()[i].c;
		if (rho>=maxRange){
			cartesianPoints[i]=Point(0,0);
		} else {
			Point p=Point(m_rangeSensor->beams()[i].pose.x+c*rho, m_rangeSensor->beams()[i].pose.y+s*rho);
			cartesianPoints[i].x=px+pc*p.x-ps*p.y;
			cartesianPoints[i].y=py+ps*p.x+pc*p.y;
		}
	}
	return cartesianPoints;
}

};

