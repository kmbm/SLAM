/*
 * RangeReading.h
 *
 *  Created on: 21 maj 2018
 *      Author: Admin
 */

#ifndef SENSORS_RANGEREADING_H_
#define SENSORS_RANGEREADING_H_

#include <vector>
#include "rangesensor.h"

namespace GMapping{

using ScanReading = std::vector<double>;

class RangeReading /*public SensorReading,*/// public std::vector<double>{
{
	public:
		RangeReading(const RangeSensor* rs, double time=0);
		//RangeReading(unsigned int n_beams, const double* d, const RangeSensor* rs, double time=0);
		virtual ~RangeReading();
		inline const OrientedPoint& getPose() const {return m_pose;}
		inline void setPose(const OrientedPoint& pose) {m_pose=pose;}
		//unsigned int rawView(double* v, double density=0.) const;
		std::vector<Point> cartesianForm(double maxRange=1e6) const;
		unsigned int activeBeams(double density=0.) const;
		ScanReading getScanReading() const {return m_scanReading;}
		int getScanReadingSize() const {return m_scanReading.size();}
	protected:
		OrientedPoint m_pose;
		const RangeSensor* m_rangeSensor;
		ScanReading m_scanReading;
};

};

#endif /* SENSORS_RANGEREADING_H_ */
