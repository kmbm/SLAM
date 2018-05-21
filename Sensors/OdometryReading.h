/*
 * OdometryReading.h
 *
 *  Created on: 21 maj 2018
 *      Author: Admin
 */

#ifndef SENSORS_ODOMETRYREADING_H_
#define SENSORS_ODOMETRYREADING_H_

#include <string.h>
#include <SensorReading.h>
#include <Utils/Point.h>
#include "OdometrySensor.h"

namespace GMapping{

class OdometryReading: public SensorReading{
	public:
		OdometryReading(const OdometrySensor* odo, double time=0);
		inline const OrientedPoint& getPose() const {return m_pose;}
		inline const OrientedPoint& getSpeed() const {return m_speed;}
		inline const OrientedPoint& getAcceleration() const {return m_acceleration;}
		inline void setPose(const OrientedPoint& pose) {m_pose=pose;}
		inline void setSpeed(const OrientedPoint& speed) {m_speed=speed;}
		inline void setAcceleration(const OrientedPoint& acceleration) {m_acceleration=acceleration;}

	protected:
		OrientedPoint m_pose;
		OrientedPoint m_speed;
		OrientedPoint m_acceleration;
};

};

#endif /* SENSORS_ODOMETRYREADING_H_ */
