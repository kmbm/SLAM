/*
 * OdometryReading.cpp
 *
 *  Created on: 21 maj 2018
 *      Author: Admin
 */

#include <OdometryReading.h>

namespace GMapping{

OdometryReading::OdometryReading(const OdometrySensor* odo, double time):
	SensorReading(odo,time){}

};
