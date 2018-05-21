/*
 * OdometrySensor.h
 *
 *  Created on: 21 maj 2018
 *      Author: Admin
 */

#ifndef SENSORS_ODOMETRYSENSOR_H_
#define SENSORS_ODOMETRYSENSOR_H_

#include <string>
#include <Sensor.h>

namespace GMapping{

class OdometrySensor: public Sensor{
	public:
		OdometrySensor(const std::string& name, bool ideal=false);
		inline bool isIdeal() const { return m_ideal; }
	protected:
		bool m_ideal;
};

};
#endif /* SENSORS_ODOMETRYSENSOR_H_ */
