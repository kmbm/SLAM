/*
 * Sensor.h
 *
 *  Created on: 21 maj 2018
 *      Author: Admin
 */

#ifndef SENSORS_SENSOR_H_
#define SENSORS_SENSOR_H_

#include <string>
#include <map>

namespace GMapping{

class Sensor{
	public:
		Sensor(const std::string& name="");
		virtual ~Sensor();
		inline std::string getName() const {return m_name;}
		inline void setName(const std::string& name) {m_name=name;}
	protected:
		std::string m_name;
};

typedef std::map<std::string, Sensor*> SensorMap;

};

#endif /* SENSORS_SENSOR_H_ */
