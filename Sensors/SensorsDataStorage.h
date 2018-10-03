/*
 * SensorsDataStorage.h
 *
 *  Created on: Oct 3, 2018
 *      Author: Krzysiek
 */

#ifndef SENSORS_SENSORSDATASTORAGE_H_
#define SENSORS_SENSORSDATASTORAGE_H_

#include <atomic>

struct RobotPose{
	void setAngle(int p_angle) { angle = p_angle; }

	int x;
	int y ;
	int angle;
};

class SensorsDataStorage {
public:
	SensorsDataStorage();
	RobotPose getRobotPose(){return m_pose;};

private:
	RobotPose m_pose;
};

#endif /* SENSORS_SENSORSDATASTORAGE_H_ */
