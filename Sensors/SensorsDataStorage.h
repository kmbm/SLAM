/*
 * SensorsDataStorage.h
 *
 *  Created on: Oct 3, 2018
 *      Author: Krzysiek
 */

#ifndef SENSORS_SENSORSDATASTORAGE_H_
#define SENSORS_SENSORSDATASTORAGE_H_

#include <mutex>
#include <memory>
#include <iostream>

struct RobotCoordinates
{
	double x;
	double y;
};

struct RobotPose{
	void setAngle(int p_angle)
	{
		m_mutex.lock();
		angle = p_angle;
		m_mutex.unlock();
	}
	void setCoordinates(RobotCoordinates p_robotCoordinates)
	{
		m_mutex.lock();
		x = p_robotCoordinates.x;
		y = p_robotCoordinates.y;
		m_mutex.unlock();
	}

	double x;
	double y ;
	int angle;

	std::mutex m_mutex;
};

class SensorsDataStorage {
public:
	SensorsDataStorage() : m_pose(std::make_shared<RobotPose>()) {};
	std::shared_ptr<RobotPose> getRobotPose(){return m_pose;};

private:
	std::shared_ptr<RobotPose> m_pose;
};

#endif /* SENSORS_SENSORSDATASTORAGE_H_ */
