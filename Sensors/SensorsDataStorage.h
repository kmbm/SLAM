/*
 * SensorsDataStorage.h
 *
 *  Created on: Oct 3, 2018
 *      Author: Krzysiek
 */

#ifndef SENSORS_SENSORSDATASTORAGE_H_
#define SENSORS_SENSORSDATASTORAGE_H_

#include "Lidar/LidarDataResponse.h"
#include <mutex>
#include <memory>
#include <vector>
#include <map>
#include <iostream>

struct RobotCoordinates
{
	double x;
	double y;
};

inline RobotCoordinates operator-(const RobotCoordinates& p_lhs, const RobotCoordinates& p_rhs)
{
	RobotCoordinates l_result;
	l_result.x = p_lhs.x - p_rhs.x;
	l_result.y = p_lhs.y - p_rhs.y;
	return l_result;
}

inline RobotCoordinates operator/(const RobotCoordinates& p_lhs, double p_value)
{
	RobotCoordinates l_result;
	l_result.x = p_lhs.x / p_value;
	l_result.y = p_lhs.y / p_value;
	return l_result;
}

inline std::ostream& operator<<(std::ostream& os, const RobotCoordinates& data)
{
	std::cout.precision(10);
    os << "Robot Position: x = " << data.x << " y = " << data.y << std::endl;
    return os;
}

struct RobotPose{
	void setAngle(int p_angle)
	{
		m_mutex.lock();
		angle = p_angle;
		m_mutex.unlock();
	}

	void setCoordinates(RobotCoordinates p_robotCoordinates)
	{
		std::cerr << p_robotCoordinates << std::endl;
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

struct LidarScan{
	void init(int);

	void addLidarReading(const LidarDataResponse&);
	std::vector<double> lidarFilteredReadings;

private:
	double countAngleStep(int);
	std::map<int, int> m_angleToIndexMap;
	std::vector<double> m_lidarFilteredReadingsPrep;

	std::mutex m_mutex;
	int m_lastIndex;
	int m_lastAngle;
};



class SensorsDataStorage {
public:
	SensorsDataStorage(int p_beams) : m_pose(std::make_shared<RobotPose>()),
									  m_lidarScan(std::make_shared<LidarScan>()),
									  m_beams(p_beams)
	{
		m_lidarScan->init(m_beams);
	}
	std::shared_ptr<RobotPose> getRobotPose(){return m_pose;}
	std::shared_ptr<LidarScan> getLidarScan(){return m_lidarScan;}
	int getNumberOfBeams(){return m_beams;}

private:
	std::shared_ptr<RobotPose> m_pose;
	std::shared_ptr<LidarScan> m_lidarScan;

	int m_beams;
};

#endif /* SENSORS_SENSORSDATASTORAGE_H_ */
