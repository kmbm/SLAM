/*
 * SensorsDataStorage.cpp
 *
 *  Created on: Oct 3, 2018
 *      Author: Krzysiek
 */

#include <SensorsDataStorage.h>

#define ANGLE_TOLERANCE 5

namespace{
int absAngleDiff(int p_lhs, int p_rhs)
{
	auto diff = abs(p_lhs - p_rhs);
	return (diff > 180) ? abs(360 - diff) : diff;
}

void printScan(std::vector<double> p_scanVector)
{
	std::cout << "SensorsDataStorage: ";
	for (auto it : p_scanVector)
	{
		std::cout << it << " ";
	}
	std::cout << std::endl;
}

}
void LidarScan::init(int p_beams)
{
	auto angleStep = countAngleStep(p_beams);
	m_lastIndex = 0;
	lidarFilteredReadings.resize(p_beams);
	m_lidarFilteredReadingsPrep.resize(p_beams);

	double angle = 0;

	for (int it = 0; it < p_beams; ++it)
	{
		angle += angleStep;
		m_angleToIndexMap.insert(std::pair<int, int>(it, static_cast<int>(angle)));
	}

}

double LidarScan::countAngleStep(int p_beams)
{
	return 360.0 / p_beams;
}

void LidarScan::addLidarReading(const LidarDataResponse& p_lidarDataResponse)
{
	auto angle = m_angleToIndexMap[m_lastIndex];

	if ((p_lidarDataResponse.angle > (angle - ANGLE_TOLERANCE)) and (p_lidarDataResponse.angle < (angle + ANGLE_TOLERANCE)))
	{
		if (absAngleDiff(angle, p_lidarDataResponse.angle) < absAngleDiff(m_lastAngle, p_lidarDataResponse.angle))
		{
			m_lidarFilteredReadingsPrep[m_lastIndex] = p_lidarDataResponse.distance;
			m_lastAngle = p_lidarDataResponse.angle;
		}
		else
		{
			m_lastIndex++;
		}
	}

	if (m_lastIndex == m_lidarFilteredReadingsPrep.size())
	{
		m_lastIndex = 0;
		m_lastAngle = 500;
		m_mutex.lock();
		lidarFilteredReadings = m_lidarFilteredReadingsPrep;
		m_mutex.unlock();
		printScan(lidarFilteredReadings);
	}
}
