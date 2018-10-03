/*
 * AxiBramDataProvider.h
 *
 *  Created on: 4 sty 2018
 *      Author: Admin
 */

#ifndef AXIBRAMDATAPROVIDER_H_
#define AXIBRAMDATAPROVIDER_H_

#include "AxiBram.h"
#include "MotorController.h"
#include "SensorsDataStorage.h"
#include <memory>
#include <atomic>
#include <mutex>
#include <iostream>



inline std::ostream& operator<<(std::ostream& os, const RobotCoordinates& data)
{
	std::cout.precision(10);
    os << "Robot Position: x = " << data.x << " y = " << data.y << std::endl;
    return os;
}

const int BRAM_POSITION_X_ADDRESS = 8;
const int BRAM_POSITION_Y_ADDRESS = 12;

const int BRAM_Z_AXIS_ANGLE_ADDRESS = 1028;
const int BRAM_SPEED_ADDRESS = 1032;
const int BRAM_ROTATION_ADDRESS = 1036;

class AxiBramDataProvider {
public:
	AxiBramDataProvider(std::shared_ptr<SensorsDataStorage>&);

	void transferData();

	//void setDirection(RobotDirection p_direction);
	void setRobotOrientation(double);
	RobotCoordinates getPosition(){ return m_currentPosition; }

private:
	RobotCoordinates countPositionDelta(const RobotCoordinates&, const RobotCoordinates&);
	int readData(int p_startAddress);
	int readPosition();
	int buffToInteger(char * p_buffer);
	std::atomic<double> m_zAxisAngle;

	std::shared_ptr<AxiBram> m_axiBram;
	std::shared_ptr<MotorController> m_motorController;
	std::shared_ptr<SensorsDataStorage>& m_sensorsDataStorage;
	std::mutex m_mutex;

	RobotDirection m_direction;
	int m_robotSpeed;
	int m_robotRotation;
	RobotCoordinates m_currentPosition;
	RobotCoordinates m_previousPosition;
};

#endif /* AXIBRAMDATAPROVIDER_H_ */
