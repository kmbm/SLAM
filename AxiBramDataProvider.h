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

const int BRAM_POSITION_X_ADDRESS = 8;
const int BRAM_POSITION_Y_ADDRESS = 12;

const int BRAM_Z_AXIS_ANGLE_ADDRESS = 1028;
const int BRAM_SPEED_ADDRESS = 1032;
const int BRAM_ROTATION_ADDRESS = 1036;

const double ENCODER_IMPULSE_TO_METERS = 2400;  //768 res; 36cm

class AxiBramDataProvider {
public:
	AxiBramDataProvider(std::shared_ptr<SensorsDataStorage>&);

	void transferData();

	void setRobotOrientation(double);

private:
	int readData(int p_startAddress);
	void updatePosition();
	RobotCoordinates readPosition();
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
	RobotCoordinates m_initialPosition;
};

#endif /* AXIBRAMDATAPROVIDER_H_ */
