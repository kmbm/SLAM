/*
 * AxiBramDataProvider.cpp
 *
 *  Created on: 4 sty 2018
 *      Author: Admin
 */

#include <AxiBramDataProvider.h>

AxiBramDataProvider::AxiBramDataProvider(std::shared_ptr<SensorsDataStorage>& p_sensorsDataStorage) :
	m_axiBram(std::make_shared<AxiBram>()),
	m_motorController(std::make_shared<MotorController>(m_axiBram)),
	m_direction(FORWARD),
	m_sensorsDataStorage(p_sensorsDataStorage)
{
	m_initialPosition = readPosition();
	std::cerr << "Initial robot position: " << m_initialPosition;
}

void AxiBramDataProvider::setRobotOrientation(double p_zAxisAngle)
{
	m_zAxisAngle = p_zAxisAngle;
}

void AxiBramDataProvider::transferData()
{
	m_motorController->setDirection(m_direction);
	m_robotSpeed = 15;
	m_robotRotation = 3;

	m_axiBram->writeData(static_cast<int>(m_zAxisAngle), BRAM_Z_AXIS_ANGLE_ADDRESS);
	m_axiBram->writeData(static_cast<int>(m_zAxisAngle) >> 8, BRAM_Z_AXIS_ANGLE_ADDRESS + 1);

	m_axiBram->writeData(static_cast<int>(m_robotSpeed), BRAM_SPEED_ADDRESS);

	m_axiBram->writeData(static_cast<int>(m_robotRotation), BRAM_ROTATION_ADDRESS);

	updatePosition();
}

void AxiBramDataProvider::updatePosition()
{
	m_currentPosition = readPosition() - m_initialPosition;
	RobotCoordinates l_currentPositionInMeters = m_currentPosition / ENCODER_IMPULSE_TO_METERS;
	m_mutex.lock();
	m_sensorsDataStorage->getRobotPose()->setCoordinates(l_currentPositionInMeters);
	m_mutex.unlock();

}

RobotCoordinates AxiBramDataProvider::readPosition()
{
	RobotCoordinates l_lastDelta = m_currentPosition - m_previousPosition;
	RobotCoordinates l_currentPosition;
	usleep(500000);
	m_previousPosition = m_currentPosition;
	l_currentPosition.x = readData(BRAM_POSITION_X_ADDRESS);
	l_currentPosition.y = readData(BRAM_POSITION_Y_ADDRESS);
	usleep(500000);

	if (abs(l_currentPosition.x - m_previousPosition.x) > 10000)
	{
		l_currentPosition.x = m_previousPosition.x + l_lastDelta.x;
		std::cout << "BUG X";
	}
	if (abs(l_currentPosition.y - m_previousPosition.y) > 10000)
	{
		l_currentPosition.y = m_previousPosition.y + l_lastDelta.y;
		std::cout << "BUG Y";
	}
	return l_currentPosition;
}

int AxiBramDataProvider::readData(int p_startAddress)
{
	char l_buffer[4];
	for (int l_it = 0; l_it < 4; ++l_it)
	{
		l_buffer[l_it] = m_axiBram->readData(p_startAddress + l_it);
	}
	return buffToInteger(l_buffer);
}

int AxiBramDataProvider::buffToInteger(char * p_buffer)
{
	int l_result = static_cast<int>(static_cast<unsigned char>(p_buffer[3]) << 24 |
    			   static_cast<unsigned char>(p_buffer[2]) << 16 |
				   static_cast<unsigned char>(p_buffer[1]) << 8 |
				   static_cast<unsigned char>(p_buffer[0]));

    return l_result;
}
