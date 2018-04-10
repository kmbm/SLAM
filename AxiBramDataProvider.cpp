/*
 * AxiBramDataProvider.cpp
 *
 *  Created on: 4 sty 2018
 *      Author: Admin
 */

#include <AxiBramDataProvider.h>

AxiBramDataProvider::AxiBramDataProvider() :
	m_axiBram(std::make_shared<AxiBram>()),
	m_motorController(std::make_shared<MotorController>(m_axiBram)),
	m_direction(FORWARD)
{}
/*
void AxiBramDataProvider::setDirection(RobotDirection p_direction)
{
	m_direction = p_direction;
}*/

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

	readPosition();
}

int AxiBramDataProvider::readPosition()
{
	RobotPosition l_lastDelta = countPositionDelta(m_currentPosition, m_previousPosition);
	usleep(500000);
	m_previousPosition = m_currentPosition;
	m_currentPosition.x = readData(BRAM_POSITION_X_ADDRESS);
	m_currentPosition.y = readData(BRAM_POSITION_Y_ADDRESS);
	usleep(500000);

	if (abs(m_currentPosition.x - m_previousPosition.x) > 10000)
	{
		m_currentPosition.x = m_previousPosition.x + l_lastDelta.x;
		std::cout << "BUG X";
	}
	if (abs(m_currentPosition.y - m_previousPosition.y) > 10000)
	{
		m_currentPosition.y = m_previousPosition.y + l_lastDelta.y;
		std::cout << "BUG Y";

	}
	std::cout << m_currentPosition << std::endl;
}

int AxiBramDataProvider::readData(int p_startAddress)
{
	char l_buffer[4];
	for (int l_it = 0; l_it < 4; ++l_it)
	{
		l_buffer[l_it] = m_axiBram->readData(p_startAddress + l_it);
	}
	std::cout<<(int)l_buffer[0]<<" " <<(int)l_buffer[1]<<" " <<(int)l_buffer[2]<<" " <<(int)l_buffer[3];//<<(int)l_buffer[4]<<" " <<(int)l_buffer[5]<<" " <<(int)l_buffer[6]<<" " <<(int)l_buffer[7]<<std::endl;

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

RobotPosition AxiBramDataProvider::countPositionDelta(const RobotPosition& p_lhs, const RobotPosition& p_rhs)
{
	RobotPosition l_delta;
	l_delta.x = p_lhs.x - p_rhs.y;
	l_delta.y = p_lhs.y - p_rhs.y;

    return l_delta;
}
