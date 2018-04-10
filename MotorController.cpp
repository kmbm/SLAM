/*
 * MotorController.cpp
 *
 *  Created on: 14 gru 2017
 *      Author: Admin
 */

#include <MotorController.h>

MotorController::MotorController(std::shared_ptr<AxiBram> p_axiBram) :
	m_axiBram(std::move(p_axiBram))
{}

void MotorController::setDirection(RobotDirection p_direction)
{
	m_axiBram->writeData(p_direction, m_directionBramAddress);
}
