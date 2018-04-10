/*
 * MotorController.h
 *
 *  Created on: 14 gru 2017
 *      Author: Admin
 */

#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#include <memory>
#include "AxiBram.h"

enum RobotDirection
{
	STOP,
	FORWARD,
	BACK
};

class MotorController
{
public:
	MotorController(std::shared_ptr<AxiBram>);
	void setDirection(RobotDirection);

private:
	std::shared_ptr<AxiBram> m_axiBram;
	const int m_directionBramAddress = 1024;

};

#endif /* MOTORCONTROLLER_H_ */
