/*
 * RobotPositionProvider.h
 *
 *  Created on: 3 sty 2018
 *      Author: Admin
 */

#ifndef ROBOTPOSITIONPROVIDER_H_
#define ROBOTPOSITIONPROVIDER_H_

struct RobotPosition
{
	double x;
	double y;
};

class RobotPositionProvider {
public:
	RobotPositionProvider();

	RobotPosition getPosition(){ return m_position; };

private:
	RobotPosition m_position;
};

#endif /* ROBOTPOSITIONPROVIDER_H_ */
