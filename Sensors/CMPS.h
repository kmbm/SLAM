/*
 * CMPS.h
 *
 *  Created on: 19 lut 2017
 *      Author: Admin
 */

#ifndef CMPS_H_
#define CMPS_H_

#include "I2Cmaster.h"

class CMPS {
public:
	CMPS();
	~CMPS();
	void update();

	int getXAxis(){return m_xAxis;}
	int getYAxis(){return m_yAxis;}
	int getZAxis(){return m_zAxis;}

private:
	int m_xAxis;
	int m_yAxis;
	int m_zAxis;

	int readXAxis();
	int readYAxis();
	int readZAxis();


	I2Cmaster m_i2cConn;

	static const int slaveAddress = 30;
};

#endif /* CMPS_H_ */
