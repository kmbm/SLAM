/*
 * CMPS.cpp
 *
 *  Created on: 19 lut 2017
 *      Author: Admin
 */

#include <CMPS.h>

CMPS::CMPS() : m_i2cConn("/dev/i2c-1") {
	m_i2cConn.setSlaveAddress(30);
	usleep(200);
	m_i2cConn.writeRegister(0x00, 0x70);
	m_i2cConn.writeRegister(0x01, 0xA0);
	m_i2cConn.writeRegister(0x02, 0x01);
	//usleep(10);
}

int CMPS::readXAxis(){
	int l_msb, l_lsb;
	l_msb = m_i2cConn.readRegister(3);
	l_lsb = m_i2cConn.readRegister(4);
	return (l_msb << 8 | l_lsb);
}
int CMPS::readYAxis(){
	int l_msb, l_lsb;
	l_msb = m_i2cConn.readRegister(7);
	l_lsb = m_i2cConn.readRegister(8);
	return (l_msb << 8 | l_lsb);
}

int CMPS::readZAxis(){
	int l_msb, l_lsb;
	l_msb = m_i2cConn.readRegister(5);
	l_lsb = m_i2cConn.readRegister(6);
	return (l_msb << 8 | l_lsb);
}

void CMPS::update(){
	m_xAxis = readXAxis();
	m_yAxis = readYAxis();
	m_zAxis = readZAxis();
}

CMPS::~CMPS() {
	// TODO Auto-generated destructor stub
}

