/*
 * Gyroscope.cpp
 *
 *  Created on: 19 lut 2017
 *      Author: Admin
 */

#include "Gyroscope.h"

Gyroscope::Gyroscope(const std::shared_ptr<AxiBramDataProvider>& p_axiBramDataProvider,
					 int& p_poseAngle)
	: m_i2cConn("/dev/i2c-1"),
	  m_axiBramDataProvider(p_axiBramDataProvider),
	  m_poseAngle(p_poseAngle)
{
	m_i2cConn.setSlaveAddress(slaveAddress);
	m_i2cConn.writeRegister(0x20,0b00001111);
}

void Gyroscope::readTemperature(){
	m_temperature = m_i2cConn.readRegister(0x26);
}

double Gyroscope::readXAxisAngle(){
	int l_msb, l_lsb;
	l_lsb = m_i2cConn.readRegister(0x28);
	l_msb = m_i2cConn.readRegister(0x29);
	return (l_msb << 8 | l_lsb) *  0.00875;
}

double Gyroscope::readYAxisAngle(){
	int l_msb, l_lsb;
	l_lsb = m_i2cConn.readRegister(0x2A);
	l_msb = m_i2cConn.readRegister(0x2B);
	return (l_msb << 8 | l_lsb) * 0.00875;
}

double Gyroscope::readZAxisAngle(){
	int l_msb, l_lsb;
	l_lsb = m_i2cConn.readRegister(0x2C);
	l_msb = m_i2cConn.readRegister(0x2D);
	return (l_msb << 8 | l_lsb) * 0.00875 *2;
}

void Gyroscope::update(){
	double l_zSum = 0;

	l_zSum += readZAxisAngle() + 0.45;

	if (l_zSum < 1 && l_zSum > -1)
	{
		l_zSum = 0;
	}

	std::chrono::high_resolution_clock::time_point l_currentTime = std::chrono::high_resolution_clock::now();

	auto l_interval = std::chrono::duration_cast<std::chrono::duration<double>>(l_currentTime - m_lastMeasure);
	m_lastMeasure = l_currentTime;

	int l_recordsToRemove = 0;
	for (auto& l_zAxis : m_zAxisCircullarBuffer)
	{
		l_zAxis.first += l_interval;
		if (l_zAxis.first.count() > 1)
		{
			++l_recordsToRemove;
		}
	}

	for (int i=0; i < l_recordsToRemove; ++i)
	{
		m_zAxisSum -= m_zAxisCircullarBuffer.front().second;
		m_zAxisCircullarBuffer.pop_front();
	}
	m_zAxisSum += l_zSum;
	m_zAxisCircullarBuffer.push_back(std::pair<std::chrono::duration<double>, double>(l_interval, l_zSum));

	if (m_zAxisCircullarBuffer.size() != 0)
	{
		m_zAxisAngle = static_cast<double>(m_zAxisSum/m_zAxisCircullarBuffer.size());
	}
	else
	{
		m_zAxisAngle = 0;
	}

	m_lastAngleUpdate += l_interval.count();
	if (m_lastAngleUpdate > 1)
	{
		m_robotOrientation = m_robotOrientation + m_zAxisAngle / m_lastAngleUpdate;
		m_lastAngleUpdate = 0;
	}
	//std::cout << "ANGLE: " << m_robotOrientation << std::endl;
	m_axiBramDataProvider->setRobotOrientation(m_robotOrientation);
	m_poseAngle = m_robotOrientation;
}

void Gyroscope::run()
{
	while(1)
	{
		update();
		usleep(10000);
	}
}

void Gyroscope::periodicUpdate()
{
	update();
	usleep(1000);
}

Gyroscope::~Gyroscope() {
}

