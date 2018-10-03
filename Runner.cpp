/*
 * Runner.cpp
 *
 *  Created on: 3 maj 2017
 *      Author: Admin
 */

#include <Runner.h>
#include "AxiBramDataProvider.h"

Runner::Runner(std::shared_ptr<SensorsDataStorage>& p_sensorsDataStorage) :
	m_axiBramDataProvider(std::make_shared<AxiBramDataProvider>()),
	m_gyro(std::make_shared<Gyroscope>(m_axiBramDataProvider, p_sensorsDataStorage)),
	m_lidar(std::make_shared<Lidar>()),
	m_bluetooth(std::make_shared<Bluetooth>()),
	m_motorController(std::make_shared<ManualMotorController>()),
	m_mapGenerator(std::make_shared<MapGenerator>(m_axiBramDataProvider,
												  m_lidar,
												  m_gyro)),
	m_sensorsDataStorage(p_sensorsDataStorage)
{}

std::thread Runner::generateMap()
{
	return std::thread( [this] { m_mapGenerator->run(); });
}

std::thread Runner::lidarScan()
{
	return std::thread( [this] { m_lidar->scan(); });
}

std::thread Runner::radioCommunication()
{
	return std::thread( [this] { this->radioCommunicationWithPC(); });
}

std::thread Runner::gyroscopeThread()
{
	return std::thread( [this] { m_gyro->run(); });
}


void Runner::radioCommunicationWithPC()
{
	char l_command;
	receiveCommand(&l_command);
	std::cout << "command: " << l_command << std::endl;
	m_motorController->sendCommandToPL(&l_command);

	sendLidarMessage();
	sendGyroMessage();

	std::cout << "ready" << std::endl;
}


void Runner::receiveCommand(char* p_command)
{
	m_bluetooth->receive(p_command, 1);
}

void Runner::sendLidarMessage()
{
	/*char l_header = 'l';
	std::string l_msg;
	std::vector<int> l_distanceVector = m_lidar->getDistanceVector();
	for(int i=0; i<8; i++)
	{
		m_bluetooth->send(&l_header,1);
		if (l_distanceVector[i])
			l_msg = std::to_string(l_distanceVector[i]);
		else
			l_msg = "0000";
		std::cout<<l_msg<<std::endl;
		m_bluetooth->send(l_msg.c_str(),4);
	}*/
}

void Runner::sendGyroMessage()
{
	/*m_gyro->update();
	char l_header = 'g';
	std::string l_msg;

	int l_gyroParameters[4];
	l_gyroParameters[0] = m_gyro->getTemperature();
	l_gyroParameters[1] = m_gyro->getXAxisAngle();
	l_gyroParameters[2] = m_gyro->getYAxisAngle();
	l_gyroParameters[3] = m_gyro->getZAxisAngle();

	for(int i=0; i<4; i++)
	{
		m_bluetooth->send(&l_header,1);
		if (l_gyroParameters[i])
			l_msg = std::to_string(l_gyroParameters[i]);
		else
			l_msg = "0000";
		std::cout << "gyro" << l_msg<< std::endl;
		m_bluetooth->send(l_msg.c_str(),4);
	}*/
}
