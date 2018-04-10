/*
 * Lidar.cpp
 *
 *  Created on: 26 lut 2017
 *      Author: Admin
 */

#include <Lidar.h>

Lidar::Lidar():uartHost("/dev/ttyUL2") {
	m_distanceVector.resize(10);
	//stop();
	//usleep(1000);
}

Lidar::~Lidar() {
}

void Lidar::sendRequest(unsigned char p_request){
	char l_buffer[2] = {startFlag, p_request};
	uartHost.sendData(l_buffer, 2);
}

void Lidar::stop(){
	sendRequest(stopReq);
}

void Lidar::reset(){
	sendRequest(resetReq);
}

void Lidar::scan(){
	char l_frameBuffer[5];

	stop();
	usleep(10000);

	sendRequest(scanReq);
	getScanResponse();

	int i=0, j=0;

	while(1)
	{
		i++;

		uartHost.readData(l_frameBuffer,5);
		LidarDataResponse l_lidarDataResponse(l_frameBuffer);

		fillNeighborhoodScanVector(l_lidarDataResponse);

		if(i == 100)
		{
			j++;
			i=0;
		}
		if (j == 10)
		{
			j=0;
			for (const auto& l_distance : m_neighborhoodScanVector)
			{
				std::cerr << l_distance.second.quality << "   " << l_distance.second.angle << "   " << l_distance.second.distance <<std::endl;
			}
			std::cerr << "DUPA" << std::endl;

		}
		//std::cerr << l_quality << "   " << l_angle << "   " << l_distance <<std::endl;
		//std::cerr << "DUPA";
	}
	//stop();

}

void Lidar::fillNeighborhoodScanVector(const LidarDataResponse& p_lidarDataResponse)
{
	m_mutex.lock();

	if (p_lidarDataResponse.angle >= 359 || p_lidarDataResponse.angle <= 2)
	{
		m_neighborhoodScanVector[LIDAR_POSITION_FRONT] = p_lidarDataResponse;
	}
	if (p_lidarDataResponse.angle >= 44 && p_lidarDataResponse.angle <= 47)
	{
		m_neighborhoodScanVector[LIDAR_POSITION_FRONT_RIGHT] = p_lidarDataResponse;
	}
	if (p_lidarDataResponse.angle >= 89 && p_lidarDataResponse.angle <= 92)
	{
		m_neighborhoodScanVector[LIDAR_POSITION_RIGHT] = p_lidarDataResponse;
	}
	if (p_lidarDataResponse.angle >= 133 && p_lidarDataResponse.angle <= 137)
	{
		m_neighborhoodScanVector[LIDAR_POSITION_BACK_RIGHT] = p_lidarDataResponse;
	}
	if (p_lidarDataResponse.angle >= 179 && p_lidarDataResponse.angle <= 182)
	{
		m_neighborhoodScanVector[LIDAR_POSITION_BACK] = p_lidarDataResponse;
	}
	if (p_lidarDataResponse.angle >= 224 && p_lidarDataResponse.angle <= 227)
	{
		m_neighborhoodScanVector[LIDAR_POSITION_BACK_LEFT] = p_lidarDataResponse;
	}
	if (p_lidarDataResponse.angle >= 269 && p_lidarDataResponse.angle <= 272)
	{
		m_neighborhoodScanVector[LIDAR_POSITION_LEFT] = p_lidarDataResponse;
	}
	if (p_lidarDataResponse.angle >= 314 && p_lidarDataResponse.angle <= 317)
	{
		m_neighborhoodScanVector[LIDAR_POSITION_FRONT_LEFT] = p_lidarDataResponse;
	}

	m_mutex.unlock();
}

void Lidar::getScanResponse()
{
	char l_responseBuffer[2];

	while(1)
	{
		uartHost.readData(l_responseBuffer,2);

		if(l_responseBuffer[0] == 0xA5 && l_responseBuffer[1] == 0x5A)
		{
			std::cout << "Start scanning..." << std::endl;
			return;
		}
	}
}
