/*
 * Lidar.h
 *
 *  Created on: 26 lut 2017
 *      Author: Admin
 */

#ifndef LIDAR_H_
#define LIDAR_H_

#include "UartDriver.h"
#include <map>
#include <vector>
#include <memory>
#include <mutex>

enum LidarPosition
{
	LIDAR_POSITION_FRONT = 0,
	LIDAR_POSITION_FRONT_RIGHT,
	LIDAR_POSITION_RIGHT,
	LIDAR_POSITION_BACK_RIGHT,
	LIDAR_POSITION_BACK,
	LIDAR_POSITION_BACK_LEFT,
	LIDAR_POSITION_LEFT,
	LIDAR_POSITION_FRONT_LEFT
};

struct LidarDataResponse
{
	int quality;
	float angle;
	float distance;

	LidarDataResponse()
	{
		quality = 0;
		angle = 0;
		distance = 0;
	}

	LidarDataResponse(char* p_responseBuffer)
	{
		quality = p_responseBuffer[0] >> 2;
		angle = ((p_responseBuffer[2] << 8 | p_responseBuffer[1]) >> 1) >> 6;
		distance = (p_responseBuffer[4] << 8 | p_responseBuffer[3]) >> 2;
	}
};

class Lidar
{
public:
	Lidar();
	~Lidar();

	void stop();
	void reset();
	void scan();
	void getScanResponse();
	std::vector<int> getDistanceVector(){return m_distanceVector;}

private:
	UartDriver uartHost;
	std::mutex m_mutex;
	void sendRequest(unsigned char p_request);
	void fillNeighborhoodScanVector(const LidarDataResponse&);

	static const char startFlag = 0xA5;

	static const char stopReq = 0x25;
	static const char resetReq = 0x40;
	static const unsigned char scanReq = 0x20;
	std::map<LidarPosition, LidarDataResponse> m_neighborhoodScanVector;
	std::vector<int> m_distanceVector;
};

#endif /* LIDAR_H_ */
