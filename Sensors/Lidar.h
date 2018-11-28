/*
 * Lidar.h
 *
 *  Created on: 26 lut 2017
 *      Author: Admin
 */

#ifndef LIDAR_H_
#define LIDAR_H_

#include "UartDriver.h"
#include "SensorsDataStorage.h"
#include "Lidar/LidarDataResponse.h"
#include <map>
#include <vector>
#include <memory>
#include <mutex>

class Lidar
{
public:
	Lidar(std::shared_ptr<SensorsDataStorage>);
	~Lidar();

	void stop();
	void reset();
	void scan();
	void getScanResponse();

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

	std::shared_ptr<SensorsDataStorage> m_sensorsDataStorage;
	int m_numOfReadingsPerTurn;
};

#endif /* LIDAR_H_ */
