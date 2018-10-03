/*
 * Runner.h
 *
 *  Created on: 3 maj 2017
 *      Author: Admin
 */

#ifndef RUNNER_H_
#define RUNNER_H_

#include "Gyroscope.h"
#include "Lidar.h"
#include "ManualMotorController.h"
#include "MapGenerator.h"
#include <thread>
#include <memory>

class Runner {
public:
	Runner(std::shared_ptr<SensorsDataStorage>&);
	/*Runner(const std::shared_ptr<Gyroscope>&,
   		   const std::shared_ptr<Lidar>&,
		   const std::shared_ptr<Bluetooth>&,
		   const std::shared_ptr<ManualMotorController>&,
		   int&);*/
	std::thread lidarScan();
	std::thread generateMap();
	std::thread radioCommunication();
	std::thread gyroscopeThread();

private:
	void radioCommunicationWithPC();
	void receiveCommand(char* p_command);
	void sendLidarMessage();
	void sendGyroMessage();

	std::shared_ptr<AxiBramDataProvider> m_axiBramDataProvider;
	std::shared_ptr<Gyroscope> m_gyro;
	std::shared_ptr<Lidar> m_lidar;
	std::shared_ptr<Bluetooth> m_bluetooth;
	std::shared_ptr<ManualMotorController> m_motorController;
	std::shared_ptr<MapGenerator> m_mapGenerator;

	std::shared_ptr<SensorsDataStorage>& m_sensorsDataStorage;
};

#endif /* RUNNER_H_ */
