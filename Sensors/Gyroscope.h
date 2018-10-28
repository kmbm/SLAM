/*
 * Gyroscope.h
 *
 *  Created on: 19 lut 2017
 *      Author: Admin
 */

#ifndef GYROSCOPE_H_
#define GYROSCOPE_H_

#include "AxiBramDataProvider.h"
#include "SensorsDataStorage.h"
#include "I2Cmaster.h"
#include <atomic>
#include <mutex>
#include <list>
#include <chrono>


class Gyroscope {
public:
	Gyroscope(const std::shared_ptr<AxiBramDataProvider>&, std::shared_ptr<SensorsDataStorage>);
	~Gyroscope();

	void run();
	void update();
	void periodicUpdate();
	int getTemperature(){return m_temperature;}
	double getXAxisAngle(){return m_xAxisAngle;}
	double getYAxisAngle(){return m_yAxisAngle;}
	double getZAxisAngle(){return m_robotOrientation;}

private:
	std::shared_ptr<AxiBramDataProvider> m_axiBramDataProvider;
	std::atomic<int> m_temperature;
	std::atomic<double> m_xAxisAngle;
	std::atomic<double> m_yAxisAngle;
	std::atomic<double> m_zAxisAngle;

	std::shared_ptr<SensorsDataStorage> m_sensorsDataStorage;

	std::atomic<double> m_robotOrientation;
	std::mutex m_mutex;

	std::list<std::pair<std::chrono::duration<double>, double>> m_zAxisCircullarBuffer;
	double m_zAxisSum = 0;

	std::chrono::high_resolution_clock::time_point m_lastMeasure;
	double m_lastAngleUpdate;

	void readTemperature();
	double readXAxisAngle();
	double readYAxisAngle();
	double readZAxisAngle();

	I2Cmaster m_i2cConn;

	static const int slaveAddress = 105;
};

#endif /* GYROSCOPE_H_ */
