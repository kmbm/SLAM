/*
 * MapGenerator.cpp
 *
 *  Created on: 3 sty 2018
 *      Author: Admin
 */

#include <MapGenerator.h>

MapGenerator::MapGenerator(std::shared_ptr<AxiBramDataProvider> p_axiBramDataProvider,
			 	 	 	   std::shared_ptr<Lidar> p_lidar,
						   std::shared_ptr<Gyroscope> p_gyro)
	: m_axiBramDataProvider(std::move(p_axiBramDataProvider)),
	  m_lidar(std::move(p_lidar)),
	  m_gyro(std::move(p_gyro)),
	  m_dataFile(std::make_shared<FileHandler>("log.txt"))
{}

void MapGenerator::run()
{
	while(1)
	{
		m_axiBramDataProvider->transferData();
		RobotPosition l_position = m_axiBramDataProvider->getPosition();
		//std::cout << l_position;
		//std::cout << "gyro: " << m_gyro->getTemperature() << " " << m_gyro->getXAxisAngle() << " "
			//	  << m_gyro->getYAxisAngle() << " " << m_gyro->getZAxisAngle() << std::endl;
		m_dataFile->writeData(m_gyro->getZAxisAngle());
		//std::cout << m_gyro->getZAxisAngle() << std::endl;
	}
}
