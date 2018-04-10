/*
 * MapGenerator.h
 *
 *  Created on: 3 sty 2018
 *      Author: Admin
 */

#ifndef MAPGENERATOR_H_
#define MAPGENERATOR_H_

#include "AxiBramDataProvider.h"
#include "Lidar.h"
#include "Gyroscope.h"
#include "FileHandler.h"

class MapGenerator {
public:
	MapGenerator(std::shared_ptr<AxiBramDataProvider>,
				 std::shared_ptr<Lidar>,
				 std::shared_ptr<Gyroscope>);

	void run();

private:
	std::shared_ptr<AxiBramDataProvider> m_axiBramDataProvider;
	std::shared_ptr<Lidar> m_lidar;
	std::shared_ptr<Gyroscope> m_gyro;
	std::shared_ptr<FileHandler> m_dataFile;



};

#endif /* MAPGENERATOR_H_ */
