/*
 * MapLogger.cpp
 *
 *  Created on: Oct 28, 2018
 *      Author: Krzysiek
 */

#include <MapLogger.h>

MapLogger::MapLogger() :
	m_mapFileHandler(std::make_unique<FileHandler>("map.txt"))
{}

void MapLogger::saveMap(GMapping::ScanMatcherMap& p_map)
{
	for (int i = 0; i < p_map.getMapSizeX(); ++i)
	{
		for (int j = 0; j < p_map.getMapSizeY(); ++j)
		{
			m_mapFileHandler->writeData(p_map.cell(i,j));
			if (p_map.cell(i,j) != -1)
				std::cerr<< "DUPA" << p_map.cell(i,j);
		}
		m_mapFileHandler->newLine();
	}
}
