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

void MapLogger::saveMap(GMapping::HierarchicalArray2D<GMapping::PointAccumulator>& p_map, int p_xSize, int p_ySize)
{
	for (int i = 0; i < p_ySize; ++i)
	{
		for (int j = 0; j < p_xSize; ++j)
		{
			m_mapFileHandler->writeData(p_map.cellState(i,j));
		}
		m_mapFileHandler->newLine();
	}
}
