/*
 * MapLogger.h
 *
 *  Created on: Oct 28, 2018
 *      Author: Krzysiek
 */

#ifndef MAPLOGGER_H_
#define MAPLOGGER_H_

#include "FileHandler.h"
#include "ScanMatching/PointAccumulator.h"
#include <memory>

using namespace GMapping;

class MapLogger {
public:
	MapLogger();
	void saveMap(GMapping::ScanMatcherMap&);
private:
	std::unique_ptr<FileHandler> m_mapFileHandler;
};

#endif /* MAPLOGGER_H_ */
