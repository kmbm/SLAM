/*
 * PointAccumulator.cpp
 *
 *  Created on: 20 maj 2018
 *      Author: Admin
 */

#include <ScanMatching/PointAccumulator.h>

namespace GMapping {

const PointAccumulator& PointAccumulator::Unknown(){
	if (! unknown_ptr)
		unknown_ptr=new PointAccumulator;
	return *unknown_ptr;
}

PointAccumulator* PointAccumulator::unknown_ptr=0;

};