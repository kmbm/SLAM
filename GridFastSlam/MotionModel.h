/*
 * MotionModel.h
 *
 *  Created on: 21 maj 2018
 *      Author: Admin
 */

#ifndef GRIDFASTSLAM_MOTIONMODEL_H_
#define GRIDFASTSLAM_MOTIONMODEL_H_

#include <Utils/Point.h>
#include <Utils/Stat.h>

namespace  GMapping {

struct MotionModel{
	OrientedPoint drawFromMotion(const OrientedPoint& p, const OrientedPoint& pnew, const OrientedPoint& pold) const;
	double srr, str, srt, stt;
};

};

#endif /* GRIDFASTSLAM_MOTIONMODEL_H_ */
