/*
 * ScanMatcher.h
 *
 *  Created on: 20 maj 2018
 *      Author: Admin
 */

#ifndef SCANMATCHING_SCANMATCHER_H_
#define SCANMATCHING_SCANMATCHER_H_

#include "PointAccumulator.h"
#include <Utils/Stat.h>
#include <iostream>
#include <vector>

#define LASER_MAXBEAMS 1024

namespace GMapping {

class ScanMatcher{
	public:
		typedef Covariance3 CovarianceMatrix;

		ScanMatcher();
		double optimize(OrientedPoint& pnew, const ScanMatcherMap& map, const OrientedPoint& p, const std::vector<double> readings) const;

		double registerScan(ScanMatcherMap& map, const OrientedPoint& p, const std::vector<double> readings);
		void setLaserParameters
			(unsigned int beams, double* angles, const OrientedPoint& lpose);
		void setMatchingParameters
			(double urange, double range, double sigma, int kernsize, double lopt, double aopt, int iterations, double likelihoodSigma=1, unsigned int likelihoodSkip=0 );
		void invalidateActiveArea();
		void computeActiveArea(ScanMatcherMap& map, const OrientedPoint& p, const std::vector<double> readings);

		double score(const ScanMatcherMap& map, const OrientedPoint& p, const std::vector<double> readings) const;
		unsigned int likelihoodAndScore(double& s, double& l, const ScanMatcherMap& map, const OrientedPoint& p, const std::vector<double> readings) const;

		static const double nullLikelihood;
	protected:
		//state of the matcher
		bool m_activeAreaComputed;

		/**laser parameters*/
		unsigned int m_laserBeams;
		double       m_laserAngles[LASER_MAXBEAMS];
		OrientedPoint m_laserPose;
		double m_laserMaxRange;
		/**scan_matcher parameters*/
		double m_usableRange;
		double m_gaussianSigma;
		double m_likelihoodSigma;
		int m_kernelSize;
		double m_optAngularDelta;
		double m_optLinearDelta;
		unsigned int m_optRecursiveIterations;
		unsigned int m_likelihoodSkip;
		double m_llsamplerange;
		double m_llsamplestep;
		double m_lasamplerange;
		double m_lasamplestep;
		bool m_generateMap = true;
		double m_enlargeStep;
		double m_fullnessThreshold;
		double m_angularOdometryReliability;
		double m_linearOdometryReliability;
		double m_freeCellRatio;
		unsigned int m_initialBeamsSkip;
};

};

#endif /* SCANMATCHING_SCANMATCHER_H_ */
