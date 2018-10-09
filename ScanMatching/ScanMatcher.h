/*
 * ScanMatcher.h
 *
 *  Created on: 20 maj 2018
 *      Author: Admin
 */

#ifndef SCANMATCHING_SCANMATCHER_H_
#define SCANMATCHING_SCANMATCHER_H_

#include "IcpOptimization.h"
#include "PointAccumulator.h"
//#include <utils/macro_params.h>
#include <Utils/Stat.h>
#include <iostream>
#include <vector>

#define LASER_MAXBEAMS 1024

namespace GMapping {

class ScanMatcher{
	public:
		typedef Covariance3 CovarianceMatrix;

		ScanMatcher();
		double icpOptimize(OrientedPoint& pnew, const ScanMatcherMap& map, const OrientedPoint& p, const std::vector<double> readings) const;
		double optimize(OrientedPoint& pnew, const ScanMatcherMap& map, const OrientedPoint& p, const std::vector<double> readings) const;
		double optimize(OrientedPoint& mean, CovarianceMatrix& cov, const ScanMatcherMap& map, const OrientedPoint& p, const std::vector<double> readings) const;

		double registerScan(ScanMatcherMap& map, const OrientedPoint& p, const std::vector<double> readings);
		void setLaserParameters
			(unsigned int beams, double* angles, const OrientedPoint& lpose);
		void setMatchingParameters
			(double urange, double range, double sigma, int kernsize, double lopt, double aopt, int iterations, double likelihoodSigma=1, unsigned int likelihoodSkip=0 );
		void invalidateActiveArea();
		void computeActiveArea(ScanMatcherMap& map, const OrientedPoint& p, const std::vector<double> readings);

		inline double icpStep(OrientedPoint & pret, const ScanMatcherMap& map, const OrientedPoint& p, const std::vector<double> readings) const;
		inline double score(const ScanMatcherMap& map, const OrientedPoint& p, const std::vector<double> readings) const;
		inline unsigned int likelihoodAndScore(double& s, double& l, const ScanMatcherMap& map, const OrientedPoint& p, const std::vector<double> readings) const;
		double likelihood(double& lmax, OrientedPoint& mean, CovarianceMatrix& cov, const ScanMatcherMap& map, const OrientedPoint& p, const std::vector<double> readings);
		double likelihood(double& _lmax, OrientedPoint& _mean, CovarianceMatrix& _cov, const ScanMatcherMap& map, const OrientedPoint& p, Gaussian3& odometry, const std::vector<double> readings, double gain=180.);
		inline const double* laserAngles() const { return m_laserAngles; }
		inline unsigned int laserBeams() const { return m_laserBeams; }

		static const double nullLikelihood;
	protected:
		//state of the matcher
		bool m_activeAreaComputed;

		/**laser parameters*/
		unsigned int m_laserBeams;
		double       m_laserAngles[LASER_MAXBEAMS];
		//OrientedPoint m_laserPose;

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
		bool m_generateMap;
		double m_enlargeStep;
		double m_fullnessThreshold;
		double m_angularOdometryReliability;
		double m_linearOdometryReliability;
		double m_freeCellRatio;
		unsigned int m_initialBeamsSkip;
};

inline double ScanMatcher::icpStep(OrientedPoint & pret, const ScanMatcherMap& map, const OrientedPoint& p, const std::vector<double> readings) const{
	const double * angle=m_laserAngles+m_initialBeamsSkip;
	OrientedPoint lp=p;
	lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
	lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
	lp.theta+=m_laserPose.theta;
	unsigned int skip=0;
	double freeDelta=map.getDelta()*m_freeCellRatio;
	std::list<PointPair> pairs;

	//for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++){
	for (auto r = readings.begin() + m_initialBeamsSkip; r < readings.end(); r++, angle++){
		skip++;
		skip=skip>m_likelihoodSkip?0:skip;
		if (*r>m_usableRange) continue;
		if (skip) continue;
		Point phit=lp;
		phit.x+=*r*cos(lp.theta+*angle);
		phit.y+=*r*sin(lp.theta+*angle);
		IntPoint iphit=map.world2map(phit);
		Point pfree=lp;
		pfree.x+=(*r-map.getDelta()*freeDelta)*cos(lp.theta+*angle);
		pfree.y+=(*r-map.getDelta()*freeDelta)*sin(lp.theta+*angle);
 		pfree=pfree-phit;
		IntPoint ipfree=map.world2map(pfree);
		bool found=false;
		Point bestMu(0.,0.);
		Point bestCell(0.,0.);
		for (int xx=-m_kernelSize; xx<=m_kernelSize; xx++)
		for (int yy=-m_kernelSize; yy<=m_kernelSize; yy++){
			IntPoint pr=iphit+IntPoint(xx,yy);
			IntPoint pf=pr+ipfree;
			//AccessibilityState s=map.storage().cellState(pr);
			//if (s&Inside && s&Allocated){
				const PointAccumulator& cell=map.cell(pr);
				const PointAccumulator& fcell=map.cell(pf);
				if (((double)cell )> m_fullnessThreshold && ((double)fcell )<m_fullnessThreshold){
					Point mu=phit-cell.mean();
					if (!found){
						bestMu=mu;
						bestCell=cell.mean();
						found=true;
					}else
						if((mu*mu)<(bestMu*bestMu)){
							bestMu=mu;
							bestCell=cell.mean();
						}

				}
			//}
		}
		if (found){
			pairs.push_back(std::make_pair(phit, bestCell));
			//std::cerr << "(" << phit.x-bestCell.x << "," << phit.y-bestCell.y << ") ";
		}
		//std::cerr << std::endl;
	}

	OrientedPoint result(0,0,0);
	//double icpError=icpNonlinearStep(result,pairs);
	std::cerr << "result(" << pairs.size() << ")=" << result.x << " " << result.y << " " << result.theta << std::endl;
	pret.x=p.x+result.x;
	pret.y=p.y+result.y;
	pret.theta=p.theta+result.theta;
	pret.theta=atan2(sin(pret.theta), cos(pret.theta));
	return score(map, p, readings);
}

inline double ScanMatcher::score(const ScanMatcherMap& map, const OrientedPoint& p, const std::vector<double> readings) const{
	double s=0;
	const double * angle=m_laserAngles+m_initialBeamsSkip;
	OrientedPoint lp=p;
	lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
	lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
	lp.theta+=m_laserPose.theta;
	unsigned int skip=0;
	double freeDelta=map.getDelta()*m_freeCellRatio;
	//for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++){
	for (auto r = readings.begin() + m_initialBeamsSkip; r < readings.end(); r++, angle++){
		skip++;
		skip=skip>m_likelihoodSkip?0:skip;
		if (*r>m_usableRange) continue;
		if (skip) continue;
		Point phit=lp;
		phit.x+=*r*cos(lp.theta+*angle);
		phit.y+=*r*sin(lp.theta+*angle);
		IntPoint iphit=map.world2map(phit);
		Point pfree=lp;
		pfree.x+=(*r-map.getDelta()*freeDelta)*cos(lp.theta+*angle);
		pfree.y+=(*r-map.getDelta()*freeDelta)*sin(lp.theta+*angle);
 		pfree=pfree-phit;
		IntPoint ipfree=map.world2map(pfree);
		bool found=false;
		Point bestMu(0.,0.);
		for (int xx=-m_kernelSize; xx<=m_kernelSize; xx++)
		for (int yy=-m_kernelSize; yy<=m_kernelSize; yy++){
			IntPoint pr=iphit+IntPoint(xx,yy);
			IntPoint pf=pr+ipfree;
			//AccessibilityState s=map.storage().cellState(pr);
			//if (s&Inside && s&Allocated){
				const PointAccumulator& cell=map.cell(pr);
				const PointAccumulator& fcell=map.cell(pf);
				if (((double)cell )> m_fullnessThreshold && ((double)fcell )<m_fullnessThreshold){
					Point mu=phit-cell.mean();
					if (!found){
						bestMu=mu;
						found=true;
					}else
						bestMu=(mu*mu)<(bestMu*bestMu)?mu:bestMu;
				}
			//}
		}
		if (found)
			s+=exp(-1./m_gaussianSigma*bestMu*bestMu);
	}
	return s;
}

inline unsigned int ScanMatcher::likelihoodAndScore(double& s, double& l, const ScanMatcherMap& map, const OrientedPoint& p, const std::vector<double> readings) const{
	using namespace std;
	l=0;
	s=0;
	const double * angle=m_laserAngles+m_initialBeamsSkip;
	OrientedPoint lp=p;
	lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
	lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
	lp.theta+=m_laserPose.theta;
	double noHit=nullLikelihood/(m_likelihoodSigma);
	unsigned int skip=0;
	unsigned int c=0;
	double freeDelta=map.getDelta()*m_freeCellRatio;
	//for (const double* r=readings+m_initialBeamsSkip; r<readings+m_laserBeams; r++, angle++){
	for (auto it = readings.begin() + m_initialBeamsSkip; it != readings.end(); it++, angle++){
		skip++;
		skip=skip>m_likelihoodSkip?0:skip;
		if (*it>m_usableRange) continue;
		if (skip) continue;
		Point phit=lp;
		phit.x+=*it*cos(lp.theta+*angle);
		phit.y+=*it*sin(lp.theta+*angle);
		IntPoint iphit=map.world2map(phit);
		Point pfree=lp;
		pfree.x+=(*it-freeDelta)*cos(lp.theta+*angle);
		pfree.y+=(*it-freeDelta)*sin(lp.theta+*angle);
		pfree=pfree-phit;
		IntPoint ipfree=map.world2map(pfree);
		bool found=false;
		Point bestMu(0.,0.);
		for (int xx=-m_kernelSize; xx<=m_kernelSize; xx++)
		for (int yy=-m_kernelSize; yy<=m_kernelSize; yy++){
			IntPoint pr=iphit+IntPoint(xx,yy);
			IntPoint pf=pr+ipfree;
			//AccessibilityState s=map.storage().cellState(pr);
			//if (s&Inside && s&Allocated){
				const PointAccumulator& cell=map.cell(pr);
				const PointAccumulator& fcell=map.cell(pf);
				if (((double)cell )>m_fullnessThreshold && ((double)fcell )<m_fullnessThreshold){
					Point mu=phit-cell.mean();
					if (!found){
						bestMu=mu;
						found=true;
					}else
						bestMu=(mu*mu)<(bestMu*bestMu)?mu:bestMu;
				}
			//}
		}
		if (found){
			s+=exp(-1./m_gaussianSigma*bestMu*bestMu);
			c++;
		}
		if (!skip){
			double f=(-1./m_likelihoodSigma)*(bestMu*bestMu);
			l+=(found)?f:noHit;
		}
	}
	return c;
}

};

#endif /* SCANMATCHING_SCANMATCHER_H_ */
