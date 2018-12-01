/*
 * ScanMatcher.cpp
 *
 *  Created on: 20 maj 2018
 *      Author: Admin
 */

#include <ScanMatching/ScanMatcher.h>

#include <cstring>
#include <limits>
#include <list>
#include <iostream>

#include "scanmatcher.h"
#include "gridlinetraversal.h"

namespace GMapping {

using namespace std;

const double ScanMatcher::nullLikelihood=-.5;

ScanMatcher::ScanMatcher(): m_laserPose(0,0,0){
	//m_laserAngles=0;
	m_laserBeams=0;
	m_optRecursiveIterations=3;
	m_activeAreaComputed=false;

	// This  are the dafault settings for a grid map of 5 cm
	m_llsamplerange=0.01;
	m_llsamplestep=0.01;
	m_lasamplerange=0.005;
	m_lasamplestep=0.005;
	m_enlargeStep=10.;
	m_fullnessThreshold=0.1;
	m_angularOdometryReliability=0.;
	m_linearOdometryReliability=0.;
	m_freeCellRatio=sqrt(2.);
	m_initialBeamsSkip=0;

/*
	// This  are the dafault settings for a grid map of 10 cm
	m_llsamplerange=0.1;
	m_llsamplestep=0.1;
	m_lasamplerange=0.02;
	m_lasamplestep=0.01;
*/
	// This  are the dafault settings for a grid map of 20/25 cm
/*
	m_llsamplerange=0.2;
	m_llsamplestep=0.1;
	m_lasamplerange=0.02;
	m_lasamplestep=0.01;
	m_generateMap=false;
*/
}

void ScanMatcher::invalidateActiveArea(){
	m_activeAreaComputed=false;
}

void ScanMatcher::computeActiveArea(ScanMatcherMap& map, const OrientedPoint& p, const ScanReading readings){
	if (m_activeAreaComputed)
		return;
	OrientedPoint lp=p;
	lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
	lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
	lp.theta+=m_laserPose.theta;
	IntPoint p0=map.world2map(lp);

	Point min(map.map2world(0,0));
	Point max(map.map2world(map.getMapSizeX()-1,map.getMapSizeY()-1));

	if (lp.x<min.x) min.x=lp.x;
	if (lp.y<min.y) min.y=lp.y;
	if (lp.x>max.x) max.x=lp.x;
	if (lp.y>max.y) max.y=lp.y;

	for (auto r = readings.begin() + m_initialBeamsSkip; r < readings.end(); r++)
	{
		if (r->quality > MIN_LIDAR_QUALITY)
		{
			auto l_distance = r->distance;
			auto l_angle = r->angle;
			if (l_distance > m_laserMaxRange) continue;
			double d=l_distance > m_usableRange ? m_usableRange : l_distance;
			Point phit=lp;
			phit.x+=d*cos(lp.theta + l_angle);
			phit.y+=d*sin(lp.theta + l_angle);
			if (phit.x<min.x) min.x=phit.x;
			if (phit.y<min.y) min.y=phit.y;
			if (phit.x>max.x) max.x=phit.x;
			if (phit.y>max.y) max.y=phit.y;
		}
	}

	if ( !map.isInside(min)	|| !map.isInside(max)){
		Point lmin(map.map2world(0,0));
		Point lmax(map.map2world(map.getMapSizeX()-1,map.getMapSizeY()-1));
		min.x=( min.x >= lmin.x )? lmin.x: min.x-m_enlargeStep;
		max.x=( max.x <= lmax.x )? lmax.x: max.x+m_enlargeStep;
		min.y=( min.y >= lmin.y )? lmin.y: min.y-m_enlargeStep;
		max.y=( max.y <= lmax.y )? lmax.y: max.y+m_enlargeStep;
		map.resize(min.x, min.y, max.x, max.y);
	}

	HierarchicalArray2D<PointAccumulator>::PointSet activeArea;

	for (auto r = readings.begin() + m_initialBeamsSkip; r < readings.end(); r++)
	{
		if (r->quality > MIN_LIDAR_QUALITY)
		{
			auto l_angle = r->angle;
			if (m_generateMap){
				double d = r->distance;
				if (d>m_laserMaxRange)
					continue;
				if (d>m_usableRange)
					d=m_usableRange;
				Point phit=lp+Point(d*cos(lp.theta+l_angle),d*sin(lp.theta+l_angle));
				IntPoint p0=map.world2map(lp);
				IntPoint p1=map.world2map(phit);

				IntPoint linePoints[20000] ;
				GridLineTraversalLine line;
				line.points=linePoints;
				GridLineTraversal::gridLine(p0, p1, &line);
				for (int i=0; i<line.num_points-1; i++){
					assert(map.isInside(linePoints[i]));
					activeArea.insert(map.storage().patchIndexes(linePoints[i]));
					assert(linePoints[i].x>=0 && linePoints[i].y>=0);
				}
				if (d<m_usableRange){
					IntPoint cp=map.storage().patchIndexes(p1);
					assert(cp.x>=0 && cp.y>=0);
					activeArea.insert(cp);
				}
			} else {
				auto l_distance = r->distance;
				if (l_distance > m_laserMaxRange|| l_distance > m_usableRange) continue;
				Point phit=lp;
				phit.x+=l_distance*cos(lp.theta+l_angle);
				phit.y+=l_distance*sin(lp.theta+l_angle);
				IntPoint p1=map.world2map(phit);
				assert(p1.x>=0 && p1.y>=0);
				IntPoint cp=map.storage().patchIndexes(p1);
				assert(cp.x>=0 && cp.y>=0);
				activeArea.insert(cp);
			}
		}
	}
	map.storage().setActiveArea(activeArea, true);
	m_activeAreaComputed=true;
}


double ScanMatcher::registerScan(ScanMatcherMap& map, const OrientedPoint& p, const ScanReading readings){
	if (!m_activeAreaComputed)
		computeActiveArea(map, p, readings);

	//this operation replicates the cells that will be changed in the registration operation
	map.storage().allocActiveArea();

	OrientedPoint lp=p;
	lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
	lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
	lp.theta+=m_laserPose.theta;
	IntPoint p0=map.world2map(lp);


	double esum=0;
	for (auto r = readings.begin() + m_initialBeamsSkip; r < readings.end(); r++)
	{
		if (r->quality > MIN_LIDAR_QUALITY)
		{
			auto l_angle = r->angle;
			auto l_distance = r->distance;
			if (m_generateMap){
				double d=l_distance;
				if (d>m_laserMaxRange)
					continue;
				if (d>m_usableRange)
					d=m_usableRange;
				Point phit=lp+Point(d*cos(lp.theta+l_angle),d*sin(lp.theta+l_angle));
				IntPoint p1=map.world2map(phit);
				IntPoint linePoints[20000] ;
				GridLineTraversalLine line;
				line.points=linePoints;
				GridLineTraversal::gridLine(p0, p1, &line);
				for (int i=0; i<line.num_points-1; i++){
					PointAccumulator& cell=map.cell(line.points[i]);
					double e=-cell.entropy();
					cell.update(false, Point(0,0));
					e+=cell.entropy();
					esum+=e;
				}
				if (d<m_usableRange){
					double e=-map.cell(p1).entropy();
					map.cell(p1).update(true, phit);
					//map.cell(p1) = 5;
					e+=map.cell(p1).entropy();
					esum+=e;
				}
			} else {
				if (l_distance > m_laserMaxRange|| l_distance > m_usableRange) continue;
				Point phit=lp;
				phit.x+= l_distance * cos(lp.theta+l_angle);
				phit.y+= l_distance * sin(lp.theta+l_angle);
				IntPoint p1=map.world2map(phit);
				assert(p1.x>=0 && p1.y>=0);
				map.cell(p1).update(true,phit);
			}
		}
	}
	return esum;
}

double ScanMatcher::optimize(OrientedPoint& pnew, const ScanMatcherMap& map, const OrientedPoint& init, const ScanReading readings) const{
	double bestScore=-1;
	OrientedPoint currentPose=init;
	double currentScore=score(map, currentPose, readings);
	double adelta=m_optAngularDelta, ldelta=m_optLinearDelta;
	unsigned int refinement=0;
	enum Move{Front, Back, Left, Right, TurnLeft, TurnRight, Done};
	int c_iterations=0;
	do{
		if (bestScore>=currentScore){
			refinement++;
			adelta*=.5;
			ldelta*=.5;
		}
		bestScore=currentScore;

		OrientedPoint bestLocalPose=currentPose;
		OrientedPoint localPose=currentPose;

		Move move=Front;
		do {
			localPose=currentPose;
			switch(move){
				case Front:
					localPose.x+=ldelta;
					move=Back;
					break;
				case Back:
					localPose.x-=ldelta;
					move=Left;
					break;
				case Left:
					localPose.y-=ldelta;
					move=Right;
					break;
				case Right:
					localPose.y+=ldelta;
					move=TurnLeft;
					break;
				case TurnLeft:
					localPose.theta+=adelta;
					move=TurnRight;
					break;
				case TurnRight:
					localPose.theta-=adelta;
					move=Done;
					break;
				default:;
			}

			double odo_gain=1;
			if (m_angularOdometryReliability>0.){
				double dth=init.theta-localPose.theta; 	dth=atan2(sin(dth), cos(dth)); 	dth*=dth;
				odo_gain*=exp(-m_angularOdometryReliability*dth);
			}
			if (m_linearOdometryReliability>0.){
				double dx=init.x-localPose.x;
				double dy=init.y-localPose.y;
				double drho=dx*dx+dy*dy;
				odo_gain*=exp(-m_linearOdometryReliability*drho);
			}
			double localScore=odo_gain*score(map, localPose, readings);

			if (localScore>currentScore){
				currentScore=localScore;
				bestLocalPose=localPose;
			}
			c_iterations++;
		} while(move!=Done);
		currentPose=bestLocalPose;

		//here we look for the best move;
	}while (currentScore>bestScore || refinement<m_optRecursiveIterations);
	pnew=currentPose;
	return bestScore;
}

void ScanMatcher::setMatchingParameters
	(double urange, double range, double sigma, int kernsize, double lopt, double aopt, int iterations,  double likelihoodSigma, unsigned int likelihoodSkip){
	m_usableRange=urange;
	m_laserMaxRange=range;
	m_kernelSize=kernsize;
	m_optLinearDelta=lopt;
	m_optAngularDelta=aopt;
	m_optRecursiveIterations=iterations;
	m_gaussianSigma=sigma;
	m_likelihoodSigma=likelihoodSigma;
	m_likelihoodSkip=likelihoodSkip;
}

double ScanMatcher::score(const ScanMatcherMap& map, const OrientedPoint& p, const ScanReading readings) const{
	double s=0;
	OrientedPoint lp=p;
	lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
	lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
	lp.theta+=m_laserPose.theta;
	unsigned int skip=0;
	double freeDelta=map.getDelta()*m_freeCellRatio;

	for (auto r = readings.begin() + m_initialBeamsSkip; r < readings.end(); r++)
	{
		skip++;
		skip=skip>m_likelihoodSkip?0:skip;
		if (r->quality > MIN_LIDAR_QUALITY)
		{
			auto l_angle = r->angle;
			auto l_distance = r->distance;
			if (l_distance > m_usableRange) continue;
			if (skip) continue;
			Point phit=lp;
			phit.x+=l_distance * cos(lp.theta+l_angle);
			phit.y+=l_distance*sin(lp.theta+l_angle);
			IntPoint iphit=map.world2map(phit);
			Point pfree=lp;
			pfree.x+=(l_distance-map.getDelta()*freeDelta)*cos(lp.theta+l_angle);
			pfree.y+=(l_distance-map.getDelta()*freeDelta)*sin(lp.theta+l_angle);
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
			{
				s+=exp(-1./m_gaussianSigma*bestMu*bestMu);
			}
		}
	}
	return s;
}

unsigned int ScanMatcher::likelihoodAndScore(double& s, double& l, const ScanMatcherMap& map, const OrientedPoint& p, const ScanReading readings) const{
	using namespace std;
	l=0;
	s=0;
	OrientedPoint lp=p;
	lp.x+=cos(p.theta)*m_laserPose.x-sin(p.theta)*m_laserPose.y;
	lp.y+=sin(p.theta)*m_laserPose.x+cos(p.theta)*m_laserPose.y;
	lp.theta+=m_laserPose.theta;
	double noHit=nullLikelihood/(m_likelihoodSigma);
	unsigned int skip=0;
	unsigned int c=0;
	double freeDelta=map.getDelta()*m_freeCellRatio;

	for (auto it = readings.begin() + m_initialBeamsSkip; it != readings.end(); it++)
	{
		skip++;
		skip=skip>m_likelihoodSkip?0:skip;
		if (it->quality)
		{
			auto l_angle = it->angle;
			auto l_distance = it->distance;
			if (l_distance > m_usableRange) continue;
			if (skip) continue;
			Point phit=lp;
			phit.x+=l_distance*cos(lp.theta+l_angle);
			phit.y+=l_distance*sin(lp.theta+l_angle);
			IntPoint iphit=map.world2map(phit);
			Point pfree=lp;
			pfree.x+=(l_distance-freeDelta)*cos(lp.theta+l_angle);
			pfree.y+=(l_distance-freeDelta)*sin(lp.theta+l_angle);
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
	}
	return c;
}

};

