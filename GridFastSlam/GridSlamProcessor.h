/*
 * GridSlamProcessor.h
 *
 *  Created on: 21 maj 2018
 *      Author: Admin
 */

#ifndef GRIDFASTSLAM_GRIDSLAMPROCESSOR_H_
#define GRIDFASTSLAM_GRIDSLAMPROCESSOR_H_

#include <climits>
#include <limits>
#include <fstream>
#include <vector>
#include <deque>
#include <Particlefilter/Particlefilter.h>
#include <utils/point.h>
#include <Sensors/RangeSensor.h>
#include <Sensors/RangeReading.h>
#include <ScanMatching/ScanMatcher.h>
#include <map>
#include "MotionModel.h"

namespace GMapping {

class GridSlamProcessor
{
public:
    struct TNode
	{
		TNode(const OrientedPoint& pose, double weight, TNode* parent=0, unsigned int childs=0);
		~TNode();

		OrientedPoint pose;
		double weight;
		double accWeight;
		double gweight;

		TNode* parent;
		const RangeReading* reading;
		unsigned int childs;
		mutable unsigned int visitCounter;
		mutable bool flag;
    };

    typedef std::vector<GridSlamProcessor::TNode*> TNodeVector;
    typedef std::deque<GridSlamProcessor::TNode*> TNodeDeque;

    struct Particle
    {
		Particle(const ScanMatcherMap& map);

		inline operator double() const {return weight;}
		inline operator OrientedPoint() const {return pose;}
		inline void setWeight(double w) {weight=w;}

		ScanMatcherMap map;
		OrientedPoint pose;
		OrientedPoint previousPose;
		double weight;
		double weightSum;
		double gweight;
		int previousIndex;
		TNode* node;
    };

    typedef std::vector<Particle> ParticleVector;

    GridSlamProcessor();
    virtual ~GridSlamProcessor();

    GridSlamProcessor* clone() const;

    void init(unsigned int size, double xmin, double ymin, double xmax, double ymax, double delta, int beams,
	      OrientedPoint initialPose=OrientedPoint(0,0,0));
    void setMatchingParameters(double urange, double range, double sigma, int kernsize, double lopt, double aopt,
			       int iterations, double likelihoodSigma=1, double likelihoodGain=1, unsigned int likelihoodSkip=0);
    void setMotionModelParameters(double srr, double srt, double str, double stt);
    void setUpdateDistances(double linear, double angular, double resampleThreshold);

    bool processScan(const RangeReading & reading, int adaptParticles=0);

    TNodeVector getTrajectories() const;

    ScanMatcher m_matcher;
    inline const ParticleVector& getParticles() const { return m_particles; }

    int getBestParticleIndex() const;
    ScanMatcherMap getBestParticleMap() const { return m_particles[getBestParticleIndex()].map; }

protected:
    GridSlamProcessor(const GridSlamProcessor& gsp);

    unsigned int m_beams;
    ParticleVector m_particles;
    std::vector<unsigned int> m_indexes;
    std::vector<double> m_weights;
    MotionModel m_motionModel;


    //************************************************
    double m_resampleThreshold;
    double m_minimumScore;

    int  m_count, m_readingCount;
    OrientedPoint m_lastPartPose;
    OrientedPoint m_odoPose;
    OrientedPoint m_pose;
    double m_linearDistance, m_angularDistance;
    double m_neff;

    //processing parameters (size of the map)
    double m_xmin;
    double m_ymin;
    double m_xmax;
    double m_ymax;

    //processing parameters (resolution of the map)
    double m_delta;

    //registration score (if a scan score is above this threshold it is registered in the map)
    double m_regScore;
    //registration score (if a scan score is below this threshold a scan matching failure is reported)
    double m_critScore;
    //registration score maximum move allowed between consecutive scans
    double m_maxMove;

    double m_linearThresholdDistance;
    double m_angularThresholdDistance;
    //smoothing factor for the likelihood
    double m_obsSigmaGain;

    std::ostream& m_infoStream;

private:
    inline void scanMatch(const ScanReading plainReading);
    inline void normalize();
    inline bool resample(const ScanReading plainReading, int adaptParticles, const RangeReading* rr=0);
    void updateTreeWeights(bool weightsAlreadyNormalized = false);
    void resetTree();
    double propagateWeights();
  };

typedef std::multimap<const GridSlamProcessor::TNode*, GridSlamProcessor::TNode*> TNodeMultimap;

#include "GridSlamProcessor.hxx"

};
#endif /* GRIDFASTSLAM_GRIDSLAMPROCESSOR_H_ */
