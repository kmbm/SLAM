/*
 * GridSlamProcessor.cpp
 *
 *  Created on: 21 maj 2018
 *      Author: Admin
 */

#include <GridFastSlam/GridSlamProcessor.h>

#include <string>
#include <deque>
#include <list>
#include <map>
#include <set>
#include <fstream>
#include <iomanip>
#include <utils/stat.h>

namespace GMapping {

const double m_distanceThresholdCheck = 20;

using namespace std;

  GridSlamProcessor::GridSlamProcessor(): m_infoStream(cout){
    m_obsSigmaGain=1;
    m_resampleThreshold=0.5;
    m_minimumScore=0.;
  }

  GridSlamProcessor::GridSlamProcessor(const GridSlamProcessor& gsp)
    :m_particles(gsp.m_particles), m_infoStream(cout){

    m_obsSigmaGain=gsp.m_obsSigmaGain;
    m_resampleThreshold=gsp.m_resampleThreshold;
    m_minimumScore=gsp.m_minimumScore;

    m_beams=gsp.m_beams;
    m_indexes=gsp.m_indexes;
    m_motionModel=gsp.m_motionModel;
    m_resampleThreshold=gsp.m_resampleThreshold;
    m_matcher=gsp.m_matcher;

    m_count=gsp.m_count;
    m_readingCount=gsp.m_readingCount;
    m_lastPartPose=gsp.m_lastPartPose;
    m_pose=gsp.m_pose;
    m_odoPose=gsp.m_odoPose;
    m_linearDistance=gsp.m_linearDistance;
    m_angularDistance=gsp.m_angularDistance;
    m_neff=gsp.m_neff;

    cerr << "FILTER COPY CONSTRUCTOR" << endl;
    cerr << "m_odoPose=" << m_odoPose.x << " " <<m_odoPose.y << " " << m_odoPose.theta << endl;
    cerr << "m_lastPartPose=" << m_lastPartPose.x << " " <<m_lastPartPose.y << " " << m_lastPartPose.theta << endl;
    cerr << "m_linearDistance=" << m_linearDistance << endl;
    cerr << "m_angularDistance=" << m_linearDistance << endl;


    m_xmin=gsp.m_xmin;
    m_ymin=gsp.m_ymin;
    m_xmax=gsp.m_xmax;
    m_ymax=gsp.m_ymax;
    m_delta=gsp.m_delta;

    m_regScore=gsp.m_regScore;
    m_critScore=gsp.m_critScore;
    m_maxMove=gsp.m_maxMove;

    m_linearThresholdDistance=gsp.m_linearThresholdDistance;
    m_angularThresholdDistance=gsp.m_angularThresholdDistance;
    m_obsSigmaGain=gsp.m_obsSigmaGain;

    TNodeVector v=gsp.getTrajectories();
    for (unsigned int i=0; i<v.size(); i++){
		m_particles[i].node=v[i];
    }

    cerr  << "Tree: normalizing, resetting and propagating weights within copy construction/cloneing ..." ;
    updateTreeWeights(false);
    cerr  << ".done!" <<endl;
  }

GridSlamProcessor* GridSlamProcessor::clone() const
{
	GridSlamProcessor* cloned=new GridSlamProcessor(*this);
	return cloned;
}

GridSlamProcessor::~GridSlamProcessor()
{
	cerr << __PRETTY_FUNCTION__ << ": Start" << endl;
	cerr << __PRETTY_FUNCTION__ << ": Deeting tree" << endl;
	for (std::vector<Particle>::iterator it=m_particles.begin(); it!=m_particles.end(); it++)
	{
	  if (it->node)
		  delete it->node;
	}
}

void GridSlamProcessor::setMatchingParameters (double urange, double range, double sigma, int kernsize, double lopt, double aopt,
					 int iterations, double likelihoodSigma, double likelihoodGain, unsigned int likelihoodSkip)
{
	m_obsSigmaGain=likelihoodGain;
	m_matcher.setMatchingParameters(urange, range, sigma, kernsize, lopt, aopt, iterations, likelihoodSigma, likelihoodSkip);
	if (m_infoStream)
	  m_infoStream << " -maxUrange "<< urange
		   << " -maxUrange "<< range
		   << " -sigma     "<< sigma
		   << " -kernelSize "<< kernsize
		   << " -lstep "    << lopt
		   << " -lobsGain " << m_obsSigmaGain
		   << " -astep "    << aopt << endl;


}

void GridSlamProcessor::setMotionModelParameters(double srr, double srt, double str, double stt)
{
    m_motionModel.srr=srr;
    m_motionModel.srt=srt;
    m_motionModel.str=str;
    m_motionModel.stt=stt;
}

void GridSlamProcessor::setUpdateDistances(double linear, double angular, double resampleThreshold)
{
	m_linearThresholdDistance=linear;
	m_angularThresholdDistance=angular;
	m_resampleThreshold=resampleThreshold;
}

GridSlamProcessor::Particle::Particle(const ScanMatcherMap& m):
	map(m), pose(0,0,0), weight(0), weightSum(0), gweight(0), previousIndex(0)
{
	node=0;
}

void GridSlamProcessor::init(unsigned int size, double xmin, double ymin, double xmax, double ymax, double delta, int beams, OrientedPoint initialPose){
    m_xmin=xmin;
    m_ymin=ymin;
    m_xmax=xmax;
    m_ymax=ymax;
    m_delta=delta;
    m_beams=beams;
    if (m_infoStream)
      m_infoStream
	<< " -xmin "<< m_xmin
	<< " -xmax "<< m_xmax
	<< " -ymin "<< m_ymin
	<< " -ymax "<< m_ymax
	<< " -delta "<< m_delta
	<< " -beams "<< m_beams
	<< " -particles "<< size << endl;

    m_particles.clear();
    TNode* node=new TNode(initialPose, 0, 0, 0);

    ScanMatcherMap lmap(Point(xmin+xmax, ymin+ymax)*.5, xmax-xmin, ymax-ymin, delta);
    for (unsigned int i=0; i<size; i++)
    {
        m_particles.push_back(Particle(lmap));
		m_particles.back().pose=initialPose;
		m_particles.back().previousPose=initialPose;
		m_particles.back().setWeight(0);
		m_particles.back().previousIndex=0;
		m_particles.back().node= node;
    }
    m_neff=(double)size;
    m_count=0;
    m_readingCount=0;
    m_linearDistance=m_angularDistance=0;
}

bool GridSlamProcessor::processScan(const RangeReading & reading, int adaptParticles)
{
	std::cerr << "Prediction step";
    OrientedPoint relPose=reading.getPose();
    if (!m_count)
    {
        m_lastPartPose=m_odoPose=relPose;
    }
    //write the state of the reading and update all the particles using the motion model
    for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++)
    {
        OrientedPoint& pose(it->pose);
        pose=m_motionModel.drawFromMotion(it->pose, relPose, m_odoPose);
    }

    // accumulate the robot translation and rotation
    OrientedPoint move=relPose-m_odoPose;
    move.theta=atan2(sin(move.theta), cos(move.theta));
    m_linearDistance+=sqrt(move*move);
    m_angularDistance+=fabs(move.theta);

    // if the robot jumps throw a warning
    if (m_linearDistance>m_distanceThresholdCheck){
      cerr << "***********************************************************************" << endl;
      cerr << "********** Error: m_distanceThresholdCheck overridden!!!! *************" << endl;
      cerr << "m_distanceThresholdCheck=" << m_distanceThresholdCheck << endl;
      cerr << "Old Odometry Pose= " << m_odoPose.x << " " << m_odoPose.y
	   << " " <<m_odoPose.theta << endl;
      cerr << "New Odometry Pose (reported from observation)= " << relPose.x << " " << relPose.y
	   << " " <<relPose.theta << endl;
      cerr << "***********************************************************************" << endl;
      cerr << "** The Odometry has a big jump here. This is probably a bug in the   **" << endl;
      cerr << "** odometry/laser input. We continue now, but the result is probably **" << endl;
      cerr << "** crap or can lead to a core dump since the map doesn't fit.... C&G **" << endl;
      cerr << "***********************************************************************" << endl;
    }

    m_odoPose=relPose;

    bool processed=false;

    // process a scan only if the robot has traveled a given distance
    if (! m_count || m_linearDistance>m_linearThresholdDistance	|| m_angularDistance>m_angularThresholdDistance)
    {
		cerr << "Laser Pose= " << reading.getPose().x << " " << reading.getPose().y	<< " " << reading.getPose().theta << endl;

		//this is for converting the reading in a scan-matcher feedable form
		assert(reading.getScanReadingSize()==m_beams);
		auto l_scanReadingVector = reading.getScanReading();

		if (m_count>0)
		{
			scanMatch(l_scanReadingVector);
			updateTreeWeights(false);
			resample(l_scanReadingVector, adaptParticles);
		}
		else
		{
			for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++)
			{
				m_matcher.invalidateActiveArea();
				m_matcher.computeActiveArea(it->map, it->pose, l_scanReadingVector);
				m_matcher.registerScan(it->map, it->pose, l_scanReadingVector);

				// cyr: not needed anymore, particles refer to the root in the beginning!
				TNode* node=new	TNode(it->pose, 0., it->node,  0);
				node->reading=0;
				it->node=node;
			}
		}
      	cerr  << "Tree: normalizing, resetting and propagating weights at the end..." ;
		updateTreeWeights(false);

		m_lastPartPose=m_odoPose;
		m_linearDistance=0;
		m_angularDistance=0;
		m_count++;
		processed=true;

		for (ParticleVector::iterator it=m_particles.begin(); it!=m_particles.end(); it++)
		{
			it->previousPose=it->pose;
		}
    }
    m_readingCount++;
    return processed;
}

int GridSlamProcessor::getBestParticleIndex() const
{
    unsigned int bi=0;
    double bw=-std::numeric_limits<double>::max();
    for (unsigned int i=0; i<m_particles.size(); i++)
    {
        if (bw<m_particles[i].weightSum)
        {
        	bw=m_particles[i].weightSum;
        	bi=i;
        }
    }
    return (int) bi;
}

};




