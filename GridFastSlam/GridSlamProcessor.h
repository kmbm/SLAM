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
#include <particlefilter/particlefilter.h>
#include <utils/point.h>
//#include <log/sensorlog.h>
#include <Sensors/RangeSensor.h>
#include <Sensors/RangeReading.h>
#include <ScanMatching/ScanMatcher.h>
#include <map>
#include "MotionModel.h"



namespace GMapping {

  /**This class defines the basic GridFastSLAM algorithm.  It
     implements a rao blackwellized particle filter. Each particle
     has its own map and robot pose.<br> This implementation works
     as follows: each time a new pair odometry/laser reading is
     received, the particle's robot pose is updated according to the
     motion model.  This pose is subsequently used for initalizing a
     scan matching algorithm.  The scanmatcher performs a local
     optimization for each particle.  It is initialized with the
     pose drawn from the motion model, and the pose is corrected
     according to the each particle map.<br>
     In order to avoid unnecessary computation the filter state is updated
     only when the robot moves more than a given threshold.
  */
  class GridSlamProcessor{
  public:


    /**This class defines the the node of reversed tree in which the trajectories are stored.
       Each node of a tree has a pointer to its parent and a counter indicating the number of childs of a node.
       The tree is updated in a way consistent with the operation performed on the particles.
   */
    struct TNode{
      /**Constructs a node of the trajectory tree.
       @param pose:      the pose of the robot in the trajectory
       @param weight:    the weight of the particle at that point in the trajectory
       @param accWeight: the cumulative weight of the particle
       @param parent:    the parent node in the tree
       @param childs:    the number of childs
      */
      TNode(const OrientedPoint& pose, double weight, TNode* parent=0, unsigned int childs=0);

      /**Destroys a tree node, and consistently updates the tree. If a node whose parent has only one child is deleted,
       also the parent node is deleted. This because the parent will not be reacheable anymore in the trajectory tree.*/
      ~TNode();

      /**The pose of the robot*/
      OrientedPoint pose;

      /**The weight of the particle*/
      double weight;

      /**The sum of all the particle weights in the previous part of the trajectory*/
      double accWeight;

      double gweight;


      /**The parent*/
      TNode* parent;

      /**The range reading to which this node is associated*/
      const RangeReading* reading;

      /**The number of childs*/
      unsigned int childs;

      /**counter in visiting the node (internally used)*/
      mutable unsigned int visitCounter;

      /**visit flag (internally used)*/
      mutable bool flag;
    };

    typedef std::vector<GridSlamProcessor::TNode*> TNodeVector;
    typedef std::deque<GridSlamProcessor::TNode*> TNodeDeque;

    /**This class defines a particle of the filter. Each particle has a map, a pose, a weight and retains the current node in the trajectory tree*/
    struct Particle{
      /**constructs a particle, given a map
	 @param map: the particle map
      */
      Particle(const ScanMatcherMap& map);

      /** @returns the weight of a particle */
      inline operator double() const {return weight;}
      /** @returns the pose of a particle */
      inline operator OrientedPoint() const {return pose;}
      /** sets the weight of a particle
	  @param w the weight
      */
      inline void setWeight(double w) {weight=w;}
      /** The map */
      ScanMatcherMap map;
      /** The pose of the robot */
      OrientedPoint pose;

      /** The pose of the robot at the previous time frame (used for computing thr odometry displacements) */
      OrientedPoint previousPose;

      /** The weight of the particle */
      double weight;

      /** The cumulative weight of the particle */
      double weightSum;

      double gweight;

      /** The index of the previous particle in the trajectory tree */
      int previousIndex;

      /** Entry to the trajectory tree */
      TNode* node;
    };


    typedef std::vector<Particle> ParticleVector;

    /** Constructs a GridSlamProcessor, initialized with the default parameters */
    GridSlamProcessor();

    /** Constructs a GridSlamProcessor, whose output is routed to a stream.
     @param infoStr: the output stream
    */
    GridSlamProcessor(std::ostream& infoStr);

    /** @returns  a deep copy of the grid slam processor with all the internal structures.
    */
    GridSlamProcessor* clone() const;

    /**Deleted the gridslamprocessor*/
    virtual ~GridSlamProcessor();

    //methods for accessing the parameters
    void setSensorMap(const RangeSensor*);
    void init(unsigned int size, double xmin, double ymin, double xmax, double ymax, double delta, int beams,
	      OrientedPoint initialPose=OrientedPoint(0,0,0));
    void setMatchingParameters(double urange, double range, double sigma, int kernsize, double lopt, double aopt,
			       int iterations, double likelihoodSigma=1, double likelihoodGain=1, unsigned int likelihoodSkip=0);
    void setMotionModelParameters(double srr, double srt, double str, double stt);
    void setUpdateDistances(double linear, double angular, double resampleThreshold);

    //the "core" algorithm
    //void processTruePos(const OdometryReading& odometry);
    bool processScan(const RangeReading & reading, int adaptParticles=0);

    /**This method copies the state of the filter in a tree.
     The tree is represented through reversed pointers (each node has a pointer to its parent).
     The leafs are stored in a vector, whose size is the same as the number of particles.
     @returns the leafs of the tree
    */
    TNodeVector getTrajectories() const;
    void integrateScanSequence(TNode* node);

    /**the scanmatcher algorithm*/
    ScanMatcher m_matcher;
    /**the stream used for writing the output of the algorithm*/
    std::ofstream& outputStream();
    /**the stream used for writing the info/debug messages*/
    std::ostream& infoStream();
    /**@returns the particles*/
    inline const ParticleVector& getParticles() const {return m_particles; }

    inline const std::vector<unsigned int>& getIndexes() const{return m_indexes; }
    int getBestParticleIndex() const;
    ScanMatcherMap getBestParticleMap() const { return m_particles[getBestParticleIndex()].map; }
    //callbacks
    virtual void onOdometryUpdate();
    virtual void onResampleUpdate();
    virtual void onScanmatchUpdate();



  protected:
    /**Copy constructor*/
    GridSlamProcessor(const GridSlamProcessor& gsp);

    /**the laser beams*/
    unsigned int m_beams;


    /**the particles*/
    ParticleVector m_particles;

    /**the particle indexes after resampling (internally used)*/
    std::vector<unsigned int> m_indexes;

    /**the particle weights (internally used)*/
    std::vector<double> m_weights;

    /**the motion model*/
    MotionModel m_motionModel;


    //************************************************

    /**this sets the neff based resampling threshold*/
    double m_resampleThreshold;
    double m_minimumScore;

    //state
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

    //process a scan each time the robot translates of linearThresholdDistance
    double m_linearThresholdDistance;

    //process a scan each time the robot rotates more than angularThresholdDistance
    double m_angularThresholdDistance;

    //smoothing factor for the likelihood
    double m_obsSigmaGain;



    //***************************************




    //stream in which to write the gfs file
    std::ofstream m_outputStream;

    // stream in which to write the messages
    std::ostream& m_infoStream;


    // the functions below performs side effect on the internal structure,
    //should be called only inside the processScan method
  private:

    /**scanmatches all the particles*/
    inline void scanMatch(const std::vector<double> plainReading);
    /**normalizes the particle weights*/
    inline void normalize();

    // return if a resampling occured or not
    inline bool resample(const std::vector<double> plainReading, int adaptParticles,
			 const RangeReading* rr=0);

    //tree utilities

    void updateTreeWeights(bool weightsAlreadyNormalized = false);
    void resetTree();
    double propagateWeights();

  };

typedef std::multimap<const GridSlamProcessor::TNode*, GridSlamProcessor::TNode*> TNodeMultimap;


#include "GridSlamProcessor.hxx"

};
#endif /* GRIDFASTSLAM_GRIDSLAMPROCESSOR_H_ */
