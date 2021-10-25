// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2015  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.rea@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/

/**
 * @file MotionProfile.h
 * @brief Definition of the profile of the movement as plannar parmatric ellipse oriented in the space.
 */


#ifndef _MOTION_PROFILE_H_
#define _MOTION_PROFILE_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/RateThread.h>
#include <yarp/math/Math.h>

#include <gsl/gsl_math.h>

#include <iostream>
#include <fstream>
#include <time.h>
#include <stdio.h>
//#include <cmath>
#include <math.h>

#define minX_DF  -0.35;  // working space x
#define maxX_DF  -0.20;  // working space x
#define minY_DF  -0.35;  // working space y
#define maxY_DF  +0.35;  // working space y
#define minZ_DF  -0.15;  // working space z
#define maxZ_DF  +0.25;  // working space z

namespace profileFactory {

class MotionProfile {
private:
	/**
    * range initialization
    */
	void initRange();


protected:
    yarp::sig::Vector O;               // vector representating the position of the center ellipse
    yarp::sig::Vector A;               // vector representating the initial position for the hand
    yarp::sig::Vector B;               // vector representating the final desired position for the hand
    yarp::sig::Vector C;               // vector representating the point symmetric to A
		yarp::sig::Vector D;               // vector representating the point symmetric to B

    yarp::sig::Vector AO;              // vector from A to center of the ellipse
    yarp::sig::Vector AOnorm;          // normalized vector from A to center of the ellipse
    yarp::sig::Vector BO;              // vector from B to center of the ellipse
    yarp::sig::Vector BOnorm;          // normalized vector from B to center of the ellipse
    yarp::sig::Vector od;              // vector representing the desired orientation of the hand
    yarp::sig::Vector xPrev;           // vector representing the position at the previous step
    yarp::sig::Vector* xd;             // vector representing the desired position of the hand
    yarp::sig::Vector* xprev;          // vector position of the previous step
    yarp::sig::Vector* xder1;          // vector position of the previous step

    std::string type;                  // vocab representing the type

    double xAxis;                      // dimension of the major axis
    double yAxis;                      // dimension of the minor axis

    int a, b, c, d;                    // parameters of the plane

    double k;                          // curvature
    double t0;                         // time at the beginning istant
    double tprev;                      // time at the previous incremental step
    double radius;                     // radius of the ellipse function of the angle theta, a, b;
    double r,r2,r3;                    // computation variables
    double radiusPrev;                 // radius at the previous incremental step
    double theta_start;                // angular position of the point A
    //double thetaB;                   // angular position of the point B (useless)
    double theta_stop;                 // angular position of the point C
    double angVelocity;                // angular velocity in rad/s
    double theta;                      // angle in rad
    double thetaPrev;                  // previous angle in rad
    double tanVelocity;                // tangential velocity function of curvature, gain and beta
    double subA2B2;                    // variable needs for the computation of tang. and ang.Velocity

    double minX;					   // working space x
    double maxX;					   // working space x
    double minY;					   // working space y
    double maxY;					   // working space y
    double minZ;					   // working space z
    double maxZ;					   // working space z

    bool firstCompute;                 //
    bool valid;                        // flag indicating whether the motionProfile is valid class
    int reverseSign;                   // integer indicating if the action is performed forward or backward(reverse flag=1)

    yarp::os::BufferedPort<yarp::os::Bottle>  dataPort;

    //yarp::os::ResourceFinder rf;     // resourceFinder for the parsing of the parameters

public:
    /**
    * constructor default
    */
    MotionProfile();

    /**
    * constructor
    * @param robotname name of the robot
    */
    MotionProfile(int i){};

    /**
     * destructor
     */
    ~MotionProfile();

		/**
		 * read arguments and initialize the variables of the motion profile
		 */
		yarp::sig::Vector InitializeParameters(const yarp::os::Bottle *b);

    /**
    * checks whether the parameters are valid
    */
    bool isValid() const {return valid; };

    /**
    * function solving the operator ==
    */
    virtual bool operator==(const MotionProfile& mp)=0;

    /**
    * function that sets the time at the starting instant
    */
    void setT0(double _t0) { t0 = _t0; };

    /**
     * function that sets the flag reverse in the class
     */
    void setReverse(const bool _reverse) {_reverse? reverseSign = -1 : reverseSign = 1; };

    /**
    * function that sets where the action should start and stop
    * @param thetaA angle of the starting point (in rad)
    * @param thetaB angle of the viapoint (in rad)
    * @param thataC angle of the stop point (in rad)
    */
    void setStartStop(const double theta_start, const double theta_stop);

    /**
    * function to set the three via points in 3D space
    */
    void setCenter(yarp::sig::Vector O);

    /**
    * function to set the three via points in 3D space
    */
    void setViaPoints(const yarp::sig::Vector A,const yarp::sig::Vector B);

    /**
    * function to se the main axes of the ellipse that belong to the reference plane
    */
    void setAxes(const double xAxis,const double yAxis);

    /**
    * function returning the A vector
    */
    yarp::sig::Vector getO() { return O; };

    /**
    * function setting the A vector
    */
    void setA(yarp::sig::Vector value) { A = value; };

    /**
    * function returning the A vector
    */
    yarp::sig::Vector getA() { return A; };

    /**
    * function setting the A vector
    */
    void setB(yarp::sig::Vector value) { B = value; };

    /**
    * function returning the B vector
    */
    yarp::sig::Vector getB() { return B; };

    /**
    * function setting the C vector
    */
    void setC(yarp::sig::Vector value) { C = value; };

		/**
		* function setting the D vector
		*/
		void setD(yarp::sig::Vector value) { D = value; };

    /**
    * function returning the C vector
    */
    yarp::sig::Vector getC() { return C; };

		/**
		* function returning the D vector
		*/
		yarp::sig::Vector getD() { return D; };

    /**
     * function that returs the initial position of the traj.
     */
    yarp::sig::Vector getInitial();

    /**
     * function that returs the initial position of the traj.
     */
    double  getTanVelocity() {return tanVelocity; };

    /**
     * function that returns the curvature
     */
    double getCurvature() {return k;};

    /**
     * function that returns the angVelocity
     */
    double getAngVelocity() {return angVelocity;};

    /**
     * function that returns the radius
     */
    double getRadius() {return r;};

    /**
    * function that return the vector with module equal to the radius
    */
    yarp::sig::Vector normalizeVector(const yarp::sig::Vector u, double const radius);

    /**
    * function that computes the angular velocity given desired tang.Velocity, xAxis and yAxis
    */
    double computeAngVelocity(const double theta);

    /**
    * function that computes the tangential velocity given desired ang.Velocity, xAxis and yAxis
    */
    double checkTanVelocity(const double theta);

    /**
    * function that computed the radius in ellipse give a theta angle
    * @param theta angle at which the radius is computed [rad]
    * @return radius of the ellipse/circle at a given angle theta
    */
    virtual double computeRadius(const double theta);

    /**
    * function that checks if all the points belong to the reaching space
    */
    bool pointsAreReachable();

    /**
     * function that return whether the theta angle is in the execution range
     *
     */
    bool inRange(const double theta);

    /**
    * function preparing the relevant set of variable used in the computation
    */
    virtual void preComputation(const double t, const double theta) = 0;

    /**
    * vector returning the 3D location at the instant t
    * @return the pointer to the desired position of the hand
    */
    virtual yarp::sig::Vector* compute(double t) = 0;

};

/**
* motion profile with constant velocity
*/
class CVMotionProfile : public MotionProfile {
protected:
public:
    CVMotionProfile();
    ~CVMotionProfile();
    CVMotionProfile(const CVMotionProfile &cvmp);
    CVMotionProfile(const yarp::os::Bottle &b);

    CVMotionProfile &operator=(const CVMotionProfile &cvmp);
    bool operator==(const CVMotionProfile &cvmp);
    bool operator==(const MotionProfile &mp) {return operator==(dynamic_cast<const CVMotionProfile&>(mp));}

    /**
    * function that sets the desired tangential velocity of the endEffector
    */
    void setVelocity(const double vel) {tanVelocity = vel;};
    void preComputation(const double t, const double theta);
    yarp::sig::Vector* compute(double t);
};


/**
* motion profile with minimumJerk profile
*/
class MJMotionProfile : public MotionProfile {
protected:

    double tfinal;
    double thetaGoal;

public:
    MJMotionProfile();
    ~MJMotionProfile();
    MJMotionProfile(const MJMotionProfile &mjmp);
    MJMotionProfile(const yarp::os::Bottle &b);

    MJMotionProfile &operator=(const MJMotionProfile &mjmp);
    bool operator==(const MJMotionProfile &mjmp);
    bool operator==(const MotionProfile &mp) {return operator==(dynamic_cast<const MJMotionProfile&>(mp));}

    /**
    * function that sets the desired tangential velocity of the endEffector
    */
    void setVelocity(const double vel) {tanVelocity = vel;};
    void preComputation(const double t, const double theta);
    /**
    * function that computes the tangVelocity according to the min-jerk constraint
    */
    double computeTangVelocity(const double t);

    yarp::sig::Vector* compute(double t);
};

/**
* motion profile with general velocity profile (GVP)
*/
class GVPMotionProfile : public MotionProfile {
protected:
		int step_counter;     // vector defining the generic velocity profile used in GVP mode
		yarp::sig::Vector velocityProfile;

public:
    GVPMotionProfile();
    ~GVPMotionProfile();
    GVPMotionProfile(const GVPMotionProfile &cvmp);
    GVPMotionProfile(const yarp::os::Bottle &b);

    GVPMotionProfile &operator=(const GVPMotionProfile &cvmp);
    bool operator==(const GVPMotionProfile &cvmp);
    bool operator==(const MotionProfile &mp) {return operator==(dynamic_cast<const GVPMotionProfile&>(mp));}

    /**
    * function that sets the desired tangential velocity of the endEffector
    */
    void setVelocity(yarp::sig::Vector vel) {velocityProfile = vel;};
    void preComputation(const double t, const double theta);
    yarp::sig::Vector* compute(double t);
};

/**
* motion profile NOT respecting the 2/3 power law
*/
class TTPLMotionProfile : public MotionProfile {
protected:

    double gain;
    double beta;

public:
    TTPLMotionProfile();
    ~TTPLMotionProfile();
    TTPLMotionProfile(const TTPLMotionProfile &ttplmp);
    TTPLMotionProfile(const yarp::os::Bottle &b);

    TTPLMotionProfile &operator=(const MJMotionProfile &ttplmp);
    bool operator==(const TTPLMotionProfile &ttplmp);
    bool operator==(const MotionProfile &mp) {return operator==(dynamic_cast<const TTPLMotionProfile&>(mp));}

    /**
    * function that sets the gain parameter of the 2/3 power law
    */
    void setGain(const double _gain) { gain = _gain; };
    /**
    * function that set the gain parameter of the 2/3 power law
    */
    void setBeta(const double _beta) { beta = _beta; };
    /**
    * function that computes the tangVelocity related to the 2/3 power law
    */
    double computeTangVelocity();

    void preComputation(const double t, const double theta);
    yarp::sig::Vector* compute(double t);
};

/**
* motion profile  respecting the 2/3 power law
*/
class TwoThirdMotionProfile : public MotionProfile {
protected:

    double gain;
    double beta;

public:
    /**
     * constructor
     */
    TwoThirdMotionProfile();
    /**
     * destructor
     */
    ~TwoThirdMotionProfile();
    /**
     * constructor
     */
    TwoThirdMotionProfile(const TwoThirdMotionProfile &ttplmp);
    /**
     * constructor
     */
    TwoThirdMotionProfile(const yarp::os::Bottle &b);

    TwoThirdMotionProfile &operator=(const MJMotionProfile &ttplmp);
    bool operator==(const TwoThirdMotionProfile &ttplmp);
    bool operator==(const MotionProfile &mp) {return operator==(dynamic_cast<const TwoThirdMotionProfile&>(mp));}

    /**
    * function that sets the gain parameter of the 2/3 power law
    */
    void setGain(const double _gain) { gain = _gain; };
    /**
    * function that set the gain parameter of the 2/3 power law
    */
    void setBeta(const double _beta) { beta = _beta; };
    /**
    * function that computes the tangVelocity related to the 2/3 power law
    */
    double computeTangVelocity();

    /**
     * function that computes the curvature in a small epsilon around the current location
     */
    double computeCurvature(const double timeDiff, const double thetePrev, const yarp::sig::Vector* xprev);

    /**
     * function preparing the computation of the location in space and time
     */
    void preComputation(const double t, const double theta);

    /**
    * function that computes the angVelocity related to the 2/3 power law
    */
    double computeAngVelFabio();

    /**
     * computing the vector location
     */
    yarp::sig::Vector* compute(double t);
};



}
#endif  //_MOTION_PROFILE_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------
