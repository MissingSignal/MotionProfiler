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
 * @file MotionProfile.cpp
 * @brief Implementation of the MotionProfile class (see MotionProfile.h).
 */

#include <iCub/MotionProfile.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace std;
using namespace profileFactory;

inline void extractVector(const string str, Vector& res){
    std::string temp;
	size_t i = 0, start = 0, end;
	do {
		end = str.find_first_of (' ', start );
		temp = str.substr( start, end );
			res[i] = ( atof ( temp.c_str ( ) ) );
			++i;
		start = end + 1;
	} while ( start );
}

inline double eval_distance(const Vector Point1, const Vector Point2){

  double x = Point1[1] - Point2[1];
  double y = Point1[2] - Point2[2];
  double z = Point1[3] - Point2[3];

	double dist;

	dist = sqrt( pow(x, 2) + pow(y, 2) + pow(z, 2));       //calculating Euclidean distance

	return dist;
}

//**************************************************************************************************************

MotionProfile::MotionProfile() : valid(false){

  initRange();

  firstCompute = true;
  A.resize(3);
  B.resize(3);
  C.resize(3);
  D.resize(3);
  O.resize(3);
  AO.resize(3);
  BO.resize(3);
  xPrev.resize(3);
  xd = new Vector(3);
  xprev = new Vector(3);
  xder1 = new Vector(3);

  //////////Fabio open port for data gathering
  //string velName("");
  //velName.append(getName("/vel:o"));
  if(!dataPort.open("/motionProfile/data:o")) {
        yError("dataPort is not open with success. Check for conflicts");
  }
}

MotionProfile::~MotionProfile(){
    delete xd;
    delete xprev;
    delete xder1;
}

Vector MotionProfile::InitializeParameters(const Bottle *b){
  ResourceFinder rf;
  rf.setVerbose(false);

  const int argc = 20;
  string stringArray[argc];
  char* argv[argc];
  stringArray[0].append("./motionProfile");
  argv[0] = (char*) stringArray[0].c_str();

  for (int j = 0; j < b->size(); j++) { //j iterates over the parameters
      Bottle* vector = b->get(j).asList();
      stringArray[j * 2 + 1].append("--"); // put -- prefix
      stringArray[j * 2 + 1].append(vector->get(0).asString().c_str()); // put name of the resource (e.g. A,O,...)
      stringArray[j * 2 + 2].append(vector->tail().toString().c_str()); // put the remaining values

      argv[j * 2 + 1] = (char*) stringArray[j * 2 + 1].c_str();
      argv[j * 2 + 2] = (char*) stringArray[j * 2 + 2].c_str();
  }

  // configuring the resource finder
  //rf.configure(b->size() * 2 + 1, argv);
  rf.configure(argc, argv,false);

  // visiting the parameters using the RF
  Vector Ovector(3);
  Vector Avector(3);
  Vector Bvector(3);


  string Ostring = rf.check("O",
                         Value("-0.3 -0.1 0.1"),
                         "position O (string)").asString();
  extractVector(Ostring, Ovector);

  string Astring = rf.check("A",
                         Value("-0.3 0.0 0.1"),
                         "position A (string)").asString();
  extractVector(Astring, Avector);
  string Bstring = rf.check("B",
                         Value("-0.3 -0.1 0.3"),
                         "position B (string)").asString();
  extractVector(Bstring, Bvector);

  // TO DO: Si puo ripulire, calcoliamolo in setAxes @LUCA
  Vector axisVector(2);
  axisVector[0] = eval_distance(Avector,Ovector); //AO Axis
  axisVector[1] = eval_distance(Bvector,Ovector); //BO Axis

  Vector thetaVector(2);
  string  thetaString = rf.check("theta",
                         Value("0 6.28"),
                         "theta angles (string)").asString();
  extractVector(thetaString, thetaVector);

  // here we read the velocity profile passed as sequence of values
  string  paramString = rf.check("param",
                         Value("0.1"),
                         "parameters (string)").asString();
  Vector paramVector;

  //if we have only 1 param the string is read as empty, this is an ugly fix @Luca
  if(paramString.empty()){
      paramVector.push_back(rf.find("param").asFloat32());
  }
  else{
      double value;
      std::stringstream stream(paramString);
      while (stream >> value){
          paramVector.push_back(value);
      }
  }

  //print paramVector

  bool reverse = rf.check("rev");

  // set parameters
  setCenter(Ovector);

  setAxes(axisVector[0], axisVector[1]);

  if(pointsAreReachable()) {valid = true; setViaPoints(Avector,Bvector);}
  else {valid = false; yError("Error positions are not allowed!");}

  setReverse(reverse);
  setStartStop(thetaVector[0], thetaVector[1]);



    cout << ("\n--------------------------------------------------------------------------\n"); //for clarity
    yInfo("got O value       :      [m] %s", O.toString().c_str());
    yInfo("got A value       :      [m] %s",  A.toString().c_str());
    yInfo("got B value       :      [m] %s",  B.toString().c_str());
    yInfo("got C value       :      [m] %s (inferred)", C.toString().c_str());
    yInfo("got D value       :      [m] %s (inferred)", C.toString().c_str());
    yInfo("got theta start   :      [rad] %f", theta_start);
    yInfo("got theta stop    :      [rad] %f", theta_stop);
    if (reverseSign==-1){
        yInfo("got reverse       :      [bool] true");
    }
    else{
        yInfo("got reverse       :      [bool] false");
    }
    yInfo("parameters        :      %s", paramVector.toString().c_str());
  cout << ("--------------------------------------------------------------------------\n\n"); //for clarity

  return paramVector;
}

void MotionProfile::initRange(){
	 minX = -0.35;
     maxX = -0.20;
     minY = -0.35;
     maxY = +0.35;
     minZ = -0.15;
     maxZ = +0.25;
}

void MotionProfile::setCenter(Vector _O){
    O = _O;
}

void MotionProfile::setStartStop(const double t_start, const double t_stop){

    if(reverseSign == -1) {
      theta_start = t_stop;
      theta_stop = t_start;
    }
    else {
      theta_start = t_start;
      theta_stop = t_stop;
    }

}

Vector MotionProfile::normalizeVector(const Vector u, double const radius){
    double module = sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
    //yDebug("module %f", module);
    Vector unorm(3);
    if(module != 0) {
        unorm[0] = (u[0] / module);
        unorm[1] = (u[1] / module);
        unorm[2] = (u[2] / module);
    }
    return unorm;
}

void MotionProfile::setViaPoints(const Vector _A, const Vector _B){
    A = _A;
    B = _B;

    // scaled by the radius in A
    AO[0] = A[0] - O[0];
    AO[1] = A[1] - O[1];
    AO[2] = A[2] - O[2];
    AOnorm = normalizeVector(AO, 0.1);

    //not interesting
    //yInfo("AO = %s", AO.toString().c_str());
    //yInfo("AOnorm = %s scaled by radius: %f", AOnorm.toString().c_str(), xAxis);

    //scaled by the radius in B
    BO[0] = B[0] - O[0];
    BO[1] = B[1] - O[1];
    BO[2] = B[2] - O[2];
    BOnorm = normalizeVector(BO, 0.2);

    //not interesting
    //yInfo("BO = %s", BO.toString().c_str());
    //yInfo("BOnorm = %s scaled by the radius %f", BOnorm.toString().c_str(), yAxis);

    Vector N(3);
    N = cross(AO, BO);

    //yInfo("NormVector = %s", N.toString().c_str());

    double revA2   = 1 / (xAxis * xAxis);
    double revB2   = 1 / (yAxis * yAxis);
    subA2B2 = revA2 - revB2;

    // POINT C is automatically evaluated
    C[0] = O[0] + xAxis * -1 * AOnorm[0] ;
    C[1] = O[1] + xAxis * -1 * AOnorm[1] ;
    C[2] = O[2] + xAxis * -1 * AOnorm[2] ;

    // POINT D is automatically evaluated
    D[0] = O[0] + yAxis * -1 * BOnorm[0];
    D[1] = O[1] + yAxis * -1 * BOnorm[1];
    D[2] = O[2] + yAxis * -1 * BOnorm[2];
}

void MotionProfile::setAxes(const double _xAxis,const double _yAxis){
    xAxis = _xAxis;
    yAxis = _yAxis;
}

double MotionProfile::computeRadius(const double theta){
    double cos2theta = cos(theta) * cos(theta);
    double sin2theta = sin(theta) * sin(theta);
    double aSquare = xAxis * xAxis;
    double bSquare = yAxis * yAxis;
    double argSqrt = (aSquare * sin2theta + bSquare * cos2theta);
    double invR2 = (cos2theta/aSquare + sin2theta/bSquare);
    double R2 = 1 / invR2;
    r = sqrt(R2);
    r2 = r * r;
    r3 = r2 * r;
    //k = (xAxis * yAxis) / sqrt(argSqrt * argSqrt * argSqrt);
    //double k = (xAxis * yAxis) / sqrt(argSqrt);
    //k = 1 / r;

    //yDebug("%f: r=%f k=",theta, r);
    return r;
}

double MotionProfile::computeAngVelocity(const double theta){
    //double revA2    = 1/(xAxis * xAxis);
    //double revB2    = 1/(yAxis * yAxis);
    //double subA2B2  = revA2 - revB2;
    //double r2       = radius * radius;
    //double r3       = radius * radius * radius;
    double drdtheta = (r3 / 2) * subA2B2 * sin(2 * theta);
    double v2       = tanVelocity * tanVelocity;
    double den      = drdtheta * drdtheta + r2;
    return            sqrt(v2 / den);
}

double MotionProfile::checkTanVelocity(const double theta){
    //double revA2   = 1/(xAxis * xAxis);
    //double revB2   = 1/(yAxis * yAxis);
    //double subA2B2 = revA2 - revB2;
    //double r2       = radius * radius;
    //double r3       = radius * radius * radius;
    double drdtheta = (r3 / 2) * subA2B2 * sin(2 * theta);
    double w2       = angVelocity * angVelocity;
    double den      = drdtheta * drdtheta + r2;
    return            sqrt(w2 * den);
}

bool MotionProfile::pointsAreReachable(){
    // TO DO: this part should take into account which is the arm used @Luca
    //we evaluate A and B
    // Vector toCheck[2];
    // toCheck[0] = A;
    // toCheck[1] = B;
    //
    // for(auto i=0; i < 2; i++) {
    //     if(((toCheck[i])[0] < minX) || ((toCheck[i])[0] > maxX)) return false;
    //     if(((toCheck[i])[1] < minY) || ((toCheck[i])[1] > maxY)) return false;
    //     if(((toCheck[i])[2] < minZ) || ((toCheck[i])[2] > maxZ)) return false;
    // }
    return true;
}

Vector MotionProfile::getInitial(){
    Vector locInitial(3);
    //FOR ELLIPSE: compute angular velocity from A,B and desired tang.velocity
    radius = computeRadius(theta_start);
    //preComputation(t, theta_start);  //computes the radius
    locInitial[0]=O[0] + radius * cos(theta_start) * AOnorm[0] + radius * sin(theta_start) * BOnorm[0];
    locInitial[1]=O[1] + radius * cos(theta_start) * AOnorm[1] + radius * sin(theta_start) * BOnorm[1];
    locInitial[2]=O[2] + radius * cos(theta_start) * AOnorm[2] + radius * sin(theta_start) * BOnorm[2];
    return locInitial;
}

bool MotionProfile::inRange(double angle) const{
    //reverseSign = 1 when reverse = FALSE
    if(reverseSign == 1) {
        if((theta >= theta_start) && (theta <= theta_stop))
            return true;
        else
            return false;
    }
    //reverseSign = -1 when reverse = TRUE
    else {
       if((theta >= theta_stop) && (theta <= theta_start))
            return true;
        else
            return false;
    }
}

//***********************************************************************************************************************

CVMotionProfile::CVMotionProfile(){
    type = "CVP";
    valid = true;
    A.resize(3);
    B.resize(3);
    C.resize(3);
    D.resize(3);
    O.resize(3);
    AO.resize(3);
    BO.resize(3);
    xPrev.resize(3);
    xd = new Vector(3);
    thetaPrev = 0;
    tprev = 0;
    radiusPrev = 0.1;
}

CVMotionProfile::CVMotionProfile(const CVMotionProfile &cvmp){
    valid = cvmp.valid;
    type  = cvmp.type;
    A.resize(3);
    B.resize(3);
    C.resize(3);
    D.resize(3);
    O.resize(3);
    AO.resize(3);
    BO.resize(3);
    xPrev.resize(3);
    xd = new Vector(3);
    thetaPrev = 0;
    tprev = 0;
    radiusPrev = 0.1;
}

CVMotionProfile::~CVMotionProfile(){
    delete xd;
}

CVMotionProfile::CVMotionProfile(const Bottle& bInit){
    type = "CVP";
    valid = true;
    A.resize(3);
    B.resize(3);
    C.resize(3);
    D.resize(3);
    O.resize(3);
    AO.resize(3);
    BO.resize(3);
    xPrev.resize(3);
    xd = new Vector(3);
    thetaPrev = 0;
    tprev = 0;
    radiusPrev = 0.1;

    Bottle* b = bInit.get(0).asList();
    //read all the parameters and return the custom parameters that are specific for this class
    yarp::sig::Vector paramVector = InitializeParameters(b);
    setVelocity(paramVector[0]);
    cout << "Constant Velocity Profile: vel = " << paramVector[0] << endl;
}

bool CVMotionProfile::operator==(const CVMotionProfile &cvmp){
    return ((valid==cvmp.valid)&&(type==cvmp.type));
}

void CVMotionProfile::preComputation(const double t, const double theta){
    //angular velocity expressed in frequency [Hz],
    //e.g.: 0.1 => 1/10 of 2PI per second; 2PI in 10sec
    //e.g.: 0.2 => 1/20 of 2PI per second; 2PI in 20sec
    //ONLY FOR CIRCLES: angular velocity constant to obtain constant tang.velocity
    //FOR ELLIPSE: compute angular velocity from A,B and desired tang.velocity
    radius = computeRadius(theta);

    yInfo("computed radius %f [m] for tangVelocity %f [m/s]", radius, tanVelocity);
    // computing angular velocity in function of the radius, theta, tangential velocity
    //angVelocity = computeAngVelocity(theta);
    yInfo("The constant velocity is set at: %f", tanVelocity);
    angVelocity = tanVelocity / radius;

    yInfo("computed angular velocity %f in rad/s", angVelocity);
    double tanVelocity_temp = checkTanVelocity(theta);
    yInfo("computed tang velocity %f in m/s", tanVelocity_temp);
    //double theta = 2.0 * M_PI * angVelocity * (t - t0);
    yInfo("theta angle [rad] %f", theta);
}

Vector* CVMotionProfile::compute(double t){
    const float delta_time = 0.1;

    double thetaStart;
    if(t-t0 == 0) {
        theta = theta_start;
        thetaStart =  theta_start;
    }
    else {
        while (t-tprev < delta_time && t-t0 != 0){t=Time::now();} //wait the end of the time step
        yDebug("DELTA_TIME = %f [s]", t-tprev);

        yDebug("theta = %f + %i * (%f - %f) * %f ", thetaPrev,reverseSign,t,tprev,angVelocity);
        yDebug("theta =  thetaPrev + reverseSign * t-tprev * angVelocity;");

        theta =  thetaPrev + reverseSign * delta_time * angVelocity;
        yDebug("next theta = %f", theta);
    }

    Vector xdes = *xd;

    if (inRange(theta)) {
        preComputation(t, theta);
        (*xd)[0]=O[0] + radius * cos(theta) * AOnorm[0] + radius * sin(theta) * BOnorm[0];
        (*xd)[1]=O[1] + radius * cos(theta) * AOnorm[1] + radius * sin(theta) * BOnorm[1];
        (*xd)[2]=O[2] + radius * cos(theta) * AOnorm[2] + radius * sin(theta) * BOnorm[2];
    }
    else {
        return nullptr;
    }

    // updating the previous incremental step values
    radiusPrev = radius;
    thetaPrev  = theta;
    tprev      = t;
    xPrev      = (*xd);
    return xd;
}

//***********************************************************************************************************************

TTPLMotionProfile::TTPLMotionProfile(){
    type = "TTPL";
    valid = true;
    A.resize(3);
    B.resize(3);
    C.resize(3);
    D.resize(3);
    O.resize(3);
    AO.resize(3);
    BO.resize(3);
    xPrev.resize(3);
    xd = new Vector(3);
    xder1->zero();
    thetaPrev = 0;
    tprev = 0;
    radiusPrev = 0.1;
}

TTPLMotionProfile::TTPLMotionProfile(const TTPLMotionProfile &ttplmp){
    valid = ttplmp.valid;
    type  = ttplmp.type;
    A.resize(3);
    B.resize(3);
    C.resize(3);
    D.resize(3);
    O.resize(3);
    AO.resize(3);
    BO.resize(3);
    xPrev.resize(3);
    xd = new Vector(3);
    thetaPrev = 0;
    tprev = 0;
    radiusPrev = 0.1;
}

TTPLMotionProfile::~TTPLMotionProfile(){
    delete xd;
}

TTPLMotionProfile::TTPLMotionProfile(const Bottle& bInit){
    type = "TTPL";
    valid = false;

    A.resize(3);
    B.resize(3);
    C.resize(3);
    D.resize(3);
    O.resize(3);
    AO.resize(3);
    BO.resize(3);
    xPrev.resize(3);
    xd = new Vector(3);
    thetaPrev = 0;
    tprev = 0;
    radiusPrev = 0.1;

    Bottle* b = bInit.get(0).asList();
    //read all the parameters and return the custom parameters that are specific for this class
    yarp::sig::Vector paramVector = InitializeParameters(b);
    setGain(paramVector[0]);
    setBeta(paramVector[1]);
    cout << "TTPL Profile: gain = " << gain << " beta = " << beta << endl;
}

bool TTPLMotionProfile::operator==(const TTPLMotionProfile &ttplmp){
    return ((valid==ttplmp.valid)&&(type==ttplmp.type));
}

double TTPLMotionProfile::computeTangVelocity(){
    //ang.vel = g * K ^ (-beta);    beta = 0.33;
    //tan.vel = ang.vel * r
    double reBeta = -1 * beta;
    double curvature = 1 / radius;
    double vel = gain * pow(curvature, reBeta);
    yInfo("ComputeTangVelocity: beta= %f curvature=%f tan.vel=%f", beta, curvature, vel);
    return vel;
}

void TTPLMotionProfile::preComputation(const double  t, const double theta){
    //angular velocity expressed in frequency [Hz],
    //e.g.: 0.1 => 1/10 of 2PI per second; 2PI in 10sec
    //e.g.: 0.2 => 1/20 of 2PI per second; 2PI in 20sec
    //ONLY FOR CIRCLES: angular veloc.090001	 0.099486

    //FOR ELLIPSE: compute angular velocity from A,B and desired tang.velocity
    radius = computeRadius(theta);
    /*
    double rDiff2     = (radius - radiusPrev) * (radius-radiusPrev);
    double thetaDiff2 = (theta  - thetaPrev) * (theta - thetaPrev);
    double sDiff2     = rDiff2  + (radius * radius) * thetaDiff2;
    yInfo("rDiff2 %f thetaDiff2 %f radius %f sDiff %f", rDiff2, thetaDiff2, radius, sqrt(sDiff2));
    */

    /*
    double drdtheta   = (radius - radiusPrev) / (theta - thetaPrev);
    double drdtheta2  =  drdtheta * drdtheta;
    double v2         = (drdtheta2 + (radius * radius)) * (angVelocity * angVelocity);
    yInfo("v %f", sqrt(v2));
    */
    tanVelocity = computeTangVelocity();
    yInfo("computed radius %f [m] for tangVelocity %f [m/s]", radius, tanVelocity);
    // computing angular velocity in function of the radius, theta, tangential velocity
    angVelocity = computeAngVelocity(theta);
    yInfo("computed angular velocity %f in rad/s", angVelocity);
    double tanVelocity_temp = checkTanVelocity(theta);
    yInfo("computed tang velocity %f in m/s", tanVelocity_temp);
    //double theta = 2.0 * M_PI * angVelocity * (t - t0);
    yInfo("theta angle [rad] %f", theta);
}

Vector* TTPLMotionProfile::compute(double t){

    if(t-t0 == 0) {
        theta = theta_start;
    }
    else {
        theta =  thetaPrev + reverseSign * (t - tprev) * angVelocity;
    }

    Vector xdes = *xd;
    if(inRange(theta)) {
        preComputation(t, theta);
        (*xd)[0]=O[0] + radius * cos(theta) * AOnorm[0] + radius * sin(theta) * BOnorm[0];
        (*xd)[1]=O[1] + radius * cos(theta) * AOnorm[1] + radius * sin(theta) * BOnorm[1];
        (*xd)[2]=O[2] + radius * cos(theta) * AOnorm[2] + radius * sin(theta) * BOnorm[2];
        yInfo("xdes %s", xd->toString().c_str());
    }
    else {
        return nullptr;
    }

    Vector distance = (*xd) - xPrev;
    yInfo("travelled distance x = %f , travelled distance y = %f absolute = %f", distance[1]/(t-tprev), distance[2]/(t-tprev),
    sqrt(distance[1] * distance[1] + distance[2] * distance[2])/(t-tprev));

    // updating the previous incremental step values
    radiusPrev = radius;
    thetaPrev  = theta;
    tprev      = t;
    xPrev      = (*xd);
    return xd;
}

//***********************************************************************************************************************

TwoThirdMotionProfile::TwoThirdMotionProfile(){
    type = "TwoThird";
    valid = true;
    A.resize(3);
    B.resize(3);
    C.resize(3);
    D.resize(3);
    O.resize(3);
    AO.resize(3);
    BO.resize(3);
    xPrev.resize(3);
    xd = new Vector(3);
    thetaPrev = 0;
    tprev = 0;
    radiusPrev = 0.1;
}

TwoThirdMotionProfile::TwoThirdMotionProfile(const TwoThirdMotionProfile &ttplmp){
    valid = ttplmp.valid;
    type  = ttplmp.type;
    A.resize(3);
    B.resize(3);
    C.resize(3);
    D.resize(3);
    O.resize(3);
    AO.resize(3);
    BO.resize(3);
    xPrev.resize(3);
    xd = new Vector(3);
    thetaPrev = 0;
    tprev = 0;
    radiusPrev = 0.1;
}

TwoThirdMotionProfile::~TwoThirdMotionProfile(){
    delete xd;
}

TwoThirdMotionProfile::TwoThirdMotionProfile(const Bottle& bInit){
    type = "TwoThird";
    valid = false;

    A.resize(3);
    B.resize(3);
    C.resize(3);
    D.resize(3);
    O.resize(3);
    AO.resize(3);
    BO.resize(3);
    xPrev.resize(3);
    xd = new Vector(3);
    thetaPrev = 0;
    tprev = 0;
    radiusPrev = 0.1;

    Bottle* b = bInit.get(0).asList();
    //read all the parameters and return the custom parameters that are specific for this class
    yarp::sig::Vector paramVector = InitializeParameters(b);
    setGain(paramVector[0]);
    setBeta(paramVector[1]);
    cout << "Two Third Motion Profile: gain = " << gain << " beta = " << beta << endl;
}

bool TwoThirdMotionProfile::operator==(const TwoThirdMotionProfile &ttplmp){
    return ((valid==ttplmp.valid)&&(type==ttplmp.type));
}

double TwoThirdMotionProfile::computeTangVelocity(){
    //vel = g * K ^ (-beta);    beta = 0.33;
    //vel = ang.vel * r
    double reBeta = -1 * beta;
    //double curvature = 1 / radius;
    // fundamental change in the computation formula of the curvature @Rea 23/6/16
    double curvature = k;
    double vel = gain * pow(curvature, reBeta);
    //yDebug("TT:ComputeTangVelocity: beta= %fcurvature=%f tan.vel=%f", beta, curvature, vel);
    return vel;
}

double TwoThirdMotionProfile::computeCurvature(const double timeDiff, const double thetaPrev, const Vector* xprev){
    Vector xcur(3);
    Vector xder1prev(3);
    Vector xder2(3);
    double epsilon = 0.01;
    //yDebug("timeDiff %f reverse %d", timeDiff, reverse);
    double theta   =  thetaPrev + reverseSign * timeDiff *  epsilon;

    xcur[0] = O[0] + radius * cos(theta) * AOnorm[0] + radius * sin(theta) * BOnorm[0];
    xcur[1] = O[1] + radius * cos(theta) * AOnorm[1] + radius * sin(theta) * BOnorm[1];
    xcur[2] = O[2] + radius * cos(theta) * AOnorm[2] + radius * sin(theta) * BOnorm[2];
    //yDebug("epsilon = %f radius = %f",epsilon, radius);
    //yDebug("xcur:%s", xcur.toString().c_str());
    //yDebug("xprev:%s", xprev->toString().c_str());

    xder1prev = (*xder1);

    //yDebug("saved the previous derivative");

    (*xder1)[0] =(xcur[0] - (*xprev)[0]) / (timeDiff * epsilon);
    (*xder1)[1] =(xcur[1] - (*xprev)[1]) / (timeDiff * epsilon);
    (*xder1)[2] =(xcur[2] - (*xprev)[2]) / (timeDiff * epsilon);
    //yDebug("Computed the first derivative:%s", xder1->toString().c_str());

    xder2[0] =((*xder1)[0] - xder1prev[0]) / (timeDiff * epsilon);
    xder2[1] =((*xder1)[1] - xder1prev[1]) / (timeDiff * epsilon);
    xder2[2] =((*xder1)[2] - xder1prev[2]) / (timeDiff * epsilon);
    //yDebug("Computed the second derivative:%s", xder2.toString().c_str());

    double xprimysec = (*xder1)[0] * xder2[1]; // x`y''
    double yprimxsec = (*xder1)[1] * xder2[0]; // y'x''
    double xprimzsec = (*xder1)[0] * xder2[2]; // x'z''
    double zprimxsec = (*xder1)[2] * xder2[0]; // z'x''
    double yprimzsec = (*xder1)[1] * xder2[2]; // y'z''
    double zprimysec = (*xder1)[2] * xder2[1]; // z'y''
    double xprimsquare = (*xder1)[0] * (*xder1)[0]; //x'^2
    double yprimsquare = (*xder1)[1] * (*xder1)[1]; //y'^2
    double zprimsquare = (*xder1)[2] * (*xder1)[2]; //z'^2
    double a = (xprimysec - yprimxsec) * (xprimysec - yprimxsec); // (x'y'' - y'x'')
    double b = (zprimxsec - xprimzsec) * (zprimxsec - xprimzsec); // (z'x'' - x'z'')
    double c = (yprimzsec - zprimysec) * (yprimzsec - zprimysec); // (y'z'' - z'y'')
    double d = sqrt(a + b + c);
    double e = sqrt(pow(xprimsquare +  yprimsquare + zprimsquare,3));
    double k = e / d;
    //yDebug("curvature = %f", k);

    return k;
}

void TwoThirdMotionProfile::preComputation(const double  t, const double theta){

    radius = computeRadius(theta);

    tanVelocity = computeTangVelocity();

    //this is the major change in the computation of the curvature
    angVelocity = computeAngVelFabio();

    double tanVelocity_temp = checkTanVelocity(theta);

}

double TwoThirdMotionProfile::computeAngVelFabio(){
    //////////Fabio
    return gain*pow(1/k, 1-beta);
    ////////////////
}

Vector* TwoThirdMotionProfile::compute(double t){
    double timeDiff = t - tprev;

    Vector xdes = *xd;

    if(t-t0 == 0) {
        //read starting pose
        xPrev[0]=O[0] + radius * cos(theta_start) * AOnorm[0] + radius * sin(theta_start) * BOnorm[0];
        xPrev[1]=O[1] + radius * cos(theta_start) * AOnorm[1] + radius * sin(theta_start) * BOnorm[1];
        xPrev[2]=O[2] + radius * cos(theta_start) * AOnorm[2] + radius * sin(theta_start) * BOnorm[2];
        //yDebug("initial condition of t-t0 = 0");
        theta = theta_start;
        k = 100;
        timeDiff = 0.05;
        // yDebug("timeDiff: %f", timeDiff);
    }
    else {
        //while (t-tprev < 0.05){} //wait the end of the time step
        yDebug("DELTA_TIME = %f [s]", t-tprev);

        theta =  thetaPrev + reverseSign * timeDiff * angVelocity;
        cout << "time Diff: " << timeDiff << endl;
        cout << "theta: " << theta << endl;
        cout << "thetaPrev: " << thetaPrev << endl;
        cout << "angVelocity: " << angVelocity << endl;
    }
    //yDebug("pre-computing %f with tanVelocity %f", theta, tanVelocity);
    //preComputation(t, theta);

    if (inRange(theta)) {
        // setting the attribute of the class k(curvature)
        k = computeCurvature(timeDiff ,thetaPrev,  &xPrev);
        yDebug("Computed Curvature: %f", k);
        preComputation(t, theta);

        (*xd)[0]=O[0] + radius * cos(theta) * AOnorm[0] + radius * sin(theta) * BOnorm[0];
        (*xd)[1]=O[1] + radius * cos(theta) * AOnorm[1] + radius * sin(theta) * BOnorm[1];
        (*xd)[2]=O[2] + radius * cos(theta) * AOnorm[2] + radius * sin(theta) * BOnorm[2];
        yInfo("xdes %s", xd->toString().c_str());
    }
    else {
        return NULL;
    }

    Vector distance = (*xd) - xPrev;
    //yInfo("TT:travelled distance x = %f , travelled distance y = %f absolute = %f", distance[1]/(t-tprev), distance[2]/(t-tprev),
    //sqrt(distance[1] * distance[1] + distance[2] * distance[2])/(t-tprev));

    // updating the previous incremental step values
    radiusPrev =  radius;
    thetaPrev  = theta;
    tprev      = t;
    xPrev      = (*xd);

    return xd;
}

//**********************************************************************************************************************************************

MJMotionProfile::MJMotionProfile(){
  type = "MJP";
  valid = true;
  A.resize(3);
  B.resize(3);
  C.resize(3);
  D.resize(3);
  O.resize(3);
  AO.resize(3);
  BO.resize(3);
  xPrev.resize(3);
  xd = new Vector(3);
  thetaPrev = 0;
  tprev = 0;
  radiusPrev = 0.1;
}

MJMotionProfile::MJMotionProfile(const MJMotionProfile &cvmp){
  valid = cvmp.valid;
  type  = cvmp.type;
  A.resize(3);
  B.resize(3);
  C.resize(3);
  D.resize(3);
  O.resize(3);
  AO.resize(3);
  BO.resize(3);
  xPrev.resize(3);
  xd = new Vector(3);
  thetaPrev = 0;
  tprev = 0;
  radiusPrev = 0.1;
}

MJMotionProfile::~MJMotionProfile(){
  delete xd;
}

MJMotionProfile::MJMotionProfile(const Bottle& bInit){
  type = "MJP";
  valid = true;
  A.resize(3);
  B.resize(3);
  C.resize(3);
  D.resize(3);
  O.resize(3);
  AO.resize(3);
  BO.resize(3);
  xPrev.resize(3);
  xd = new Vector(3);
  thetaPrev = 0;
  tprev = 0;
  radiusPrev = 0.1;

  Bottle* b = bInit.get(0).asList();
  //read all the parameters and return the custom parameters that are specific for this class
  yarp::sig::Vector paramVector = InitializeParameters(b);
  // in the final implementation the param vector will contain the execution time (seconds)
  cout << "Minimum Jerk Profile:" << endl;
  yDebug("WARNING! this profile is not yet implemented, it fixes theta_end = 1.57 rad (90??) and execution time = 5 seconds");

}

bool MJMotionProfile::operator==(const MJMotionProfile &cvmp){
  return ((valid==cvmp.valid)&&(type==cvmp.type));
}

double MJMotionProfile::computeTangVelocity(const double t){
  //v = g * K ^ (-beta);    beta = 0.33;
  //double reBeta = -1 * beta;
  //double curvature = 1 / radius;
  thetaGoal = 1.57; //theta Goal is fixed, we should change it to a parameter
  double goalRadius = computeRadius(thetaGoal);
  Vector goal(3);
  goal[0]=O[0] + goalRadius * cos(thetaGoal) * AOnorm[0] + goalRadius * sin(thetaGoal) * BOnorm[0];
  goal[1]=O[1] + goalRadius * cos(thetaGoal) * AOnorm[1] + goalRadius * sin(thetaGoal) * BOnorm[1];
  goal[2]=O[2] + goalRadius * cos(thetaGoal) * AOnorm[2] + goalRadius * sin(thetaGoal) * BOnorm[2];
  Vector distanceVect = goal - (*xd);
  //yDebug("distanceVector: %s", distanceVect.toString().c_str());
  double distance = sqrt(pow(distanceVect[0],2) + pow(distanceVect[1],2) + pow(distanceVect[2],2));
  //yDebug("distance:%f", distance);
  double epsilon = (t - t0) / (tfinal - t0);
  //yDebug("epsilon:%f t:%f tfinal:%f", epsilon, t, tfinal);
  double epsilon3 = epsilon * epsilon * epsilon;
  double epsilon4 = epsilon3 * epsilon;
  double epsilon5 = epsilon4 * epsilon;
  double vel = distance * (15 * epsilon4  - 6 * epsilon5  - 10 * epsilon3);
  return vel;
}

void MJMotionProfile::preComputation(const double t, const double theta){
  //angular velocity expressed in frequency [Hz],
  //e.g.: 0.1 => 1/10 of 2PI per second; 2PI in 10sec
  //e.g.: 0.2 => 1/20 of 2PI per second; 2PI in 20sec
  //ONLY FOR CIRCLES: angular velocity constant to obtain constant tang.velocity
  //FOR ELLIPSE: compute angular velocity from A,B and desired tang.velocity
  radius = computeRadius(theta);
  tanVelocity = computeTangVelocity(t);
  //yInfo("computed radius %f [m] for tangVelocity %f [m/s]", radius, tanVelocity);
  // computing angular velocity in function of the radius, theta, tangential velocity
  angVelocity = computeAngVelocity(theta);
  //yInfo("computed angular velocity %f in rad/s", angVelocity);
  double tanVelocity_temp = checkTanVelocity(theta);
  //yInfo("computed tang velocity %f in m/s", tanVelocity_temp);
  //double theta = 2.0 * M_PI * angVelocity * (t - t0);
  //yInfo("theta angle [rad] %f", theta);
}

Vector* MJMotionProfile::compute(double t){
    if(t-t0 == 0) {
        theta = theta_start;
        tfinal = t0 + 5.0; //execution time is fixed, we should make it a parameter @Luca
    }
    else {
        theta =  thetaPrev + (t - tprev) * angVelocity;
    }

    Vector xdes = *xd;

    if (inRange(theta)) {
        preComputation(t, theta);
        (*xd)[0]=O[0] + radius * cos(theta) * AOnorm[0] + radius * sin(theta) * BOnorm[0];
        (*xd)[1]=O[1] + radius * cos(theta) * AOnorm[1] + radius * sin(theta) * BOnorm[1];
        (*xd)[2]=O[2] + radius * cos(theta) * AOnorm[2] + radius * sin(theta) * BOnorm[2];

        yInfo("xdes %s", xd->toString().c_str());
    }
    else {
        return nullptr;
    }

    Vector distance = (*xd) - xPrev;
    yInfo("travelled distance x = %f , travelled distance y = %f absolute = %f", distance[1]/(t-tprev), distance[2]/(t-tprev),
    sqrt(distance[1] * distance[1] + distance[2] * distance[2])/(t-tprev));

    // updating the previous incremental step values
    radiusPrev =  radius;
    thetaPrev = theta;
    tprev     = t;
    xPrev = (*xd);
    return xd;

}

//**********************************************************************************************************************************************

GVPMotionProfile::GVPMotionProfile(){
    type = "GVP";
    valid = true;
    A.resize(3);
    B.resize(3);
    C.resize(3);
    D.resize(3);
    O.resize(3);
    AO.resize(3);
    BO.resize(3);
    xPrev.resize(3);
    xd = new Vector(3);
    thetaPrev = 0;
    tprev = 0;
    radiusPrev = 0.1;
}

GVPMotionProfile::GVPMotionProfile(const GVPMotionProfile &gvmp){
  valid = gvmp.valid;
  type  = gvmp.type;
  A.resize(3);
  B.resize(3);
  C.resize(3);
  D.resize(3);
  O.resize(3);
  AO.resize(3);
  BO.resize(3);
  xPrev.resize(3);
  xd = new Vector(3);
  thetaPrev = 0;
  tprev = 0;
  radiusPrev = 0.1;
}

GVPMotionProfile::~GVPMotionProfile(){
    delete xd;
}

GVPMotionProfile::GVPMotionProfile(const Bottle& bInit) {
  type = "GVP";
  valid = true;
  A.resize(3);
  B.resize(3);
  C.resize(3);
  D.resize(3);
  O.resize(3);
  AO.resize(3);
  BO.resize(3);
  xPrev.resize(3);
  xd = new Vector(3);
  thetaPrev = 0;
  tprev = 0;
  step_counter = 0; //index to iterate inside the velocity vector
  radiusPrev = 0.1;

  Bottle* b = bInit.get(0).asList();

  //read all the parameters and return the custom parameters that are specific for this class
  yarp::sig::Vector velVector = InitializeParameters(b);

  setVelocity(velVector); // set the velocity generic velocity profile, in  the other profiles this is a different parameter
  cout << "General Velocity Profile: velocity samples = " << velVector.size() << endl;
}

bool GVPMotionProfile::operator==(const GVPMotionProfile &gvmp){
  return ((valid==gvmp.valid)&&(type==gvmp.type));
}

void GVPMotionProfile::preComputation(const double t, const double theta){

  radius = computeRadius(theta);

  yInfo("computed radius %f [m] for tangVelocity %f [m/s]", radius, tanVelocity);

  //IMPOSE THE TANGENTIAL VELOCITY AND COMPUTE THE CORRESPONDING ANGULAR VELOCITY

  // iteratively read the velocity profile and impose the tang velocity
  tanVelocity = velocityProfile[step_counter];

  // increment the step counter
  if (step_counter < velocityProfile.size()-1) {step_counter++;}

  //evaluate corresponding angular velocity
  angVelocity = tanVelocity / radius;

  yInfo("computed angular velocity %f in rad/s", angVelocity);
  double tanVelocity_temp = checkTanVelocity(theta);

  yInfo("desired  tang velocity %f in m/s", tanVelocity);
  yInfo("computed tang velocity %f in m/s", tanVelocity_temp);
  yInfo("theta angle [rad] %f", theta);
}

Vector* GVPMotionProfile::compute(double t){

  yDebug("****************** STEP %i/%li **********************", step_counter, velocityProfile.size());

  const float delta_time = 0.01; // [s]

  if(t-t0 == 0){
      step_counter = 0;
      theta = theta_start;
  }
  else {
      while (t-tprev < delta_time && t-t0 != 0){t=Time::now();} //wait the end of the time step
      yDebug("DELTA_TIME = %f [s]", t-tprev);

      theta =  thetaPrev + reverseSign * (t-tprev) * angVelocity;
  }

  Vector xdes = *xd; //perch?? dichiarato qui ? @Luca
  if (inRange(theta)) {
      preComputation(t, theta); //given theta, evaluate radius and ang vel

      (*xd)[0] = O[0] + radius * cos(theta) * AOnorm[0] + radius * sin(theta) * BOnorm[0];
      (*xd)[1] = O[1] + radius * cos(theta) * AOnorm[1] + radius * sin(theta) * BOnorm[1];
      (*xd)[2] = O[2] + radius * cos(theta) * AOnorm[2] + radius * sin(theta) * BOnorm[2];

      yInfo("xdes %s", xd->toString().c_str());
  }
  else {
      return nullptr;
  }

  //yDebug("STEP = %i/%li", step_counter, velocityProfile.size());
  yDebug("TRAVELLED DISTANCE = %f", eval_distance(*xd, xPrev));
  yDebug("ESTIMATED VELOCITY (@LUCA) = %f", eval_distance(*xd, xPrev)/(t-tprev));

  // updating the previous incremental step values
  radiusPrev = radius;
  thetaPrev  = theta;
  tprev      = t;
  xPrev      = (*xd);
  return xd;
}

// to be removed
Matrix GVPMotionProfile::offline_compute(yarp::dev::ICartesianControl *icart, yarp::dev::IEncoders *encs) {
    yInfo("OFFLINE JOINTS TRAJECTORY COMPUTATION FOR GVP CLASS");
    std::ofstream myfile ("Velocity_Report.csv");
    int steps = velocityProfile.size();
    const float delta_time = 0.01; //time interval between trajectory points 10 ms
    double thetaStart = 0;
    angVelocity = 0;
    theta = 0;
    thetaPrev = 0;
    step_counter = 0;

    // hand pose and orientation, final(hat) may differ from the commanded desired
    yarp::sig::Vector xd(3),xd_final(3),od(4),od_final(4);
    yarp::sig::Vector x_dot(3),o_dot(4);

    // joints configuration
    yarp::sig::Vector q_start(10),q_final(10);  //3 values for the torso + 7 for the arm

    // Trajectory, matrix containing the joints config at each time step
    yarp::sig::Matrix trajectory(steps,10); //3 values for the torso + 7 for the arm
    // Velcoity Matrix, matrix containing the joints velcoity at each time step
    yarp::sig::Matrix velocity(steps,3); //3 values for X,Y,Z components of the velocity

    //get actual starting pose [JOINTS]
//    yInfo("GETTING STARTING JOINT POSITION");
//    Vector arm_pose(7);
//    encs->getEncoders(arm_pose.data()); // read arm pose
//    q_start.setSubvector(3,arm_pose); //append arm pose to the torso pose (zeros)
//    //get actuol pose [CARTESIAN]
//    icart->getPose(xd,od);

    //PRINT Q_START
    yInfo("q_start: %s",q_start.toString().c_str());

    //we don't care about the hand orientation
    icart->setPosePriority("position");

    while(inRange(theta)){
        auto time = Time::now();
        icart->getTaskVelocities(x_dot,o_dot);
        velocity.setRow(step_counter,x_dot);

        if (step_counter%50 == 0 && step_counter != steps) {
            yInfo("STEP = %i/%i", step_counter, steps);
        }

        //read the desired velocity
        tanVelocity = velocityProfile[step_counter];
        //knowing the angle theta, evaluate radius
        radius = computeRadius(theta);
        //convert tangential velocty to angular velocity
        angVelocity = tanVelocity / radius;
        //compute the angle of the next point to reach
        theta =  thetaPrev + reverseSign * (delta_time) * angVelocity;

        // evaluate the corresponding cartesian point of the trajectory
        (xd)[0] = O[0] + radius * cos(theta) * AOnorm[0] + radius * sin(theta) * BOnorm[0];
        (xd)[1] = O[1] + radius * cos(theta) * AOnorm[1] + radius * sin(theta) * BOnorm[1];
        (xd)[2] = O[2] + radius * cos(theta) * AOnorm[2] + radius * sin(theta) * BOnorm[2];

        //ask the solver to compute the joint angles that allow to reach that position
        //note that the solver takes into account the prevoious position of the robot
        //////////////icart->askForPose(q_start,xd,od,xd_final,od_final,q_final);

        //print q_final
        //yInfo("q_final: %s",q_final.toString().c_str());

        icart->goToPose(xd,od);

//        cout << "Movement performed with Cartesian Control" << endl;
//        while(!motionDone){
//            icart->checkMotionDone(&motionDone);
//            Time::delay(0.005);
//        }

        //starting joint position, CART desired pos+ori, CART target pos+ori, final joint config.

        //add joints config to the trajectory
        ////////////////trajectory.setRow(step_counter, q_final);

        // update the variables
        radiusPrev = radius;
        thetaPrev  = theta;
        q_start = q_final;
        step_counter++;
        //print time
        while(Time::now() - time < delta_time){
            Time::delay(0.001);
        }
        yInfo("DELTA TIME: %f",Time::now()-time);
    }
    yInfo("STEP = %i/%i", step_counter, steps);

    /////////////////////// clean the trajectory ///////////////////
    // remove the last rows of the trajectory matrix that have not beeen written
    trajectory.resize(step_counter, 10);
    // remove first columns dedicated to the torso that is not actuated
    trajectory.removeCols(0, 3);
    //set the hand joints 5 6 7 to a constant position
//    for (int i=5;i<7;i++){
//        trajectory.setCol(i,0);
//    }
    ///////////////////////////////////////////////////////////////
    yInfo("PRINTING VELOCITY REPORT ON FILE");
    myfile << velocity.toString().c_str() << std::endl;
    myfile.close();

    return trajectory;
}
