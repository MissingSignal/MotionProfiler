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

MotionProfile::MotionProfile() : valid(false), type(""){

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
    if(reverse == true) {
        yInfo("Setting for REVERSE motion");
        theta_stop = t_stop;
        theta_start = t_start;
    }
    else {
      yInfo("Setting for FORWARD motion");
      theta_stop = t_start;
      theta_stop = t_stop;
    }
}

Vector MotionProfile::normalizeVector(const Vector u, double const radius){
    double module = sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
    yDebug("module %f", module);
    Vector unorm(3);
    if(module != 0) {
        unorm[0] = (u[0] / module);
        unorm[1] = (u[1] / module);
        unorm[2] = (u[2] / module);
    }
    return unorm;
}

void MotionProfile::setViaPoints(const Vector _A, const Vector _B){
    yInfo("MotionProfile::setViaPoints");
    A = _A;
    B = _B;

    // scaled by the radius in A
    AO[0] = A[0] - O[0];
    AO[1] = A[1] - O[1];
    AO[2] = A[2] - O[2];
    AOnorm = normalizeVector(AO, 0.1);

    yInfo("AO = %s", AO.toString().c_str());
    yInfo("AOnorm = %s scaled by radius: %f", AOnorm.toString().c_str(), xAxis);

    //scaled by the radius in B
    BO[0] = B[0] - O[0];
    BO[1] = B[1] - O[1];
    BO[2] = B[2] - O[2];
    BOnorm = normalizeVector(BO, 0.2);

    yInfo("BO = %s", BO.toString().c_str());
    yInfo("BOnorm = %s scaled by the radius %f", BOnorm.toString().c_str(), yAxis);

    Vector N(3);
    N = cross(AO, BO);

    yInfo("NormVector = %s", N.toString().c_str());

    double revA2   = 1 / (xAxis * xAxis);
    double revB2   = 1 / (yAxis * yAxis);
    subA2B2 = revA2 - revB2;

    // POINT C is automatically evaluated
    C[0] = O[0] + xAxis * -1 * AOnorm[0] ;
    C[1] = O[1] + xAxis * -1 * AOnorm[1] ;
    C[2] = O[2] + xAxis * -1 * AOnorm[2] ;
    yInfo("C has coordinates:(%s)", C.toString().c_str());

    // POINT D is automatically evaluated
    D[0] = O[0] + yAxis * -1 * BOnorm[0];
    D[1] = O[1] + yAxis * -1 * BOnorm[1];
    D[2] = O[2] + yAxis * -1 * BOnorm[2];
    yInfo("D has coordinates:(%s)", D.toString().c_str());

    //ORIGINAL FORMULA
    //(*xd)[0] = O[0] + radius * cos(theta) * AOnorm[0] + radius * sin(theta) * BOnorm[0];
    //(*xd)[1] = O[1] + radius * cos(theta) * AOnorm[1] + radius * sin(theta) * BOnorm[1];
    //(*xd)[2] = O[2] + radius * cos(theta) * AOnorm[2] + radius * sin(theta) * BOnorm[2];

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
    yInfo("getting the initial position %s for thetaStart %f with radius %f", locInitial.toString().c_str(), theta_start, radius);
    return locInitial;
}

bool MotionProfile::inRange(double theta){
    //reverseGain = 1 when reverse = FALSE
    if(reverse == 1) {
        if((theta > theta_start) && (theta < theta_stop))
            return true;
        else
            return false;
    }
    //reverseGain = -1 when reverse = TRUE
    else {
       if((theta > theta_stop) && (theta < theta_start))
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
    ResourceFinder rf;
    rf.setVerbose(true);
	//fix: max size would be 8 * 2 + 1; round it to 20 @amaroyo 18/01/2016
    //int argc = b->size() * 2 + 1;
	// fix:
	const int argc = 20;
    string stringArray[argc];
    char* argv[argc];
    stringArray[0].append("./motionProfile");
    argv[0] = (char*) stringArray[0].c_str();
    //yDebug("added first %s", argv[0]);

    for (int j = 0; j < b->size(); j++) {
        Bottle* vector = b->get(j).asList();
        stringArray[j * 2 + 1].append("--");
        //stringArray[j * 2 + 1].append("A");
        stringArray[j * 2 + 1].append(vector->get(0).asString().c_str());
        char temp[50];
        sprintf(temp,"%f %f %f", vector->get(1).asFloat64(), vector->get(2).asFloat64(), vector->get(3).asFloat64());
        yDebug("stringArray %s", stringArray[j * 2 + 1].c_str());
        stringArray[j * 2 + 2].append(&temp[0]);
        argv[j * 2 + 1] = (char*) stringArray[j * 2 + 1].c_str();
        argv[j * 2 + 2] = (char*) stringArray[j * 2 + 2].c_str();
        //yDebug("param %d %s", j, argv[j * 2 + 1]);
        //yDebug("value %s", argv[j * 2 + 2]);
    }
    yDebug("parsing ");
    yDebug("%s",argv[0] );
    yDebug("%s, %s",argv[1], argv[2] );
    // configuring the resource finder
    rf.configure(b->size() * 2 + 1, argv);
    yInfo("resorceFinder: %s",rf.toString().c_str());
    // visiting the parameters using the RF
    Vector Ovector(3);
    Vector Avector(3);
    Vector Bvector(3);
    string Ostring = rf.check("O",
                           Value("-0.3 -0.1 0.1"),
                           "position O (string)").asString();
    extractVector(Ostring, Ovector);
    yDebug("got O value %s", Ovector.toString().c_str());
    string Astring = rf.check("A",
                           Value("-0.3 0.0 0.1"),
                           "position A (string)").asString();
    extractVector(Astring, Avector);
    yDebug("got A value %s", Avector.toString().c_str());
    string Bstring = rf.check("B",
                           Value("-0.3 -0.1 0.3"),
                           "position B (string)").asString();
    extractVector(Bstring, Bvector);
    yDebug("got B value %s", Bvector.toString().c_str());
    Vector thetaVector(2);
    string  thetaString = rf.check("theta",
                           Value("0 1.57"),
                           "theta angles (string)").asString();
    extractVector(thetaString, thetaVector);
    yDebug("got theta angles:(%s)", thetaVector.toString().c_str());
    Vector axisVector(2);
    string  axisString = rf.check("axes",
                           Value("0.1 0.2"),
                           "minor and major axes (string)").asString();
    extractVector(axisString, axisVector);
    yDebug("got minor and major axes:(%s)", axisVector.toString().c_str());
    Vector paramVector(1);
    string  paramString = rf.check("param",
                           Value("0.1"),
                           "profile parameters (string)").asString();
    extractVector(paramString, paramVector);
    yDebug("got profile parameters:(%s)", paramVector.toString().c_str());
    bool reverse = rf.check("rev");
    reverse? yDebug("reverse is ON") : yDebug("reverse is OFF");

    setCenter(Ovector);
    setReverse(reverse);
    setStartStop(thetaVector[0], thetaVector[1]);
    setAxes(axisVector[0], axisVector[1]);

    if(pointsAreReachable()) {
      valid = true;
      setViaPoints(Avector,Bvector);
    }
    else {yError("Error positions are not allowed!");}
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

    yInfo("computed radius %f [m] for tangVelocity %f [m/s]", radius, tanVelocity);
    // computing angular velocity in function of the radius, theta, tangential velocity
    angVelocity = computeAngVelocity(theta);
    yInfo("computed angular velocity %f in rad/s", angVelocity);
    double tanVelocity_temp = checkTanVelocity(theta);
    yInfo("computed tang velocity %f in m/s", tanVelocity_temp);
    //double theta = 2.0 * M_PI * angVelocity * (t - t0);
    yInfo("theta angle [rad] %f", theta);
}

Vector* CVMotionProfile::compute(double t){
    double thetaStart;
    if(t-t0 == 0) {
        theta = theta_start;
        thetaStart =  theta_start;
    }
    else {
        theta =  thetaPrev + reverse * (t - tprev) * angVelocity;
    }

    Vector xdes = *xd;
    if(theta == thetaStart) {
        yInfo("theta=thetaStart check");
        preComputation(t, theta);
        //(*xd)[0]=O[0] + xAxis * cos(theta) * AO[0] + yAxis * sin(theta) * BO[0];
        //(*xd)[1]=O[1] + xAxis * cos(theta) * AO[1] + yAxis * sin(theta) * BO[1];
        //(*xd)[2]=O[2] + xAxis * cos(theta) * AO[2] + yAxis * sin(theta) * BO[2];
        (*xd)[0]=O[0] + radius * cos(theta) * AOnorm[0] + radius * sin(theta) * BOnorm[0];
        (*xd)[1]=O[1] + radius * cos(theta) * AOnorm[1] + radius * sin(theta) * BOnorm[1];
        (*xd)[2]=O[2] + radius * cos(theta) * AOnorm[2] + radius * sin(theta) * BOnorm[2];
        if( ((*xd)[0]!=A[0]) || ((*xd)[1]!=A[1]) || ((*xd)[2]!=A[2]) ){
            yInfo("Beware: difference between desired location A and computed position!");
            yInfo("vector xdes: %s", xd->toString().c_str());
            //Time::delay(5.0);
        }
    }
    else if (inRange(theta) /*(theta > theta_start) && (theta<=theta_stop)*/) {
        yInfo("In the range xAxis:%f yAxis:%f", xAxis,yAxis);
        preComputation(t, theta);
        //(*xd)[0]=O[0] + xAxis * cos(theta) * AO[0] + yAxis * sin(theta) * BO[0];
        //(*xd)[1]=O[1] + xAxis * cos(theta) * AO[1] + yAxis * sin(theta) * BO[1];
        //(*xd)[2]=O[2] + xAxis * cos(theta) * AO[2] + yAxis * sin(theta) * BO[2];
        (*xd)[0]=O[0] + radius * cos(theta) * AOnorm[0] + radius * sin(theta) * BOnorm[0];
        (*xd)[1]=O[1] + radius * cos(theta) * AOnorm[1] + radius * sin(theta) * BOnorm[1];
        (*xd)[2]=O[2] + radius * cos(theta) * AOnorm[2] + radius * sin(theta) * BOnorm[2];

        yInfo("xdes %s", xd->toString().c_str());
    }
    else {
        return NULL;
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
    ResourceFinder rf;
    rf.setVerbose(true);
	//fix: max size would be 8 * 2 + 1; round it to 20 @amaroyo 18/01/2016
  //int argc = b->size() * 2 + 1;   //@JONAS: made the argc variable 11/9/17
	//fix:
	  const int argc = 20;
    string stringArray[argc];
    char* argv[argc];
    stringArray[0].append("./motionProfile");
    argv[0] = (char*) stringArray[0].c_str();
    yDebug("added first %s", argv[0]);

    for (int j = 0; j < b->size(); j++) {
        Bottle* vector = b->get(j).asList();
        stringArray[j * 2 + 1].append("--");
        //stringArray[j * 2 + 1].append("A");
        stringArray[j * 2 + 1].append(vector->get(0).asString().c_str());
        char temp[50];
        yDebug("%f %f %f",vector->get(1).asFloat64(), vector->get(2).asFloat64(), vector->get(3).asFloat64());
        sprintf(temp,"%f %f %f", vector->get(1).asFloat64(), vector->get(2).asFloat64(), vector->get(3).asFloat64());

        stringArray[j * 2 + 2].append(&temp[0]);
        yDebug("string: %s %s", stringArray[j * 2 + 1].c_str(), stringArray[j * 2 + 2].c_str());
        argv[j * 2 + 1] = (char*) stringArray[j * 2 + 1].c_str();
        argv[j * 2 + 2] = (char*) stringArray[j * 2 + 2].c_str();
        yDebug("param %s", argv[j * 2 + 1]);
        yDebug("value %s", argv[j * 2 + 2]);
    }
    yDebug("parsing......");
    yDebug("argc %d argv %s", argc, argv[0]);
    // configuring the resource finder
    rf.configure(argc, argv);

    yInfo("resorceFinder: %s",rf.toString().c_str());
    // visiting the parameters using the RF
    Vector Ovector(3);
    Vector Avector(3);
    Vector Bvector(3);
    string Ostring = rf.check("O",
                           Value("-0.3 -0.1 0.1"),
                           "position O (string)").asString();
    extractVector(Ostring, Ovector);
    yDebug("got O value %s", Ovector.toString().c_str());
    string  Astring = rf.check("A",
                           Value("-0.3 0.0 0.1"),
                           "position A (string)").asString();
    extractVector(Astring, Avector);
    yDebug("got A value %s", Avector.toString().c_str());
    string  Bstring = rf.check("B",
                           Value("-0.3 -0.1 0.2"),
                           "position B (string)").asString();
    extractVector(Bstring, Bvector);
    yDebug("got B value %s", Bvector.toString().c_str());
    Vector thetaVector(2);
    string  thetaString = rf.check("theta",
                           Value("0 1.57"),
                           "theta angles (string)").asString();
    extractVector(thetaString, thetaVector);
    yDebug("got theta angles:(%s)", thetaVector.toString().c_str());
    Vector axisVector(2);
    string  axisString = rf.check("axes",
                           Value("0.1 0.2"),
                           "minor and major axes (string)").asString();
    extractVector(axisString, axisVector);
    yDebug("got minor and major axes:(%s)", axisVector.toString().c_str());
    Vector paramVector(1);
    string  paramString = rf.check("param",
                           Value("0.1"),
                           "profile parameters (string)").asString();
    extractVector(paramString, paramVector);
    yDebug("got profile parameters:(%s)", paramVector.toString().c_str());
    bool reverse = rf.check("rev");
    reverse? yDebug("reverse is ON") : yDebug("reverse is OFF");

    setCenter(Ovector);
    setReverse(reverse);
    setStartStop(thetaVector[0], thetaVector[1]);
    setAxes(axisVector[0], axisVector[1]);
    setGain(paramVector[0]);
    setBeta(paramVector[1]);

    if(pointsAreReachable()) {
      valid = true;
      setViaPoints(Avector,Bvector);
    }
    else {yError("Error positions are not allowed!");}
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
    yInfo("ComputeTangVelocity: beta= %fcurvature=%f tan.vel=%f", beta, curvature, vel);
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
    double thetaStart;

    if(t-t0 == 0) {
        theta = theta_start;
        thetaStart =  theta_start;
    }
    else {
        theta =  thetaPrev + reverse * (t - tprev) * angVelocity;
    }

    Vector xdes = *xd;
    if(theta == thetaStart) {
        yInfo("theta=thetaStart check");
        preComputation(t, theta);
        //(*xd)[0]=O[0] + xAxis * cos(theta) * AO[0] + yAxis * sin(theta) * BO[0];
        //(*xd)[1]=O[1] + xAxis * cos(theta) * AO[1] + yAxis * sin(theta) * BO[1];
        //(*xd)[2]=O[2] + xAxis * cos(theta) * AO[2] + yAxis * sin(theta) * BO[2];
        (*xd)[0]=O[0] + radius * cos(theta) * AOnorm[0] + radius * sin(theta) * BOnorm[0];
        (*xd)[1]=O[1] + radius * cos(theta) * AOnorm[1] + radius * sin(theta) * BOnorm[1];
        (*xd)[2]=O[2] + radius * cos(theta) * AOnorm[2] + radius * sin(theta) * BOnorm[2];
        if( ((*xd)[0]!=A[0]) || ((*xd)[1]!=A[1]) || ((*xd)[2]!=A[2]) ){
            yInfo("Beware: difference between desired location A and computed position!");
            yInfo("vector xdes: %s", xd->toString().c_str());
            //Time::delay(5.0);
        }
    }
    else if (inRange(theta)) {
        yInfo("In the range xAxis:%f yAxis:%f", xAxis,yAxis);
        preComputation(t, theta);
        //(*xd)[0]=O[0] + xAxis * cos(theta) * AO[0] + yAxis * sin(theta) * BO[0];
        //(*xd)[1]=O[1] + xAxis * cos(theta) * AO[1] + yAxis * sin(theta) * BO[1];
        //(*xd)[2]=O[2] + xAxis * cos(theta) * AO[2] + yAxis * sin(theta) * BO[2];
        (*xd)[0]=O[0] + radius * cos(theta) * AOnorm[0] + radius * sin(theta) * BOnorm[0];
        (*xd)[1]=O[1] + radius * cos(theta) * AOnorm[1] + radius * sin(theta) * BOnorm[1];
        (*xd)[2]=O[2] + radius * cos(theta) * AOnorm[2] + radius * sin(theta) * BOnorm[2];
        yInfo("xdes %s", xd->toString().c_str());
    }
    else {
        return NULL;
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
    ResourceFinder rf;
    rf.setVerbose(true);
	//fix: max size would be 8 * 2 + 1; round it to 20 @amaroyo 18/01/2016
    //int argc = b->size() * 2 + 1;
	//fix:
	const int argc = 20;
    string stringArray[argc];
    char* argv[argc];
    stringArray[0].append("./motionProfile");
    argv[0] = (char*) stringArray[0].c_str();
    //yDebug("added first %s", argv[0]);
    //yDebug("number of elements %d", b->size());

    for (int j = 0; j < b->size(); j++) {
        Bottle* vector = b->get(j).asList();
        stringArray[j * 2 + 1].append("--");
        //stringArray[j * 2 + 1].append("A");
        stringArray[j * 2 + 1].append(vector->get(0).asString().c_str());
        char temp[50];
        //yDebug("%f %f %f",vector->get(1).asFloat64(), vector->get(2).asFloat64(), vector->get(3).asFloat64());
        sprintf(temp,"%f %f %f", vector->get(1).asFloat64(), vector->get(2).asFloat64(), vector->get(3).asFloat64());

        stringArray[j * 2 + 2].append(&temp[0]);
        //yDebug("string: %s %s", stringArray[j * 2 + 1].c_str(), stringArray[j * 2 + 2].c_str());
        argv[j * 2 + 1] = (char*) stringArray[j * 2 + 1].c_str();
        argv[j * 2 + 2] = (char*) stringArray[j * 2 + 2].c_str();
        //yDebug("param %d %s",j , argv[j * 2 + 1]);
        //yDebug("value %s", argv[j * 2 + 2]);
    }
    yDebug("parsing......");
    //yDebug("argc %d argv %s", b->size(), argv[0]);
    //for (int j = 0; j < b->size() * 2 + 1; j++) {
        //yDebug("argv %s", argv[j]);
    //}
    // configuring the resource finder
    yDebug("success %d", rf.configure(b->size() * 2 + 1, argv));

    yInfo("resorceFinder: %s",rf.toString().c_str());
    // visiting the parameters using the RF
    Vector Ovector(3);
    Vector Avector(3);
    Vector Bvector(3);
    string Ostring = rf.check("O",
                           Value("-0.3 -0.1 0.1"),
                           "position O (string)").asString();
    extractVector(Ostring, Ovector);
    yDebug("got O value %s", Ovector.toString().c_str());
    string  Astring = rf.check("A",
                           Value("-0.3 0.0 0.1"),
                           "position A (string)").asString();
    extractVector(Astring, Avector);
    yDebug("got A value %s", Avector.toString().c_str());
    string  Bstring = rf.check("B",
                           Value("-0.3 -0.1 0.2"),
                           "position B (string)").asString();
    extractVector(Bstring, Bvector);
    yDebug("got B value %s", Bvector.toString().c_str());
    // TO DO: Si puo ripulire, calcoliamolo in setAxes @LUCA
    Vector axisVector(2);
    axisVector[0] = eval_distance(Avector,Ovector); //AO Axis
    axisVector[1] = eval_distance(Bvector,Ovector); //BO Axis

    Vector thetaVector(2);
    string  thetaString = rf.check("theta",
                           Value("0 6.28"),
                           "theta angles (string)").asString();
    extractVector(thetaString, thetaVector);
    yDebug("got angles (rad): theta_start: %f theta_stop: %f", thetaVector[0], thetaVector[1]);
    Vector paramVector(1);
    string  paramString = rf.check("param",
                           Value("0.1"),
                           "profile parameters (string)").asString();
    extractVector(paramString, paramVector);
    yDebug("got profile parameters:(%s)", paramVector.toString().c_str());
    bool reverse = rf.check("rev");
    reverse? yDebug("reverse is ON") : yDebug("reverse is OFF");

    setCenter(Ovector);
    setReverse(reverse);
    setStartStop(thetaVector[0], thetaVector[1]);
    setAxes(axisVector[0], axisVector[1]);
    setGain(paramVector[0]);
    setBeta(paramVector[1]);

    if(pointsAreReachable()) {
      valid = true;
      setViaPoints(Avector,Bvector);
    }
    else {yError("Error positions are not allowed!");}
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
    double theta   =  thetaPrev + reverse * timeDiff *  epsilon;

    xcur[0] = O[0] + radius * cos(theta) * AOnorm[0] + radius * sin(theta) * BOnorm[0];
    xcur[1] = O[1] + radius * cos(theta) * AOnorm[1] + radius * sin(theta) * BOnorm[1];
    xcur[2] = O[2] + radius * cos(theta) * AOnorm[2] + radius * sin(theta) * BOnorm[2];
    //yDebug("epsilon = %f radius = %f",epsilon, radius);
    //yDebug("xcur:%s", xcur.toString().c_str());
    //yDebug("xprev:%s", xprev->toString().c_str());

    xder1prev[0] = (*xder1)[0];
    xder1prev[1] = (*xder1)[1];
    xder1prev[2] = (*xder1)[2];

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

    ///////////////// Fabio
    //if (dataPort.getOutputCount()) {
        Bottle& dataBottle = dataPort.prepare();
        dataBottle.clear();
        dataBottle.addFloat64(xcur[0]); dataBottle.addFloat64(xcur[1]); dataBottle.addFloat64(xcur[2]); //position
        dataBottle.addFloat64((*xder1)[0]); dataBottle.addFloat64((*xder1)[1]); dataBottle.addFloat64((*xder1)[2]); //velocity
        dataBottle.addFloat64( xder2[0]); dataBottle.addFloat64( xder2[1]); dataBottle.addFloat64( xder2[2]); //acceleration
        dataBottle.addFloat64(d); dataBottle.addFloat64(e); dataBottle.addFloat64(k); //d, e, k

        //dataPort.setEnvelope(ts);
        dataPort.write();
    //}
    ///////////////////////////
    return k;
}

void TwoThirdMotionProfile::preComputation(const double  t, const double theta){
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
    //yDebug("TT:computed radius %f [m] for tangVelocity %f [m/s]", radius, tanVelocity);
    // computing angular velocity in function of the radius, theta, tangential velocity
    angVelocity = computeAngVelocity(theta);
    double newAngVelocity = computeAngVelFabio();
    angVelocity = newAngVelocity;
    //yDebug("-----------old: %f ---------new: %f", angVelocity, newAngVelocity);
    //yDebug("TT:computed angular velocity %f in rad/s", angVelocity);
    double tanVelocity_temp = checkTanVelocity(theta);
    //yDebug("TT:computed tangVelocity %f k %f r %f", tanVelocity, k, radius);
    //double theta = 2.0 * M_PI * angVelocity * (t - t0);
    //yDebug("TT:theta angle [rad] %f", theta);
}

double TwoThirdMotionProfile::computeAngVelFabio(){
    //////////Fabio
    return gain*pow(1/k, 1-beta);
    ////////////////
}

Vector* TwoThirdMotionProfile::compute(double t){
    double thetaStart;
    double timeDiff = t - tprev;

    Vector xdes = *xd;
    if(firstCompute) {
        xPrev[0]=O[0] + radius * cos(theta) * AOnorm[0] + radius * sin(theta) * BOnorm[0];
        xPrev[1]=O[1] + radius * cos(theta) * AOnorm[1] + radius * sin(theta) * BOnorm[1];
        xPrev[2]=O[2] + radius * cos(theta) * AOnorm[2] + radius * sin(theta) * BOnorm[2];

        firstCompute = false;
    }


    if(t-t0 == 0) {
        //yDebug("initial condition of t-t0 = 0");
        theta = theta_start;
        thetaStart =  theta_start;
        k = 100;
        timeDiff = 0.05;
        // yDebug("timeDiff: %f", timeDiff);
    }
    else {
        theta =  thetaPrev + reverse * timeDiff * angVelocity;
    }
    //yDebug("pre-computing %f with tanVelocity %f", theta, tanVelocity);
    //preComputation(t, theta);

    if(theta == thetaStart) {
        // setting the attribute of the class k(curvature)
        //yDebug("timeDiff: %f", timeDiff);
        k = computeCurvature(timeDiff ,thetaPrev,  &xPrev);
        k = 1;
        //yDebug("Computed Curvature: %f", k);
        //yDebug("TT:theta=thetaStart check");
        preComputation(t, theta);
        //(*xd)[0]=O[0] + xAxis * cos(theta) * AO[0] + yAxis * sin(theta) * BO[0];
        //(*xd)[1]=O[1] + xAxis * cos(theta) * AO[1] + yAxis * sin(theta) * BO[1];
        //(*xd)[2]=O[2] + xAxis * cos(theta) * AO[2] + yAxis * sin(theta) * BO[2];
        (*xd)[0]=O[0] + radius * cos(theta) * AOnorm[0] + radius * sin(theta) * BOnorm[0];
        (*xd)[1]=O[1] + radius * cos(theta) * AOnorm[1] + radius * sin(theta) * BOnorm[1];
        (*xd)[2]=O[2] + radius * cos(theta) * AOnorm[2] + radius * sin(theta) * BOnorm[2];
        if( ((*xd)[0]!=A[0]) || ((*xd)[1]!=A[1]) || ((*xd)[2]!=A[2]) ){
            yInfo("Beware: difference between desired location A and computed position!");
            yInfo("vector xdes: %s", xd->toString().c_str());
            //Time::delay(5.0);
        }
        (*xprev)[0] = (*xd)[0];
        (*xprev)[1] = (*xd)[1];
        (*xprev)[2] = (*xd)[2];

    }
    else if (inRange(theta)) {
        // setting the attribute of the class k(curvature)
        k = computeCurvature(timeDiff ,thetaPrev,  &xPrev);
        //yDebug("Computed Curvature: %f", k);
        //yInfo("In the range xAxis:%f yAxis:%f", xAxis,yAxis);
        preComputation(t, theta);
        //(*xd)[0]=O[0] + xAxis * cos(theta) * AO[0] + yAxis * sin(theta) * BO[0];
        //(*xd)[1]=O[1] + xAxis * cos(theta) * AO[1] + yAxis * sin(theta) * BO[1];
        //(*xd)[2]=O[2] + xAxis * cos(theta) * AO[2] + yAxis * sin(theta) * BO[2];
        (*xd)[0]=O[0] + radius * cos(theta) * AOnorm[0] + radius * sin(theta) * BOnorm[0];
        (*xd)[1]=O[1] + radius * cos(theta) * AOnorm[1] + radius * sin(theta) * BOnorm[1];
        (*xd)[2]=O[2] + radius * cos(theta) * AOnorm[2] + radius * sin(theta) * BOnorm[2];
        //yInfo("xdes %s", xd->toString().c_str());
        (*xprev)[0] = (*xd)[0];
        (*xprev)[1] = (*xd)[1];
        (*xprev)[2] = (*xd)[2];
    }
    else {
        return NULL;
    }


    Vector distance = (*xd) - xPrev;
    //yInfo("TT:travelled distance x = %f , travelled distance y = %f absolute = %f", distance[1]/(t-tprev), distance[2]/(t-tprev),
    //sqrt(distance[1] * distance[1] + distance[2] * distance[2])/(t-tprev));

    // updating the previous incremental step values
    radiusPrev =  radius;
    thetaPrev = theta;
    tprev     = t;
    xPrev = (*xd);

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
  yDebug("Analyzing the list of vectors from bottle b:%s", b->toString().c_str());
  ResourceFinder rf;
  rf.setVerbose(true);
  //fix: max size would be 8 * 2 + 1; round it to 20 @amaroyo 18/01/2016
  //int argc = b->size() * 2 + 1;
  const int argc = 20;
  string stringArray[argc];
  char* argv[argc];
  stringArray[0].append("./motionProfile");
  argv[0] = (char*) stringArray[0].c_str();
  yDebug("added the first %s", argv[0]);

  for (int j = 0; j < b->size(); j++) {
      Bottle* vector = b->get(j).asList();
      stringArray[j * 2 + 1].append("--");
      //stringArray[j * 2 + 1].append("A");
      stringArray[j * 2 + 1].append(vector->get(0).asString().c_str());
      char temp[50];
      sprintf(temp,"%f %f %f", vector->get(1).asFloat64(), vector->get(2).asFloat64(), vector->get(3).asFloat64());
      stringArray[j * 2 + 2].append(&temp[0]);
      argv[j * 2 + 1] = (char*) stringArray[j * 2 + 1].c_str();
      argv[j * 2 + 2] = (char*) stringArray[j * 2 + 2].c_str();
      yDebug("param %s", argv[j * 2 + 1]);
      yDebug("value %s", argv[j * 2 + 2]);
  }
  // configuring the resource finder
  //rf.configure(argc, argv);
  yDebug("success %d", rf.configure(b->size() * 2 + 1, argv));
  yInfo("resorceFinder: %s",rf.toString().c_str());
  // visiting the parameters using the RF
  Vector Ovector(3);
  Vector Avector(3);
  Vector Bvector(3);
  string Ostring = rf.check("O",
                         Value("-0.3 -0.1 0.1"),
                         "position O (string)").asString();
  extractVector(Ostring, Ovector);
  string  Astring = rf.check("A",
                         Value("-0.3 0.0 -0.1"),
                         "position A (string)").asString();
  extractVector(Astring, Avector);
  yDebug("got A value %s", Avector.toString().c_str());
  string  Bstring = rf.check("B",
                         Value("-0.3 0.0 -0.1"),
                         "position B (string)").asString();
  extractVector(Bstring, Bvector);
  yDebug("got B value %s", Bvector.toString().c_str());
  // TO DO: Si puo ripulire, calcoliamolo in setAxes @LUCA
  Vector axisVector(2);
  axisVector[0] = eval_distance(Avector,Ovector); //AO Axis
  axisVector[1] = eval_distance(Bvector,Ovector); //BO Axis

  Vector thetaVector(2);
  string  thetaString = rf.check("theta",
                         Value("0 6.28"),
                         "theta angles (string)").asString();
  extractVector(thetaString, thetaVector);
  yDebug("got angles (rad): theta_start: %f theta_stop: %f", thetaVector[0], thetaVector[1]);
  Vector paramVector(1);
  string  paramString = rf.check("param",
                         Value("0.1"),
                         "profile parameters (string)").asString();
  extractVector(paramString, paramVector);
  yDebug("got profile parameters:(%s)", paramVector.toString().c_str());
  bool reverse = rf.check("rev");
  reverse? yDebug("reverse is ON") : yDebug("reverse is OFF");

  setCenter(Ovector);
  setStartStop(thetaVector[0], thetaVector[1]);
  setAxes(axisVector[0], axisVector[1]);
  setVelocity(paramVector[0]);

  if(pointsAreReachable()) {
      valid = true;
      setViaPoints(Avector,Bvector);
  }
  else {
      yError("Error positions are not allowed!");
  }
}

bool MJMotionProfile::operator==(const MJMotionProfile &cvmp){
  return ((valid==cvmp.valid)&&(type==cvmp.type));
}

double MJMotionProfile::computeTangVelocity(const double t){
  //v = g * K ^ (-beta);    beta = 0.33;
  //double reBeta = -1 * beta;
  //double curvature = 1 / radius;
  thetaGoal = 1.57;
  double goalRadius = computeRadius(thetaGoal);
  Vector goal(3);
  goal[0]=O[0] + goalRadius * cos(thetaGoal) * AOnorm[0] + goalRadius * sin(thetaGoal) * BOnorm[0];
  goal[1]=O[1] + goalRadius * cos(thetaGoal) * AOnorm[1] + goalRadius * sin(thetaGoal) * BOnorm[1];
  goal[2]=O[2] + goalRadius * cos(thetaGoal) * AOnorm[2] + goalRadius * sin(thetaGoal) * BOnorm[2];
  Vector distanceVect = goal - (*xd);
  //yDebug("distanceVector: %s", distanceVect.toString().c_str());
  double distance = sqrt(distanceVect[0] * distanceVect[0] + distanceVect[1] * distanceVect[1] + distanceVect[2] * distanceVect[2]);
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
        tfinal = t0 + 5.0;
    }
    else {
        theta =  thetaPrev + (t - tprev) * angVelocity;
    }

    Vector xdes = *xd;
    if(theta == theta_start) {
        yInfo("theta=theta_start check");
        preComputation(t,theta);
        //(*xd)[0]=O[0] + xAxis * cos(theta) * AO[0] + yAxis * sin(theta) * BO[0];
        //(*xd)[1]=O[1] + xAxis * cos(theta) * AO[1] + yAxis * sin(theta) * BO[1];
        //(*xd)[2]=O[2] + xAxis * cos(theta) * AO[2] + yAxis * sin(theta) * BO[2];
        (*xd)[0]=O[0] + radius * cos(theta) * AOnorm[0] + radius * sin(theta) * BOnorm[0];
        (*xd)[1]=O[1] + radius * cos(theta) * AOnorm[1] + radius * sin(theta) * BOnorm[1];
        (*xd)[2]=O[2] + radius * cos(theta) * AOnorm[2] + radius * sin(theta) * BOnorm[2];
        if( ((*xd)[0]!=A[0]) || ((*xd)[1]!=A[1]) || ((*xd)[2]!=A[2]) ){
            yInfo("Beware: difference between desired location A and computed position!");
            yInfo("vector xdes: %s", xd->toString().c_str());
            //Time::delay(5.0);
        }
    }
    else if ((theta > theta_start) && (theta<=theta_stop)) {
        yInfo("In the range xAxis:%f yAxis:%f", xAxis,yAxis);
        preComputation(t, theta);
        //(*xd)[0]=O[0] + xAxis * cos(theta) * AO[0] + yAxis * sin(theta) * BO[0];
        //(*xd)[1]=O[1] + xAxis * cos(theta) * AO[1] + yAxis * sin(theta) * BO[1];
        //(*xd)[2]=O[2] + xAxis * cos(theta) * AO[2] + yAxis * sin(theta) * BO[2];
        (*xd)[0]=O[0] + radius * cos(theta) * AOnorm[0] + radius * sin(theta) * BOnorm[0];
        (*xd)[1]=O[1] + radius * cos(theta) * AOnorm[1] + radius * sin(theta) * BOnorm[1];
        (*xd)[2]=O[2] + radius * cos(theta) * AOnorm[2] + radius * sin(theta) * BOnorm[2];

        yInfo("xdes %s", xd->toString().c_str());
    }
    else {
        return NULL;
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
  ResourceFinder rf;
  rf.setVerbose(true);

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

      yInfo("got velocity profile with %zu points", vector->size());
  }

  yDebug("parsing ");
  yDebug("%s, %s",argv[1], argv[2] );
  // configuring the resource finder
  rf.configure(b->size() * 2 + 1, argv);
  // visiting the parameters using the RF
  Vector Ovector(3);
  Vector Avector(3);
  Vector Bvector(3);

  string Ostring = rf.check("O",
                         Value("-0.3 -0.1 0.1"),
                         "position O (string)").asString();
  extractVector(Ostring, Ovector);
  yDebug("got O value %s", Ovector.toString().c_str());
  string Astring = rf.check("A",
                         Value("-0.3 0.0 0.1"),
                         "position A (string)").asString();
  extractVector(Astring, Avector);
  yDebug("got A value %s", Avector.toString().c_str());
  string Bstring = rf.check("B",
                         Value("-0.3 -0.1 0.3"),
                         "position B (string)").asString();
  extractVector(Bstring, Bvector);
  yDebug("got B value %s", Bvector.toString().c_str());

  // TO DO: Si puo ripulire, calcoliamolo in setAxes @LUCA
  Vector axisVector(2);
  axisVector[0] = eval_distance(Avector,Ovector); //AO Axis
  axisVector[1] = eval_distance(Bvector,Ovector); //BO Axis

  Vector thetaVector(2);
  string  thetaString = rf.check("theta",
                         Value("0 6.28"),
                         "theta angles (string)").asString();
  extractVector(thetaString, thetaVector);
  yDebug("got angles (rad): theta_start: %f theta_stop: %f", thetaVector[0], thetaVector[1]);
  // here we read the velocity profile passed as sequence of values
  string  velString = rf.check("vel",
                         Value("0.1 0.1 0.1"),
                         "velocity profile (string)").asString();

  //read the number of velocity points (I don't like this implementation) @Luca
  std::stringstream  stream(velString);
  unsigned int count = 0;
  string space = " ";
  while(stream >> space) { ++count;}
  ///////////////////////////////////////////////////////////////////////////

  Vector velVector(count);
  extractVector(velString, velVector);

  yDebug("got velocity profile:(%s)", velVector.toString().c_str());

  bool reverse = rf.check("rev");
  reverse? yDebug("reverse is ON") : yDebug("reverse is OFF");

  setCenter(Ovector);
  setReverse(reverse);
  setStartStop(thetaVector[0], thetaVector[1]);
  setAxes(axisVector[0], axisVector[1]);
  setVelocity(velVector); // set the velocity generic velocity profile, in  the other profiles this is a different parameter

  if(pointsAreReachable()) {
    valid = true;
    setViaPoints(Avector,Bvector);
  }
  else {yError("Error positions are not allowed!");}

}

bool GVPMotionProfile::operator==(const GVPMotionProfile &gvmp){
  return ((valid==gvmp.valid)&&(type==gvmp.type));
}

void GVPMotionProfile::preComputation(const double t, const double theta){
  //angular velocity expressed in frequency [Hz],
  //e.g.: 0.1 => 1/10 of 2PI per second; 2PI in 10sec
  //e.g.: 0.2 => 1/20 of 2PI per second; 2PI in 20sec
  //ONLY FOR CIRCLES: angular velocity constant to obtain constant tang.velocity
  //FOR ELLIPSE: compute angular velocity from A,B and desired tang.velocity
  radius = computeRadius(theta);

  yInfo("computed radius %f [m] for tangVelocity %f [m/s]", radius, tanVelocity);
  // computing angular velocity in function of the radius, theta, tangential velocity
  //angVelocity = computeAngVelocity(theta);

  // Written by @Luca
  //--------------------------------------------------------------------------
  //IMPONGO LA VELOCITA TANGENZIALE E RICAVO LA VELOCITA ANGOLARE DA IMPORRE
  // the formula is Tan_vel = r *ang_vel

  //iteratively impose the velocity
  tanVelocity = velocityProfile[step_counter];

  if (step_counter < velocityProfile.size()) {
    step_counter = step_counter++;
  }

  //yInfo("VELOCITY PROFILE %s", velocityProfile.toString().c_str()); @Luca
  yInfo("DESIRED TANGENTIAL VELOCITY  %f IN m/s", tanVelocity);

  angVelocity = tanVelocity / radius;
  //--------------------------------------------------------------------------

  yInfo("computed angular velocity %f in rad/s", angVelocity);
  double tanVelocity_temp = checkTanVelocity(theta);
  yInfo("computed tang velocity %f in m/s", tanVelocity_temp);
  //double theta = 2.0 * M_PI * angVelocity * (t - t0);
  yInfo("theta angle [rad] %f", theta);
}

Vector* GVPMotionProfile::compute(double t){
  double thetaStart;
  if(t-t0 == 0) {
      theta = theta_start;
      thetaStart =  theta_start;
  }
  else {
      theta =  thetaPrev + reverse * (t - tprev) * angVelocity;
  }

  Vector xdes = *xd; //perchè dichiarato qui ? @Luca
  if (inRange(theta) || theta == thetaStart) {
      preComputation(t, theta); //given theta, evaluate radius and ang vel

      (*xd)[0] = O[0] + radius * cos(theta) * AOnorm[0] + radius * sin(theta) * BOnorm[0];
      (*xd)[1] = O[1] + radius * cos(theta) * AOnorm[1] + radius * sin(theta) * BOnorm[1];
      (*xd)[2] = O[2] + radius * cos(theta) * AOnorm[2] + radius * sin(theta) * BOnorm[2];

      yInfo("xdes %s", xd->toString().c_str());
  }
  else {
      return NULL;
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
