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
 * @file handProfilerThread.cpp
 * @brief Implementation of the handProfiler thread (see handProfilerThread.h).
 */

#include <iCub/handProfilerThread.h>
#include <cstring>

#define MAX_TORSO_PITCH     10.0    // [deg]
#define MIN_TORSO_YAW       -40.0   // [deg]
#define MAX_TORSO_YAW       40.0    // [deg]
#define RATETHREAD          5       // [ms]
#define GAZEINTERVAL        20

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace std;
using namespace profileFactory;
using namespace fingerFactory;


//******************** factories for fingers ***************************************

FingerProfile* factoryCVFingerProfile() {
    CVFingerProfile *fingerProfile = new CVFingerProfile();
    return static_cast<FingerProfile*>(fingerProfile);
}

FingerProfile* factoryCVVFingerProfile() {
    CVVFingerProfile *fingerProfile = new CVVFingerProfile();
    return static_cast<FingerProfile*>(fingerProfile);
}

//********************* factories for motion profiles ***********************************
MotionProfile* factoryCVMotionProfile(const Bottle &param) {
    CVMotionProfile *cvmp = new CVMotionProfile(param);
    if(!cvmp->isValid()){
        yError("factory ERROR");
        delete cvmp;
        return nullptr;
    }
    else {
        return static_cast<MotionProfile*>(cvmp);
    }
}

MotionProfile* factoryMJMotionProfile(const Bottle &param) {
    MJMotionProfile *mjmp = new MJMotionProfile(param);
    if(!mjmp->isValid()){
        yError("factory ERROR");
        delete mjmp;
        return nullptr;
    }
    else {
        return static_cast<MotionProfile*>(mjmp);
    }
}

MotionProfile* factoryGVPMotionProfile(const Bottle &param) {
    GVPMotionProfile *gvmp = new GVPMotionProfile(param);
    if(!gvmp->isValid()){
        yError("factory ERROR");
        delete gvmp;
        return nullptr;
    }
    else {
        return static_cast<MotionProfile*>(gvmp);
    }
}

MotionProfile* factoryTTPLMotionProfile(const Bottle &param) {
    TTPLMotionProfile *ttplmp = new TTPLMotionProfile(param);
    if(!ttplmp->isValid()){
        yError("factory ERROR: not valid profile");
        //delete ttplmp;
        yDebug("deleting the invalid profile");
        return nullptr;
    }
    else {
        return static_cast<MotionProfile*>(ttplmp);
    }
}

MotionProfile* factoryTwoThirdMotionProfile(const Bottle &param) {
    TwoThirdMotionProfile *ttplmp = new TwoThirdMotionProfile(param);
    if(!ttplmp->isValid()){
        yError("factory ERROR: not valid profile");
        //delete ttplmp;
        yDebug("deleting the invalid profile");
        return nullptr;
    }
    else {
        return static_cast<MotionProfile*>(ttplmp);
    }
}
//*************************************************************************************************//

handProfilerThread::handProfilerThread(): RateThread(RATETHREAD) {
    robot = "icub";
    icart = nullptr;
    count = 0;
    firstIteration = true;
    idle = true;
    state = none;
    fileCounter = 0;
    timestamp = new yarp::os::Stamp(0,0);
    gazetracking = false;
    saveOn = false;
    speedFactor = 1.0;
    partnerTime = 0;
    infoSamples = 0;
    firstDuration = 0;
    repsNumber = 1;
    firstPos.resize(3);
    firstOri.resize(4);
    // we want to raise an event each time the arm is at 20%
    // of the trajectory (or 70% far from the target) // too verbose @Luca
    //cartesianEventParameters.type="motion-ongoing";
    ///cartesianEventParameters.motionOngoingCheckPoint=0.2;

}

handProfilerThread::handProfilerThread(string _robot, string _configFile, ResourceFinder rf): RateThread(RATETHREAD) {
    robot = _robot;
    configFile = _configFile;
    icart = 0;
    count = 0;
    firstIteration = true;
    idle = true;
    state = none;
    fileCounter = 1;
    timestamp = new Stamp(0,0);
    verbosity = true;
    gazetracking = false;
    saveOn = false;
    graspOn = false;
    speedFactor = 1.0;
    partnerTime = 0;
    infoSamples = 0;
    firstDuration = 0;
    repsNumber = 1;
    this->rf = rf;
    firstPos.resize(3);
    firstOri.resize(4);
    // we wanna raise an event each time the arm is at 20%
    // of the trajectory (or 70% far from the target)
    //cartesianEventParameters.type="motion-ongoing";
    //cartesianEventParameters.motionOngoingCheckPoint=0.2;

}

handProfilerThread::~handProfilerThread() {
    // do nothing
}

bool handProfilerThread::threadInit() {


    /* set the cartesian conntroller for movement */
    Property optionCartesian("(device cartesiancontrollerclient)");
    string str("/");
    str.append(robot);
    str.append("/cartesianController/"+part);
    yDebug("remote: %s", str.c_str());
    optionCartesian.put("remote","/" +  robot + "/cartesianController/"+part);
    optionCartesian.put("local","/motionProfiler/"+part);
    optionCartesian.put("writeStrict","on");

    if (!client.open(optionCartesian)) {
        yInfo("Client not available. Proceeding to pure imagination action performance ");
        icart = nullptr;
    }
    else {
        yInfo("preparing the icart");
        // open the view
        client.view(icart);

        // latch the controller context in order to preserve
        // it after closing the module
        // the context contains the dofs status, the tracking mode,
        // the resting positions, the limits and so on.
        icart->storeContext(&startup_context_id);

        // set trajectory time
        icart->setTrajTime(1.0);

        // get the torso dofs
        Vector newDof, curDof;
        icart->getDOF(curDof);
        newDof=curDof;

        // enable the torso yaw and pitch
        // disable the torso roll
        if(pitchDof == 1) {
            yInfo("pitchDof = ON");
            newDof[0]=pitchDof;
        }
        else {
            yInfo("pitchDof = OFF");
            newDof[0] = 0; //1
        }
        if(rollDof == 1) {
            yInfo("rollDof = ON");
            newDof[1]=rollDof;
        }
        else {
            yInfo("rollDof = OFF");
            newDof[1] = 0; //0;
        }
        if(yawDof == 1) {
            yInfo("yawDof = ON");
            newDof[2] = yawDof;
        }
        else {
            yInfo("yawDof = OFF");
            newDof[2] = 0;
        }

        // impose some restriction on the torso pitch and yaw
        limitTorsoPitch();
        limitTorsoYaw();

        // send the request for dofs reconfiguration
        icart->setDOF(newDof,curDof);

        // print out some info about the controller
        Bottle info;
        icart->getInfo(info);
        fprintf(stdout,"info = %s\n",info.toString().c_str());

        // register the event, attaching the callback
        icart->registerEvent(*this);
    }

    /* set the position controller for movement from file */
    Property optionJoints;
    optionJoints.put("device", "remote_controlboard");
    optionJoints.put("local", "/motionProfiler/joints");                 //local port names
    optionJoints.put("remote", "/"+ robot + "/" +part);                        //where we connect to
    optionJoints.put("writeStrict","on");

    if (!robotDevice.open(optionJoints)) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
    }

    bool ok;
    ok = robotDevice.view(encs);
    ok = ok && robotDevice.view(ictrl);
    ok = ok && robotDevice.view(idir);


    if (!ok) {
        yError("Problems acquiring interfaces\n");
        return 0;
    }

    idir->getAxes(&njoints);
    //yDebug("njoints = %d", njoints);

    //initializing gazecontrollerclient
    if(gazetracking){
        printf("initialising gazeControllerClient \n");
        Property optionGaze;
        optionGaze.put("device","gazecontrollerclient");
        optionGaze.put("remote","/iKinGazeCtrl");
        string localCon("/motionProfiler/gaze");
        localCon.append(getName(""));
        optionGaze.put("local",localCon.c_str());
        yInfo("activating the PolyDriver");

        clientGazeCtrl = new PolyDriver();
        clientGazeCtrl->open(optionGaze);
        igaze = nullptr;

        if (clientGazeCtrl->isValid()) {
           clientGazeCtrl->view(igaze);
           igaze->storeContext(&originalContext);
           igaze->setEyesTrajTime(0.8);
           igaze->setNeckTrajTime(0.9);

           blockNeckPitchValue = -1;
           if(blockNeckPitchValue != -1) {
               igaze->blockNeckPitch(blockNeckPitchValue);
               yInfo("pitch fixed at %d \n",blockNeckPitchValue);
           }
           else {
               yInfo("pitch free to change\n");
           }
        }
        else {
            yError("Not Valid clientGazeCtrl");
            igaze = 0;
        }
        yInfo("Success in initialising the gaze");
    }

    string rootNameGui("");
    rootNameGui.append(getName("/gui:o"));
    if(!guiPort.open(rootNameGui.c_str())) {
          yError("guiPort is not open with success. Check for conflicts");
    }
    //string xdNameGui("");
    //xdNameGui.append(getName("/xd:o"));
    if(!xdPort.open("/motionProfiler/xd:o")) {
          yError("xdPort is not open with success. Check for conflicts");
    }
    string velName("");
    velName.append(getName("/vel:o"));
    if(!velPort.open(velName.c_str())) {
          yError("velPort is not open with success. Check for conflicts");
    }

    /* initialization of the thread */
    x.resize(3);
    o.resize(4);
    xd.resize(3);
    xdHome.resize(3);
    xdGazeHome.resize(3);
    od.resize(4);
    odHome.resize(4);
    xdhat.resize(3);
    odhat.resize(4);
    qdhat.resize(4);

    t0 = Time::now();
    //initialization of the relevant vectors
    xdHome[0] = -0.298; xdHome[1] = -0.210; xdHome[2] = 0.029;
    xdGazeHome[0] = -0.5; xdGazeHome[1] = -0.0; xdGazeHome[2] = 0.35;
    //od[0] = -0.096; od[1] = 0.513; od[2] = -0.8528; od[3] = 2.514;
    od[0] = -0.076; od[1] = -0.974; od[2] = 0.213; od[3] = 3.03;
    //0.024206	 0.867429	-0.496971	 2.449946
    //-0.041466	 0.602459	-0.797072	 2.850305
    odHome[0] = -0.041; odHome[1] = 0.602; odHome[2] = -0.797; odHome[3] = 2.85;

    //setting grasping reset home for movement that are independent from the fingerProfile
    graspHome.resize(9);
    graspFinal.resize(9);
    graspCurrent.resize(9);
    graspHome[0] = 40.0; graspHome[1] = 40.0; graspHome[2] = 0.0; graspHome[3] = 0.0; graspHome[4] = 0.0; graspHome[5] = 0.0; graspHome[6] = 0.0; graspHome[7] = 0.0; graspHome[8] = 0.0;
    graspFinal[0] = 40.0; graspFinal[1] = 50.0; graspFinal[2] = 20.0; graspFinal[3] = 50.0; graspFinal[4] = 50.0; graspFinal[5] = 50.0; graspFinal[6] = 50.0; graspFinal[7] = 50.0; graspFinal[8] = 125.0;

    graspNumber = 9;
    fingerJoints.resize(njoints);
    fd.resize(9);
    if(graspOn){
        yInfo("grasping already activated");
        fp = factoryCVVFingerProfile();
    }
    else{
        yInfo("grasping not active during initialization");
        fp = nullptr;
    }
    yInfo("handProfiler thread correctly started");

    return true;
}

void handProfilerThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

void handProfilerThread::setGrasp(bool grasp) {
    this->graspOn=grasp;
    if(grasp) {
        yInfo("Grasping ON");
        //checking and instantiating the fp oject correctly
        if (fp==nullptr) {
            //fp was not initialized in the threadInit
            fp = factoryCVVFingerProfile();
        }

        for (int i = 7; i < 16; i++) {
            ictrl->setControlMode(i, VOCAB_CM_POSITION_DIRECT);
        }
    }else{
        yInfo("Grasping OFF");
        //setting the pointer to the finger class fp to null
        fp = nullptr;
    }
    encs->getEncoders(fingerJoints.data());
    Time::delay(1.0);
}

void handProfilerThread::setPart(string _part) {
    if(_part == "left_arm" || _part == "right_arm"){
        this->part=_part;
        yInfo("Selected part: %s", part.c_str());
    }else{
        yError("Failed to set arm, please restart the module specifying the arm to use");
    }
}

void handProfilerThread::getPart() {
    yInfo("Selected part is: %s", part.c_str());
}

void handProfilerThread::loadFile(string str) {
    this->fileName=str;

    filePath = rf.findFile(fileName + ".info");           //look for info file
    infoInputFile.open(filePath.c_str());                 //open info file
    yInfo("opening file.....");
    if(infoInputFile.is_open()){
        infoInputFile >> movementDuration;                //save info from file
        infoInputFile >> sampleNumber;
        startPos.resize(3);
        startOri.resize(4);
        for(int i=0; i<3; i++){
          infoInputFile >> startPos(i);
          //yDebug("startpos %d, : %f", i, startPos(i));
        }
        for(int i=0; i<4; i++){
          infoInputFile >> startOri(i);
          //yDebug("startori %d, : %f", i, startOri(i));
        }
        yInfo("duration: %f  number: %d", movementDuration, sampleNumber );
        infoInputFile.close();
    }else{
        yError(".info File not found");
    }
}

void handProfilerThread::setPartnerStart() {
    this->partnerStart = Time::now();
    yDebug("start time: %f", partnerStart);
}

void handProfilerThread::setRepsNumber(int n) {
     this->repsNumber = n;
     yInfo("number of repetitions set to: %d", repsNumber);
}

void handProfilerThread::setPartnerStop() {
    this->partnerStop = Time::now();
    yDebug("stop time: %f", partnerStop);
}

void handProfilerThread::setPartnerTime(double _time) {
    if(_time == 0.0){
        this->partnerTime = partnerStop - partnerStart;
    }else{
        this->partnerTime = _time;
    }
    yDebug("sync time: %f", partnerTime);
}

std::string handProfilerThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void handProfilerThread::rotAxisX(const double& angle) {
    Vector Anew, Bnew, Cnew, Dnew;
    Anew = mp->getA();
    Bnew = mp->getB();
    Cnew = mp->getC();
    Dnew = mp->getD();
    // multiplying the vector by the matrix
    mp->setA(Anew);
    mp->setB(Bnew);
    mp->setC(Cnew);
    mp->setD(Dnew);
    mp->setViaPoints(Anew, Bnew);
}

// Non mi sembra implementato, da rimuovere ? @LUCA
void handProfilerThread::rotAxisY(const double& angle) {
    Vector Anew, Bnew, Cnew, Dnew;
    Anew = mp->getA();
    Bnew = mp->getB();
    Cnew = mp->getC();
    Dnew = mp->getD();
    mp->setA(Anew);
    mp->setB(Bnew);
    mp->setC(Cnew);
    mp->setD(Dnew);
}

void handProfilerThread::rotAxisZ(const double& angle) {
    Vector Anew, Bnew, Cnew, Dnew;
    Anew = mp->getA();
    Bnew = mp->getB();
    Cnew = mp->getC();
    Dnew = mp->getD();
    mp->setA(Anew);
    mp->setB(Bnew);
    mp->setC(Cnew);
    mp->setD(Dnew);
}

void handProfilerThread::setInputPortName(string InpPort) {

}

bool handProfilerThread::resetExecution() {
    bool result = true;

    if(0 != icart){
        Vector xInit(3);
        xInit = mp->getInitial();

        //go to the desired pose
        icart->goToPose(xInit,od);
        Time::delay(3.0);

        //read the reached position (real)
        icart->getPose(x,o);

        double e_x=norm(xdhat-xInit);
        double e_o=norm(odhat-od);

        cout << "\n++++++++++++++++++++  going to initial position  ++++++++++++++++++++++++\n";
        yInfo("desired hand pose         :      %s [m]\n", xInit.toString().c_str());
        yInfo("actual position           :      %s [m]\n", x.toString().c_str());
        yInfo("desired orientation       :      %s [rad]\n",od.toString().c_str());
        yInfo("actual orientation        :      %s [rad]\n",o.toString().c_str());
        yInfo("norm(e_x)                 :      %g [m]\n",e_x);
        yInfo("norm(e_o)                 :      %g [rad]\n",e_o);
        cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n\n";

        if(gazetracking) {
            yDebug("resetExecution::lookAtFixationPoint");
            igaze->lookAtFixationPoint({-0.3, -0.1, 0.1});
            yDebug("resetExecution::lookAtFixationPoint:success");
        }

        if (e_x > 0.5) {
            yError("Error in resetting the initial position");
            result = false;
        }

    }
    idle = true;
    return result;
}

bool handProfilerThread::startExecution() {
    //valuto che mp sia inizializzato
    if(!mp){
        yError("mp is not initialized");
        return false;
    }
    idle = false;
    state = execution;
    firstIteration = true;
    t0 = Time::now();
	return true;
}

bool handProfilerThread::startSimulation(const bool _reverse) {
    //count = 0;
    //mp->setReverse(_reverse);
    idle = false;
    state = simulation;
    firstIteration = true;
    t0 = Time::now();
	return true;
}

bool handProfilerThread::startResetting() {
    //count = 0;
    //mp->setReverse(_reverse);
    idle = true;
    state = none;
    bool ret = true;
    icart-> goToPose(xdHome,odHome);
    if(gazetracking) {
        igaze->lookAtFixationPoint(xdGazeHome);
    }
	return ret;
}

bool handProfilerThread::saveJoints() {                                  //save to file
    //count = 0;
    //mp->setReverse(_reverse);
    saveOn = !saveOn;
    t0 = Time::now();
    if(saveOn) yInfo("save to file ENABLED");
    else yInfo("save to file DISABLED");
	return true;
}

bool handProfilerThread::startJoints(double factor = 1.0) {              //move from file
    //count = 0;
    //mp->setReverse(_reverse);
    idle = false;
    state = file;
    firstIteration = false;
    t0 = Time::now();
    speedFactor = factor;
	return true;
}

bool handProfilerThread::fingerfactory(const string type, const Bottle finalB) {
    yDebug("finalB: %s", finalB.toString().c_str());
    if (!strcmp(type.c_str(),"CVV")) {
        fp = factoryCVVFingerProfile();
        yDebug("returned from CVV factory");
        if (fp == NULL){
            yError("finger factory returned error");
            return false;
        }
    }
    else{
        yError("Error.Type is unknown.");
        return false;
    }

    return true;

}

bool handProfilerThread::factory(const string type, const Bottle finalB) {

    if (!strcmp(type.c_str(),"CVP")) {
        yInfo("Velocity profile: CONSTANT VELOCITY PROFILE");
        mp = factoryCVMotionProfile(finalB);
        if (mp == NULL){
            yError("factory returned error");
            return false;
        }
    }
    else if(!strcmp(type.c_str(),"TTPL")) {
        yInfo("Velocity profile: FAKE(?) TWO THIRD POWER LAW");
        mp = factoryTTPLMotionProfile(finalB);
        if (mp == NULL){
            yError("factory returned error");
            return false;
        }
    }
    else if(!strcmp(type.c_str(),"TwoThird")){
        yInfo("Velocity profile: TWO THIRD POWER LAW");
        mp = factoryTwoThirdMotionProfile(finalB);
        if (mp == NULL){
            yError("factory returned error");
            return false;
        }
    }
    else if(!strcmp(type.c_str(),"MJP")){
        yInfo("Velocity profile: MINIMUM JERK");
        mp = factoryMJMotionProfile(finalB);
        if (mp == NULL){
            yError("factory returned error");
            return false;
        }
    }
    else if(!strcmp(type.c_str(),"GVP")){
        yInfo("Velocity profile: GENERIC VELOCITY PROFILE");
        mp = factoryGVPMotionProfile(finalB);
        if (mp == NULL){
            yError("factory returned error");
            return false;
        }
    }
    else{
        yError("Error. Factory type is unknown.");
        return false;
    }

    bool result = true;
    result = result & resetExecution();
    return result;
}

Matrix velocityReport(1000,3);
Vector x_dot,o_dot;

void handProfilerThread::run() {

    if(!idle) {
        count++;
        if (firstIteration) {
            yInfo("first iteration");                                                           // first itaration set variables
            t = t0;
            mp->setT0(t0);
            firstIteration = false;
            // show O,A,B,C,D in the iCubGUI
            displayProfile();

            //if(Constant velocity profile or generic velocity profile)
            //start offline trajectory precomputation
//            if(mp->type == "GVP" && state == execution) { //to add mp->type == "CVP" ||
////                yInfo("start offline trajectory precomputation");
////                Matrix joints_trajectory = mp->offline_compute(icart,encs);
////                yInfo("Performing the trajectory in POSITION DIRECT MODE");
////                playTrajectory(joints_trajectory);
//
//                state = none;
//                idle = true;
//
//            }

            //-----file opening to save -------
            if(saveOn){                                                                         // if save is enabled, open files for output and
                ostringstream convert;                                                          // set variables for duration calculation and initial position
                convert << fileCounter;
                string fileToSave = "action_" + convert.str() + ".log";
                outputFile.open(fileToSave.c_str(), std::ofstream::out);
                fileToSave = "action_" + convert.str() + ".info";
                infoOutputFile.open(fileToSave.c_str(), std::ofstream::out);
                firstDuration = Time::now();
                icart->getPose(firstPos, firstOri);
                // icart->storeContext(&icartContext);
            }
            //-------------------------------
        }
        else {
            t=Time::now();
        }

        bool success;

        switch (state) {
            case execution:
                cout << count << endl;
                success = generateTarget();
                if(success){
                    //go to next position, here we don't care about the orientation
                    icart->getTaskVelocities(x_dot,o_dot);
                    velocityReport.setRow(count,x_dot);
                    icart->setPosePriority("position");
                    cout << "going to: " << xd.toString() << endl;
                    icart->goToPose(xd,od);
                    //time::delay(0.01);
                    //icart->waitMotionDone();

                    if(graspOn){
                        const int graspJoints[] = {7,8,9,10,11,12,13,14,15};
                        idir->setPositions(graspNumber, graspJoints, fd.data());
                    }
                    if(saveOn) {
                        saveToArray();
                    }
                    if(gazetracking && (count%GAZEINTERVAL==0)) {
                        // set starting gaze pose
                        igaze->lookAtFixationPoint(xd);}
                }
                // if movement is finished and we need to save the files
                else if(!success && outputFile.is_open() && saveOn){
                    yInfo("file saved");
                    yDebug("cicli: %d ", infoSamples);
                    bool motionDone;
                    icart->checkMotionDone(&motionDone);
                    while(!motionDone){
                        if(saveOn)
                            saveToArray();
                        icart->checkMotionDone(&motionDone);
                        Time::delay(0.005);
                    }

                    saveToFile();
                    saveInfo();                                           //save info to file
                    fileCounter++;                                        //movement finished, if save enabled close the files and reset the state
                    outputFile.close();
                    infoOutputFile.close();
                    firstDuration = 0;
                    infoSamples = 0;
                    state = none;
                    idle = true;
                    // icart->restoreContext(icartContext);
                }else{
                    // if the movement is finished
                    ////////////////////////////////////////////////////////////////
                    //write velocity report to file
                    cout << "writing report after " << count << "steps" << endl;
                    velocityReport.resize(count,3);
                    std::ofstream myfile;
                    myfile.open("velocityReport.csv", std::ofstream::out);
                    myfile << velocityReport.toString();
                    myfile.close();
                    //////////////////////////////////////////////////////////////////////
                    state = none;
                    idle = true;
                    // icart->restoreContext(icartContext);
                }
                break;
            case simulation:
                success = generateTarget();
                if(!success){
                    state = none;
                    idle = true;
                }
                break;
            case file:
                yWarning("starting movement from file");
                startFromFile();
                state = none;
                idle = true;
                break;
            default:
                yWarning("thread is not idle but the state is %c",state);
                break;
        }

        // show points on the trajectory, yellow = simulated , green = attained
        displayTarget();
        if(xdPort.getOutputCount()) {
            printXd();
        }
        if(velPort.getOutputCount()) {
            printVel();
        }

        // some verbosity
        if(verbosity) {
            //printStatus();
        }
    }
    else if(idle && count!=0) {
        count=0;
    }
}

void handProfilerThread::graspReset(){
    encs->getEncoders(fingerJoints.data());
    const int graspJoints[] = {7,8,9,10,11,12,13,14,15};
    for(int i = 0; i<9; i++){
        graspCurrent[i] = fingerJoints[i+7];
    }
    yDebug("%f %f %f %f %f %f %f %f %f",graspCurrent[0],graspCurrent[1],graspCurrent[2],graspCurrent[3],graspCurrent[4],graspCurrent[5],graspCurrent[6],graspCurrent[7],graspCurrent[8]);
    for(int i = 0; i<9; i++){
        while(graspCurrent[i]>graspHome[i]+1 || graspCurrent[i]<graspHome[i]-1){
            graspCurrent[i] = graspCurrent[i]+((graspHome[i]-graspCurrent[i])/3);
            idir->setPosition(i+7, graspCurrent[i]);
            //yDebug("while2 %d", i);
            Time::delay(0.01);
        }
    }

    yInfo("Grasp resetted");
    //yDebug("%f %f %f %f %f %f %f %f %f",graspCurrent[0],graspCurrent[1],graspCurrent[2],graspCurrent[3],graspCurrent[4],graspCurrent[5],graspCurrent[6],graspCurrent[7],graspCurrent[8]);
    encs->getEncoders(fingerJoints.data());
    //yDebug("%f %f %f %f %f %f %f %f %f",fingerJoints[7],fingerJoints[8],fingerJoints[9],fingerJoints[10],fingerJoints[11],fingerJoints[12],fingerJoints[13],fingerJoints[14],fingerJoints[15]);
}

void handProfilerThread::printErr() {
    Stamp ts;
    ts.update();
    Bottle& b = errPort.prepare();
    b.clear();
    //b.addFloat64(mp->getError());
    b.addFloat64(-1.0);
    errPort.setEnvelope(ts);
    errPort.write();
}

void handProfilerThread::printVel() {
    Stamp ts;
    ts.update();
    Bottle& b = velPort.prepare();
    b.clear();
    b.addFloat64(mp->getTanVelocity());
    b.addFloat64(mp->getCurvature());
    b.addFloat64(mp->getRadius());
    b.addFloat64(mp->getAngVelocity());
    velPort.setEnvelope(ts);
    velPort.write();
}

void handProfilerThread::printXd() {
    Stamp ts;
    ts.update();
    Bottle& b = xdPort.prepare();
    b.clear();
    b.addFloat64(xd[0]);
    b.addFloat64(xd[1]);
    b.addFloat64(xd[2]);
    xdPort.setEnvelope(ts);
    xdPort.write();
}

bool handProfilerThread::setCurrentOrientation() {
    Vector x0,o0;
    icart->getPose(x0,o0);
    od = o0;
    return true;
}

bool handProfilerThread::setOrientation(const Vector vectorOrientation) {
    if (vectorOrientation.size() == 4) {
        od = vectorOrientation;
        return true;
    }
    return false;
}

bool handProfilerThread::generateTarget() {
    // translational target part: a circular trajectory
    // in the yz plane centered in [-0.3,-0.1,0.1] with radius=0.1 m
    // and frequency 0.1 Hz (1/10 of 2PI per second)
    //xd[0]=-0.3;
    ///xd[1]=-0.1+0.1*cos(2.0*M_PI*0.1*(t-t0));
    //xd[2]=+0.1+0.1*sin(2.0*M_PI*0.1*(t-t0));

    Vector* _xdpointer = mp->compute(t);

    //Vector _xd;
    if(_xdpointer == NULL) {
        yInfo("STOP");
        //_xd.resize(3);
        //_xd[0] = 0.0; _xd[1] = 0.0; _xd[2] = 0.0;
        return false;
    }


    xd = *_xdpointer;
    //printf("Error %f %f %f \n", xd[0] -_xd[0], xd[1] - _xd[1], xd[2] - _xd[2]);

    // we keep the orientation of the left arm constant:
    // we want the middle finger to point forward (end-effector x-axis)
    // with the palm turned down (end-effector y-axis points leftward);
    // to achieve that it is enough to rotate the root frame of pi around z-axis
    //od[0] = 0.0; od[1] = 0.0; od[2] = 1.0; od[3] = M_PI;
    //od[0] = 0.29; od[1] = 0.40; od[2] = -0.86; od[3] = 3.09;
    //od[0] = -0.43; od[1] = -0.02; od[2] = -0.90; od[3] = 2.98;
    //od[0] = -0.06; od[1] = -0.87; od[2] = 0.49; od[3] = 2.97;


    //yWarning("%f %f %f %f %f %f %f %f %f",fingerJoints(7),fingerJoints(8),fingerJoints(9),fingerJoints(10),fingerJoints(11),fingerJoints(12),fingerJoints(13),fingerJoints(14),fingerJoints(15));
    if(fingerJoints[15]>graspFinal[8]){                   //end of grasping
        graspOn = 0;
        //yWarning("Grasp OFF, grasp finished");
    }
    if(graspOn){
        Vector* _fdpointer = fp->compute(fingerJoints, t-t0);
        if(_fdpointer == NULL) {
            yError("Finger motion not valid, GRASP OFF");
            graspOn = false;
        }
        else {
            fd = *_fdpointer;  // TODO: check if this copy is necessary
            for (int i = 0; i < 9; i++) {
                fingerJoints[i+7] = fd[i];
            }
        }

        //yDebug("%f %f %f %f %f %f %f %f %f",fd(0),fd(1),fd(2),fd(3),fd(4),fd(5),fd(6),fd(7),fd(8));
    }

    return true;
}

void handProfilerThread::saveToArray() {                         //save to file: read encoders and save values in file with timestamp
    //yDebug("saveToArray");
    Vector tempJoints;
    tempJoints.resize(njoints);
    bool retFromEncoders = encs->getEncoders(tempJoints.data());
    if(!retFromEncoders)
        yError("encoders misreading");

    timestamp->update();
    jointsToSave.push_back(timestamp->getTime());
    jointsToSave.setSubvector(1,tempJoints);
    infoSamples++;
}

void handProfilerThread::saveToFile() {                         //save to file: read encoders and save values in file with timestamp
    //yDebug("saveToFile");
    outputFile.precision(13);
    if (outputFile.is_open()){
        for(int i=0; i<jointsToSave.length(); i++){
            if (i%17 == 0 && i != 0)
                outputFile << "\n";
            outputFile << jointsToSave[i] << " ";
            //yDebug("position %d : %f", i, jointsToSave[i]);
        }
    }
    else
        yError("Unable to open output file");

    jointsToSave.resize(0);
}

void handProfilerThread::saveInfo() {                                            //save the info in separate file
    if (infoOutputFile.is_open()){
        infoOutputFile << Time::now() - firstDuration << " ";                   //duration
        infoOutputFile << infoSamples << " ";                                   //samples number
        for(int i=0; i<3; i++){
          infoOutputFile << firstPos(i) << " ";                                 //initial cartesian position
        }
        for(int i=0; i<4; i++){
          infoOutputFile << firstOri(i) << " ";                                 //initial cartesian orientation
        }
    }
    else
        yError("Unable to open info output file");
}

void handProfilerThread::startFromFile() {                                       //move from file
    if (partnerTime != 0.0) {
        speedFactor = movementDuration / partnerTime;                           //if partnerTime is set change speedFactor
        yWarning("start from partner time");
    }

    filePath = rf.findFile(fileName + ".log");                                  //read joints file
    for(int i=0; i<repsNumber; i++){
        inputFile.open(filePath.c_str());
        yInfo("opening file.....");
        if(inputFile.is_open()){
            if(speedFactor >= 0.1 && speedFactor <= 10.0){
                yWarning("speedFactor %f ", speedFactor);
                playFromFile();
                yInfo("finished movement");
            }else{
                yError("Speed Factor value is either too low or too high, please insert value between 0.1 and 6.0");
            }
        }
        else {
            yError(".log File not found");
        }
        inputFile.close();
    }
    repsNumber = 1;
    yInfo("reps number reset to 1");
    partnerTime = 0.0;
}

void handProfilerThread::playFromFile() {
    double jointValue = 0.0;
    double playJoints[17];
    double executionTime = 0;
    double previousTime = 0;

    int playCount = 0;

    bool done = false;
    bool first = true;

    Vector encoders;
    Vector command;
    encoders.resize(njoints);
    command.resize(njoints);

    encs->getEncoders(encoders.data());                                         // encoder reading from current position
    command = encoders;

    while(inputFile >> jointValue){
        done = false;
        playJoints[playCount%17] = jointValue;
        playCount++;
        if(playCount%17 == 0){
            if(first){
                first = false;
                if(abs(command[0] - playJoints[1]) > 10 || abs(command[1] - playJoints[2]) > 10 || abs(command[2] - playJoints[3]) > 10 || abs(command[3] - playJoints[4]) > 10 || abs(command[4] - playJoints[5]) > 10 || abs(command[5] - playJoints[6]) > 10 || abs(command[6] - playJoints[7]) > 10 ){
                    if(startPos(0) != 0.0 && startPos(1) != 0.0 && startPos(2) != 0.0){         //go to initial position with cartesian controller
                        yInfo("icart moving to initial position");
                        icart->goToPose(startPos, startOri);
                        icart->waitMotionDone();
                       // Time::delay(2.0);

                    }
                }

                encs->getEncoders(encoders.data());                                         // encoder reading from current position
                command = encoders;
                for (int i = 0; i < 16; i++) {
                    ictrl->setControlMode(i, VOCAB_CM_POSITION_DIRECT);
                }
                for(int i=1; i<17; i++){
                    if(abs(command[i-1] - playJoints[i]) > 2){
                        yInfo("idir moving joint %d to initial position", i);
                        yInfo("from    %f    to   %f", command[i-1], playJoints[i]);
                        double offsetInitial = 0;                                   // put robot in initial position  (adjust position reached with cartesian controller
                        if(command[i-1] < playJoints[i]){                             // to be more precise according to the first position of .log file)
                            offsetInitial = 0.2;
                        }else if(command[i-1] > playJoints[i]){
                            offsetInitial = -0.2;
                        }
                        while(command[i-1] < playJoints[i]-0.2 || command[i-1] > playJoints[i]+0.2){
                            command[i-1] = command[i-1] + offsetInitial;
                            idir->setPositions(command.data());
                            Time::delay(0.01);
                        }
                    }
                }

                yInfo("initial position reached");

                Time::delay(2.0); //julia

                previousTime = playJoints[0];
            }else{
                for(int i=1; i<17; i++){
                    command[i-1] = playJoints[i]; //fill the next-joint-pose vector
                }
                executionTime = playJoints[0] - previousTime;
                previousTime = playJoints[0];
                if(executionTime <= 0.0) {
                    executionTime = 0.05;
                }
                idir->setPositions(command.data());                             //move robot through trajectory
                //yDebug("execution time: %f", executionTime / speedFactor);
                Time::delay(executionTime / speedFactor);                       // time to wait before reaching next point as read from file and modified with speedFactor
            }
        }
    }
}

void handProfilerThread::playTrajectory(Matrix &trajectory) {

    // read the joints to control from the file
    size_t step_count = trajectory.rows();
    //read the actuated joints
    idir->getAxes(&njoints);

    if (njoints != trajectory.cols()) {
        yError("Number of joints in trajectory (%li) does not match the number of joints in the controller (%i)", trajectory.cols(), njoints);
        return;
    }


    //arm has 7 DOF + 3 from the body
    cout << "Playing trajectory with " << step_count << " steps" << endl;

    Vector encoders;
    Vector command;
    encoders.resize(njoints);
    command.resize(njoints);

    //set all joints in position direct mode
    for (int i = 0; i < 17; i++) {
        ictrl->setControlMode(i, VOCAB_CM_POSITION_DIRECT);
    }
    Time::delay(1.0);

    encs->getEncoders(encoders.data()); // encoder reading from current position
    Vector prev_command = encoders; //trajectory.getRow(0);

    cout << "Starting from: " << endl << prev_command.toString().c_str() <<  endl;

    yarp::dev::IPositionControl *ipos;
    robotDevice.view (ipos);
    ipos->positionMove(trajectory.getRow(0).data());
    //idir->setPositions(prev_command.data());
    Time::delay(10.0);

    //printing the trajectory
    cout << "Original Trajectory: " << endl << trajectory.toString().c_str() <<  endl;

    // checking that target joints positions are safe, if not replace the wrong values
    // explaination: position direct mode can be used only for small angular movements
//    cout << "Checking that target joints positions are safe" << endl;
//    for (int i = 0; i < step_count; i++) {
//        command = trajectory.getRow(i);
//        for (int j = 0; j < njoints; j++) {
//            //fix the single encoder value for each row by setting the previous value
//            if (abs(command[j]-prev_command[j]) > 10.0) {command[j] = prev_command[j];}
//        }
//        //update
//        trajectory.setRow(i, command);
//        prev_command = command;
//    }

    //printing the trajectory
    //cout << "Filtered Trajectory:" <<  endl<< trajectory.toString().c_str() <<  endl;

    // finally move the robot
    for (int i=0; i<step_count; i++){
        //start timer
        auto time = Time::now();
        //command = trajectory.getRow(i).data();
        idir->setPositions(trajectory.getRow(i).data());
        //icart->getTaskVelocities(xdot,odot); read velocity
        Time::delay(0.05);
        //check time elapesed
        cout << "Time elapsed: " << Time::now() - time << endl;
    }

    yInfo("Movement done");
}

void handProfilerThread::limitTorsoYaw() {
    int axis=1; // yaw joint
    double min, max;

    // sometimes it may be helpful to reduce
    // the range of variability of the joints;
    // for example here we don't want the torso
    // to rotate out more than neededSSS

    // we keep the lower limit
    icart->getLimits(axis,&min,&max);
    yInfo("Torso Yaw limits [%f:%f]->[%f:%f]", min, max, MIN_TORSO_YAW, MAX_TORSO_YAW);
    icart->setLimits(axis,MIN_TORSO_YAW,MAX_TORSO_YAW);
}

void handProfilerThread::limitTorsoPitch() {
    int axis=0; // pitch joint
    double min, max;

    // sometimes it may be helpful to reduce
    // the range of variability of the joints;
    // for example here we don't want the torso
    // to lean out more than 30 degrees forward

    // we keep the lower limit
    icart->getLimits(axis,&min,&max);
    icart->setLimits(axis,min,MAX_TORSO_PITCH);
}

void handProfilerThread::processing() {
}

void handProfilerThread::threadRelease() {
    // we require an immediate stop
    // before closing the client for safety reason
    if(icart) {
        yInfo("Stopping the icart");
        icart->stopControl();
        // it's a good rule to restore the controller
        // context as it was before opening the module
        icart->restoreContext(startup_context_id);
        client.close();
    }
    if(igaze){
        igaze->restoreContext(originalContext);
        delete clientGazeCtrl;
    }
    guiPort.interrupt();
    guiPort.close();
    xdPort.interrupt();
    xdPort.close();
    velPort.interrupt();
    velPort.close();

    yInfo("success in thread release");
}

void handProfilerThread::afterStart(bool s) {
    if (s)
        yInfo("Thread started successfully");
    else
        yError("Thread did not start");

    t=t0=t1=yarp::os::Time::now();
}

void handProfilerThread::displayProfile() {
    int r, g, b;
    r =255; g = 0; b = 0;

    if (guiPort.getOutputCount()) {
        // extract important parameters
        Vector vectorList[5];
        vectorList[0] = mp->getO();
        vectorList[1] = mp->getA();
        vectorList[2] = mp->getB();
        vectorList[3] = mp->getC();
        vectorList[4] = mp->getD();

        string objectName[5];
        objectName[0].append("O");
        objectName[1].append("A");
        objectName[2].append("B");
        objectName[3].append("C");
        objectName[4].append("D");

        for(int i = 0; i < 5; i++) {
            // preparing the bottle
            Bottle& obj = guiPort.prepare();
            obj.clear();
            obj.addString("object"); // command to add/update an object
            obj.addString(objectName[i]);
            // object dimensions in millimiters
            // (it will be displayed as an ellipsoid with the tag "my_object_name")
            obj.addFloat64(5);
            obj.addFloat64(5);
            obj.addFloat64(5);
            // object position in millimiters
            // reference frame: X=fwd, Y=left, Z=up
            obj.addFloat64((vectorList[i])[0] * 1000);
            obj.addFloat64((vectorList[i])[1] * 1000);
            obj.addFloat64((vectorList[i])[2] * 1000);
            // object orientation (roll, pitch, yaw) in degrees
            obj.addFloat64(0.0);
            obj.addFloat64(0.0);
            obj.addFloat64(0.0);
            // object color (0-255)
            obj.addInt32(r);
            obj.addInt32(g);
            obj.addInt32(b);
            // transparency (0.0=invisible 1.0=solid)
            obj.addFloat64(1.0);

            guiPort.writeStrict();
        }
    }
}

void handProfilerThread::displayTarget(){
    int r, g, b;
    if(state == simulation) {
        r =255; g =255; b = 0;
    }
    else {
        r = 0; g =255; b =0;
    }

    if ((guiPort.getOutputCount()) && (count%2==0)) {
        Bottle& obj = guiPort.prepare();
        obj.clear();
        obj.addString("object"); // command to add/update an object
        //EXPLAINATION by @Luca
        //if you create object with the same name it will overwrite the existing object
        //the module creates many different objects but it's a mess !
        string str("");
        sprintf((char*)str.c_str(),"%d",count);
        obj.addString(str.c_str());
        // object dimensions in millimiters
        // (it will be displayed as an ellipsoid with the tag "my_object_name")
        obj.addFloat64(5);
        obj.addFloat64(5);
        obj.addFloat64(5);
        // object position in millimiters
        // reference frame: X=fwd, Y=left, Z=up
        obj.addFloat64(xd[0] * 1000);
        obj.addFloat64(xd[1] * 1000);
        obj.addFloat64(xd[2] * 1000);
        // object orientation (roll, pitch, yaw) in degrees
        obj.addFloat64(0.0);
        obj.addFloat64(0.0);
        obj.addFloat64(0.0);
        // object color (0-255)
        obj.addInt32(r);
        obj.addInt32(g);
        obj.addInt32(b);
        // transparency (0.0=invisible 1.0=solid)
        obj.addFloat64(1.0);

        guiPort.writeStrict();
    }
}

void handProfilerThread::printStatus() {
    Vector x,o,xdhat,odhat,qdhat;

    // we get the current arm pose in the
    // operational space
    icart->getPose(x,o);

    // we get the final destination of the arm
    // as found by the solver: it differs a bit
    // from the desired pose according to the tolerances
    icart->getDesired(xdhat,odhat,qdhat);

    double e_xdhat=norm(xdhat-x);
    double e_o=norm(odhat-o);
    double e_xd=norm(xd-x);

    fprintf(stdout,"+++++++++\n");
    fprintf(stdout,"xd          [m] = %s\n",xd.toString().c_str());
    fprintf(stdout,"xdhat       [m] = %s\n",xdhat.toString().c_str());
    fprintf(stdout,"x           [m] = %s\n",x.toString().c_str());
    fprintf(stdout,"od        [rad] = %s\n",od.toString().c_str());
    fprintf(stdout,"odhat     [rad] = %s\n",odhat.toString().c_str());
    fprintf(stdout,"o         [rad] = %s\n",o.toString().c_str());
    fprintf(stdout,"norm(e_xd)   [m] = %g\n",e_xd);
    fprintf(stdout,"norm(e_xdhat)[m] = %g\n",e_xdhat);
    fprintf(stdout,"norm(e_o) [rad] = %g\n",e_o);
    fprintf(stdout,"---------\n\n");

    t1=t;
}

void handProfilerThread::clearGui() {
  Bottle& obj = guiPort.prepare();
  obj.clear();

  obj.addString("reset");

  guiPort.writeStrict();
}
