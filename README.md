# MotionProfiler 2.0

This module allows to control the velocity of icub's end-effector. \
The trajectory is set by the user and is described as an ellipsoid:

<div align="center">
  <img src=".github/ellipse.png" width="600" >
</div>

## Parameters:
- **--name**           : changes the rootname of the module ports
- **--robot**          : changes the name of the robot where the module interfaces to
- **--name**           : rootname for all the connection of the module
- **--part**           : selected arm
- **--pitchDof**       : 0/1 disable/enable the DoF
- **--yawDof**         : 0/1 disable/enable the DoF
- **--rollDof**        : 0/1 disable/enable the DoF
- **--gazeTracking**   : enable gaze tracking
- **--help**           : get help from terminal

## Motion profiles:
It is possible to adopt different motion profiles


**Generate constant velocity profile:**
```
GEN CVP (((O -0.3 -0.1 0.1) (A -0.3 -0.0 0.1) (B -0.3 -0.1 0.2) (C -0.3 -0.1 0.0) (theta 0.0 1.57 4.71) (axes 0.1 0.1) (rev) (param 0.1)))
```

**Generate minimum jerk profile:**
```
GEN MJP (((O -0.3 -0.1 0.1) (A -0.3 -0.0 0.1) (B -0.3 -0.1 0.2) (C -0.3 -0.1 0.0) (theta 0.0 1.57 4.71) (axes 0.1 0.1) (rev) (param 1.57 3.0)))
```

**Generate general velocity profile:**
```
GEN GVP((O -0.3 -0.0 0.1) (A -0.3 0.1 0.1) (B -0.3 -0.0 0.2) (C -0.3 -0.0 0.0) (theta 0.0 1.57 4.71) (axes 0.1 0.1) (rev) (param 0.1)))
```

**Generate NON two-third power law profile:**
```
GEN TTPL (((O -0.3 -0.1 0.1) (A -0.3 -0.0 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 0.0) (theta 0.0 1.57 4.71) (axes 0.1 0.2) (rev) (param 0.1 0.33)))
```

**Generate two-third power law profile:**
```
GEN TTL (((O -0.3 -0.1 0.1) (A -0.3 -0.0 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 0.0) (theta 0.0 1.57 4.71) (axes 0.1 0.2) (rev) (param 0.01 0.33)))
```

## Other Useful commands: 
- PALM CUS : to change the orientation of the palm 
- PALM CUS (-0.076 -0.974 0.213 3.03) 
- START simulation and execute
- STAR SIM : start simulation (yellow)
- STAR EXE : start execution  (green)
- STAR RES : start resetting of the posture
- SIM CLR  : simulator cleaning
- STAR FILE SPEE #value: start execution from file with speed multiplied for the value, if omitted default value is 1.0
- SAVE JOI: save joints positions in file
- MARK START/STOP: start/stop time interval
- SYNC: set time interval decided with MARK START/STOP
- REPS #value: set number of repetitions for file execution
- LOAD FILE: set the name of the file to load
- GRAS ON: turn on grasping
- GRAS RES: reset the open hand
- GRAZ CVV (params): constant velocity grasping


Mantainers
==========
Perfect code does not exist, we try our best.\
For info:

| Name | Mail | GitHub |
|:----:|:----:|:------:|
| Luca Garello   | Luca.Garello@iit.it   | [@MissingSignal](https://github.com/MissingSignal) |
| Linda Lastrico | Linda.Lastrico@iit.it | [@lindalastrico](https://github.com/lindalastrico) |
| Fabio Vannucci | Fabio.Vannucci@iit.it | Unknown |
