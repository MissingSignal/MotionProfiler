# Motion Profiler 2.0
<div align="center">
  <img src=".github/Changelog.png" >
</div>

This module allows to control the velocity of icub's end-effector. </br>
The trajectory is set by the user and is described as an ellipse:

<div align="center">
  <img src=".github/Ellisse.png" width="600" >


|    **O**     |      **A**    |   **B** | **θ<sub>start</sub>** | **θ<sub>stop</sub>** |
|:--------:|:-----------:|:-------------:|:----------:|:----------:|
| *center (m)*    | *vertex (m)*  |  *co-vertex (m)*  | *starting angle (rad)* | *final angle (rad)* |
</div>

## Parameters:
- **--name**           : changes the rootname of the module ports
- **--robot**          : changes the name of the robot where the module interfaces to
- **--name**           : rootname for all the connection of the module
- **--part**           : selected arm
- **--pitchDof**       : 0/1 disable/enable the torso DoF
- **--yawDof**         : 0/1 disable/enable the torso DoF
- **--rollDof**        : 0/1 disable/enable the torso DoF
- **--gazeTracking**   : enable gaze tracking
- **--help**           : get help from terminal

## Motion profiles:
It is possible to generate different motion profiles


**Generate constant velocity profile:**
```
GEN CVP (((O -0.3 -0.1 0.1) (A -0.3 -0.0 0.1) (B -0.3 -0.1 0.2) (theta 0.0 1.57) (param 0.1)))
```

**Minimum jerk profile:**
```
GEN MJP (((O -0.3 -0.1 0.1) (A -0.3 -0.0 0.1) (B -0.3 -0.1 0.2) (theta 0.0 1.57) (param 1.57 3.0)))
```

**General velocity profile:** \
Custom velocity profile read from file
```
GEN GVP (((O -0.3 0.1 0.1) (A -0.3 0.2 0.1) (B -0.3 0.1 0.2) (theta 0.0 3.14) (vel 0.3 0.3 0.3)))
```

**NON two-third power law profile:**
```
GEN TTPL (((O -0.3 -0.1 0.1) (A -0.3 -0.0 0.1) (B -0.3 -0.1 0.3) (theta 0.0 1.57) (param 0.1 0.33)))
```

**Two-third power law profile:**
```
GEN TTL (((O -0.3 -0.1 0.1) (A -0.3 -0.0 0.1) (B -0.3 -0.1 0.3) (theta 0.0 1.57) (param 0.01 0.33)))
```

## Other Useful commands:
- ```PALM CUS``` : to change the orientation of the palm *e.g. PALM CUS (-0.076 -0.974 0.213 3.03)*
- ```STAR SIM```: start simulation (yellow trajectory in iCubGUI)
- ```STAR EXE```: start execution  (green trajectory in iCubGUI)
- ```STAR RES```: start resetting of the posture
- ```SIM CLR```: simulator cleaning
- ```STAR FILE SPEE #value```: start execution from file with speed multiplied for the value, default value is 1.0
- ```SAVE JOI```: save joints positions in file
- ```MARK START/STOP```: start/stop time interval
- ```SYNC```: set time interval decided with MARK START/STOP
- ```REPS #value```: set number of repetitions for file execution
- ```LOAD FILE```: set the name of the file to load
- ```GRAS ON```: turn on grasping
- ```GRAS RES```: reset the open hand
- ```GRAZ CVV (params)```: constant velocity grasping


Mantainers
==========
Perfect code does not exist, we try our best.\
For info:

| Name | Mail | GitHub |
|:----:|:----:|:------:|
| Luca Garello   | Luca.Garello@iit.it   | [@MissingSignal](https://github.com/MissingSignal) |
| Linda Lastrico | Linda.Lastrico@iit.it | [@lindalastrico](https://github.com/lindalastrico) |
| Fabio Vannucci | Fabio.Vannucci@iit.it | Unknown |
