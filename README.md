# FCND Controls Project

The goals/steps of this project are following:

* Update `backyard_flyer.py` from first project to use the controllers.
* Write the low level flight controllers in `controller.py` to follow a timed trajectory.
* Modify a C++ controller


[//]: # (Image References)
[image1]: ./images/ControlStructure.png
[image2]: ./images/bodyrate_eq1.png
[image3]: ./images/bodyrate_eq2.png
[image4]: ./images/bodyrate_eq3.png
[image5]: ./images/R.png
[image6]: ./images/R2.png
[image7]: ./images/rollpitch_eq1.png
[image8]: ./images/rollpitch_eq2.png
[image9]: ./images/rollpitch_eq3.png
[image10]: ./images/rollpitch_eq4.png
[image11]: ./images/rollpitch_eq5.png
[image12]: ./images/yaw_eq1.png
[image13]: ./images/altitude_eq1.png
[image14]: ./images/altitude_eq2.png
[image15]: ./images/altitude_eq3.png
[image16]: ./images/altitude_eq4.png
[image17]: ./images/altitude_eq5.png
[image18]: ./images/altitude_eq6.png
[image19]: ./images/lateral_eq1.png
[image20]: ./images/lateral_eq2.png
[image21]: ./images/lateral_eq3.png
[image22]: ./images/motorthrust_eq1.png
[image23]: ./images/altitude_eq7.png
[image24]: ./images/motorthrust_eq2.png
[image25]: ./images/motorthrust_eq3.png
[image26]: ./images/motorthrust_eq4.png
[image27]: ./images/motorthrust_eq5.png
[image28]: ./images/trajectory.png
[image29]: ./images/trajectory_compare.png
[image30]: ./images/horizontal_error.png
[image31]: ./images/vertical_error.png
[image32]: ./images/scenario2.gif
[image33]: ./images/scenario3.gif
[image34]: ./images/scenario4.gif
[image35]: ./images/scenario5.gif
[image36]: ./images/test_trajectory.gif

## Setup

### Python

* Download the [simulator](https://github.com/udacity/FCND-Simulator-Releases/releases)
* Setup the python [envirnonment](https://github.com/udacity/FCND-Term1-Starter-Kit)
* Start the simulator and run the `controls_flyer.py` script.

```
source activate fcnd
python controls_flyer.py
```

### C++

Here are the setup and install instructions for each of the recommended IDEs for each different OS options:

#### Windows

1. Download and install [Visual Studio](https://www.visualstudio.com/vs/community/)
2. Select *Open Project / Solution* and open `<simulator>/Simulator.sln`
3. From the *Project* menu, select the *Retarget solution* option and select the Windows SDK that is installed on your computer (this should have been installed when installing Visual Studio or upon opening of the project).
4. To compile and run the project / simulator, simply click on the green play button at the top of the screen.  When you run the simulator, you should see a single quadcopter, falling down.


#### OS X

1. Download and install XCode from the App Store if you don't already have it installed.
2. Open the project from the `<simulator>/project` directory.
3. After opening project, you need to set the working directory:
  1. Go to *(Project Name)* | *Edit Scheme*
  2. In new window, under *Run/Debug* on left side, under the *Options* tab, set Working Directory to `$PROJECT_DIR` and check ‘use custom working directory’.
  3. Compile and run the project. You should see a single quadcopter, falling down.


#### Linux

1. Download and install QtCreator.
2. Open the project from the `<simulator>/project` directory.
3. Compile and run the project.  You should see a single quadcopter, falling down.

## Python Control System

![alt text][image36]

I implemented the control system in `controller.py`. All python control system files are in the directory `./python`.

The controller is separated into five parts:

 - body rate control
 - reduced attitude control
 - altitude control
 - heading control
 - lateral position control

These fit together as the structure shown below:

![alt text][image1]

### 1. Body rate controller

The commanded and actual body rate are collected by a P controller, and they are translated into the desired moment along the axis in the body frame.

I first calculated the body rate error: 

![alt text][image2]

I obtained the desired rotational accelerations along the axis in the body frame:
 
![alt text][image3]

Then I translated the desired accelerations to moment with moment of inertia:

![alt text][image4]

I further limited the moment with a maximum Euclidean norm of 1.0 N·m.

I did this in the function `body_rate_control()` in lines 170 to 184 of `controller.py`.

### 2. Roll-pitch controller

The roll-pitch controller is a P controller responsible for commanding the roll and pitch rates in the body frame. The goal here is to generate roll and pitch commands to control lateral acceleration.

To transform between the world framd and body frame, I first used the attitude to generated the rotation matrix R:

![alt text][image5]

The x, y, z rotations are:

![alt text][image6]

I did this using the function `euler2RM()` in `frame_utils.py`. 


Then I translated the commanded lateral accelerations to elements in rotation matrix:

![alt text][image7]

where

![alt text][image8]  
![alt text][image9]

The c value is the total thrust command in Newton converted into acceleration.

I obtained the desired rate of change of the given elements in rotation matrix by:

![alt text][image10]

Then I converted the desired change in rotation matrix into the angular velocities in body frame by following matrix multiplication:

![alt text][image11]

I also omitted downward thrust command by setting the roll and pitch command to 0 - because a real drone only has upward thrusts.

I did this in the function `roll_pitch_controller()` in lines 135 to 168 of `controller.py`.


### 3. Yaw controller

A P controller is used to control the drone's yaw.

![alt text][image12]

To avoid making uncessary turns, I optimized the controller by wraping the yaw command to [0, 2π], and the yaw error to [-π, π].

I did this in the function `yaw_control()` in lines 188 to 210 of `controller.py`. 

### 4. Altitude controller

The goal here is to generate thrust command to control vertical acceleration. 

Linear acceleration can be expressed by the next linear equation:

![alt text][image14]

where R is the rotation matrix:

![alt text][image5]

The vertical acceleration has the form of:

![alt text][image15]  

where

![alt text][image18]

We are controlling the vertical acceleration:

![alt text][image16]

Therefore

![alt text][image17]

A PD controller is used for the altitude which results in:

![alt text][image13]

I did this in the function `altitude_control()` in lines 110 to 132 of `controller.py`.

### 5. Lateral controller

A PD controller is used to command target values for lateral accelerations. The drone generates lateral acceleration by changing the body orientation which results in non-zero thrust in the desired direction. The errors in x and y position and velocity will translate into the desired commanded accelerations:

![alt text][image19]  
![alt text][image20]  

The control equations have the same form for y direction as above.

I did this in the function `lateral_control()` in lines 89 to 108 of `controller.py`.

### Test the controller

I tested the controller following against a trajectory defined in `test_trajectory.txt`. To pass the test, the controller should be able to satisfy following performance metrics:

* The drone flies the test trajectory faster than the default threshold (20 seconds)
* The maximum horizontal error is less than the default threshold (2 meters)
* The maximum vertical error is less than the default threshold (1 meter)

I obtained following results:  

```
Maximum Horizontal Error:  1.785402058486391
Maximum Vertical Error:  0.6485167092411652
Mission Time:  1.630382
Mission Success:  True
```

#### 2D plots of the controller flying the test trajectory

![alt text][image28] 

#### 2D position overplotted with designed 2D position

![alt text][image29]

#### Horizontal position error vs. time from test trajectory

![alt text][image30]

#### Vertical position error vs. time from test trajectory

![alt text][image31]

## C++ Control System
I rebuilt the python controller into C++. All C++ files are in the directory `./cpp`. 

* `config/QuadControlParams.txt`: the config file for the controller. While the simulator is running, you can edit this file in real time and see the affects of your changes have on the drone.
* `src/QuadControl.cpp`: the implementation of the controller. 

### 1. Body rate controller

#### Setting the individual motor commands

Based on the input from the controller (i.e., moment command from body rate controller and total thrust command from roll-pitch controller), we can set the individual motor thrust commands.

I use F1, F2, F3, F4 to denote the individual thurst command for the front left, front right, rear left, rear right motors. 

There are forces generated by propellers - the collective force directed upward is:

![alt text][image22] 

The moments are also created by propellers. Roll is produced by the moments from 1st and 3rd propellers conteracted by moments from 2nd and 4th propellers:

![alt text][image24]

Pitch is produced by the moment from 1st and 2nd propellers mismatch with moments from 3rd and 4th propellers:

![alt text][image25]

Here 𝛕 is the moment. 

The `l` is a distance between x-axis and propeller location, which is equal to half of the distance between neighboring propellers, and each arm is always at 45° relative to each axis:

![alt text][image26]

Yaw is produced by the mismatch between moments from propellers along z-axis by reactive forces.

![alt text][image27]

I did this in the function `GenerateMotorCommands()` in lines 56 to 85 in `QuadControl.cpp`.

#### Implement the body rate controller

I simply converted the body rate controller from python to C++ here, in the function `BodyRateControl()` in lines 87 to 105 of `QuadControl.cpp `.

### 2. Roll-pitch controller

I simply converted the roll-pitch controller from python to C++ here, in the function `RollPitchControl()` in lines 108 to 139 of `QuadControl.cpp `.

### 3. Lateral controller

I converted the lateral controller from python to C++, in the function `RollPitchControl()` in lines 108 to 139 of `QuadControl.cpp `.

I further limited the velocity and acceleration commands under their maximum values, and made sure to return the desired horizontal accelerations by setting the Z component to 0.

### 4. Altitude controller

To cope with non-idealisties and achieve robustness, I converted the PD lateral controller from python to C++, and moved it to PID controller. 

The controller accumulates error over time, and then adjust the system to counter the accumulation of error:

![alt text][image23]

I did this in the function `AltitudeControl()` in lines 56 to 85 in `QuadControl.cpp`.

### 5. Yaw controller

I simply converted the yaw controller from python to C++ here, in the function `YawControl()` in lines 218 to 244 of `QuadControl.cpp `.

### Test the controller

#### Scenario 2: Attitude control

The drone above the origin was created with a small initial rotation speed about its roll axis. 

I implemented `GenerateMotorCommands()`, `BodyRateControl()`, `RollPitchControl()`, tuned `Kp_pqr` and `Kp_bank` to stabilize the rotational motion and bring the vehicle back to level attitude.

![alt text][image32]

```
PASS: ABS(Quad.Roll) was less than 0.025000 for at least 0.750000 seconds
PASS: ABS(Quad.Omega.X) was less than 2.500000 for at least 0.750000 seconds
```

#### Scenario 3: Position control

Two identical drones were created, while both offset from their target point, one was initialized with yaw = 0 and another was initialized with yaw = 45 degrees.

I implemented `LateralPositionControl()`, `AltitudeControl()`, `YawControl()`, tuned `Kp_pos_z`, `Kp_vel_z`, `Kp_vel_xy`, `Kp_vel_z`, `Kp_yaw`, and the z component of `Kp_pqr` to move the drones to their target points.	

![alt text][image33]

```
PASS: ABS(Quad1.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Yaw) was less than 0.100000 for at least 1.000000 seconds
```

#### Scenario 4: Non-idealities and robustness

Three drones were created and all are trying to move one meter forward, but these drones are all a bit different:

* The green quad has its center of mass shifted back
* The orange vehicle is an ideal quad
* The red vehicle is heavier than usual

I added basic integral control to `AltitudeControl()` and tuned several parameters to help all drones successfully move properly.

![alt text][image34]

```
PASS: ABS(Quad1.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad2.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
PASS: ABS(Quad3.PosFollowErr) was less than 0.100000 for at least 1.500000 seconds
```

#### Scenario 5: Trajectory following

Two drones were created to follow same trajectory. The orange one is following `traj/FigureEight.txt`, and another one is following `traj/FigureEightFF.txt`.

I tuned the parameters again to help the drones pass the test.

![alt text][image35]

```
PASS: ABS(Quad2.PosFollowErr) was less than 0.250000 for at least 3.000000 seconds
```