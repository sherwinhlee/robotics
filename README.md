# robotics
## Introduction

This repository contains a robotics programming environment to model connected and autonomous vehicles (CAVs) using iRobot creates (or "Roomas," as they are more commonly known). The core component is an acceleration controller which drives the robots with a given acceleration input. Additional parameters can be passed in to model various control scenarios. This initial version includes an "ECO-AND" scenario that minimizes fuel consumption and travel time, as well as a human driver trial, both of which use assumptions from [Meng and Cassandras (2019)](https://arxiv.org/pdf/1901.11423.pdf). The code is written specifically for trial experimentation in the Boston University (BU) Shared Robotics Lab (SRL). 

## Model & Environment 
This environment is designed around the following model representations:
- Each **iRobot create robot** models one **CAV** entity  
- The **SRL lab floor** models the University of Michigan **Mcity** test facility road network

Given the velocity maxima of the creates and the CAV specifications as outlined in [(2)](https://ieeexplore.ieee.org/abstract/document/8618939), the model currently assumes a scale of 1:32.5. This, of course, can be adjusted, but should not violate any environmental constants (e.g., a maximum CAV speed should not translate to a robot velocity beyond the hardware max). All of the assumed environmental constants for both the Mcity and SRL worlds are listed here:
- The Mcity road segment (also referred to as the "control environment") length (`L`) is set to 70 m.
- The maximum CAV velocity (`vM`) is set to 15 m/s. The robot's maximum is capped at 0.4 m/s.
- The minimum CAV velocity (`vm`) is set to 2.78 m/s.
- The maximum CAV acceleration (`aM`) is set to 2.5 m/s<sup>2</sup>.
- The maximum CAV deceleration (`am`) is set to -2.9 m/s<sup>2</sup>.

Beyond these constants, other environmental variables, like traffic light cycle intervals, can be specified as user parameters.

## Repository
The repository is divided into two modules: **controller**, which contains all code pertaining to the control and movement of robots, and **helper**, which contains an assortment of helper functions. Experiments can be run directly from `main.py`, through which all trial parameters can be specified. An overview of the module is outlined below with additional class and function-specific docstrings located in each file:
### controller 
- `controller.acceleration` is the main acceleration controller file. It can be called through the `main.py` wrapper or imported into another file. Its main `AccControl()` class takes as arguments the experiment trial type, specific robot being used, a constant acceleration command, a constant velocity command (with no acceleration), a calibration parameter for the motion capture system, and a delay interval before the robots begin to move.
- `controller.lateral_control` acts as a helper function to control the lateral motion of the robot as it travels along a straight-line path. It computes an angular velocity command when the robot drifts outside a 3cm threshold.
- `controller.eco_and` is the optimization module that computes an optimal acceleration profile. It formulates a nonlinear constrained optimization problem based on [(1)](https://arxiv.org/pdf/1901.11423.pdf), and uses scipy's SLSQP (Sequential Least-Squares Programming) solver. The problem takes as inputs an initial velocity, traffic light information (red/green light cycle times, starting light, time remaining of the starting light's interval), distance to the intersection, and starting points for the decision variables. Additional parameters are assumed constant within the environment (see **Model & Environment**).
- `controller.trials` contains helper functions that run the experiment specified by the `trial` argument passed into `controller.acceleration`'s `AccControl()` class. The two trials that can be evaluated in this initial implementation are `"ECO-AND"` and `"HUMAN"`. Otherwise, the default argument is `None` and the controller runs either a constant acceleration or velocity experiment.
- `controller.config` is a configuration file that initializes all the global variables that need to be accessed or updated across modules as each experiment is run. It is called directly from `main.py`.
### helper
- `helper.scale` is a simple helper function that calculates the scaling transform between the Mcity world and the lab environment. As noted above, the default scale is currently set to 1:32.5.
- `helper.trajectory` is a plotting module that plots the robot's trajectory after an experiment is completed. The `logplot()` function (not to be confused with a logarithmic plot) plots data that has been exported out to a log text file. The `liveplot()` function is called directly from any file running the `controller.acceleration` module as it uses the class's robot position variables - it immediately produces a plot after the experiment is killed.
- `helper.logprint` is a printing module that prints a running log of the robot's information as it travels along a path. It is called directly from `controller.acceleration` and will either print to the console or write to an external text file depending on whether or not there is a `sys.stdout` command in the acceleration controller, which is not included by default.

## Setup Instructions
As specified earlier, this programming environment is written specifically for use in the Boston University (BU) Shared Robotics Lab (SRL). The following points outline the experimental setup conditions and instructions for running CAV control trials. More detailed setup information can be found through the [BU Robotics Lab Wiki](http://wiki.bu.edu/robotics/index.php/Main_Page). Specific SRL inquiries should be directed to Zack Serlin: zserlin@bu.edu. 

### Setup
Prior to running experiments, the following computers will need to be set up:
- The Linux computer (**"burobotics"**) is the primary machine through which the Robot Operating System (ROS) and programming will run
- The Windows computer #1 (**"windows1"**) contains the *Motive* software that runs the *OptiTrack* motion capture system.
- The Windows computer #2 (**"windows2"**) contains the *Resolume Arena* software for projecting Mcity onto the lab floor from its six projectors.

### Instructions
1. Undock the desired create robot from the wall charger, place it on the floor of the SRL experimental area, and point it directly at the south wall (where I-90 is).
2. Open *Motive* on windows1 and load the most recent calibration file. 
3. Identify the create in *Motive*, select its markers, and right-click to create a rigid body. Name the rigid body "create#" with the # replaced by the create number.
4. Turn on the lab floor projectors and open *Resolume Arena* on windows2. Load the Mcity map from the right-hand side 'Files' pane and drag the image to the top-left cell under "Column 1." Scale/position the image accordingly. Projection display settings can be configured in Output > Advanced.
5. On burobotics, start a new Terminator session with multiple windows.
6. In one window, run `roscore` and in another run `roslaunch vrpn_client_ros multirobot.launch server:=optitrack`.
7. Verify the create's heading/orientation configuration by running `$ rostopic echo /create#/pose` in a third window. The robot's position information will then be outputted live to the terminal window. Under `orientation`, note the z and w values of the orientation's quaternion. Convert these values to a euler angle (in degrees). This angle will be the `zero_hd` parameter in the `controller.acceleration.AccControl()` class. In a default environment, the zero heading is the SRL's south wall and this value should be 0.
8. Connect to the create's raspberry pi in another Terminator window using `ssh pi@create#`. Then run `$ roslaunch ca_driver create_2.launch`. 
9. In a fifth window, drive the create to its desired start point using the xbox360 controller and this launch command: `$ roslaunch ca_tools joy_teleop.launch [joy_config:=xbox360]`.
10. Line the robot up with its represented start point in the Mcity projection, ensuring the robot is facing the west wall (this is important!). After the robot is positioned, kill the joystick launch file.
11. In another Terminator window, `cd` into the directory where the repository is stored.
12. Open `main.py` using a text editor/IDE and adjust the environment parameters as needed. See the docstring for `controller.acceleration.AccControl()` before creating an instance of the acceleration controller object.
13. Run `$ python main.py` to start the control experiment.

### Notes
1. Depending on which motion capture / VRPN launch file is used, the create robot's namespace in the ROS subscriber may be different. This can be verified with `$ rostopic list` to see what the full namespace of `/create#/pose` is. The `self.mocap` attribute in `controller.acceleration.AccControl()` can then be updated accordingly. 
2. The optimization module in `controller.eco_and` uses a sequential least-squares programming solver, which can be sensitive to the chosen starting points, particularly tau or `t0`, which is the intersection crossing time. The code sets the starting tau to be the starting light's interval, but in some cases, this may not lead to local minimum convergence.

## References
1. [A Real-Time Optimal Eco-driving for Autonomous Vehicles Crossing Multiple Signalized Intersections](https://arxiv.org/pdf/1901.11423.pdf). Xiangyu Meng, Christos G. Cassandras. January 2019.
2. [Optimal Control of Autonomous Vehicles for Non-Stop Signalized Intersection Crossing](https://ieeexplore.ieee.org/abstract/document/8618939). Xiangyu Meng, Christos G. Cassandras. IEEE Conference on Decision and Control, 2018.
