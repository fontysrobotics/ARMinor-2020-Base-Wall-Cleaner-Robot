# Base Wall Cleaner

## Table of Contents

- [Introduction](#intro)
- [Videos](#videos)
- [Installation for ROS machine](#install_ros)
- [Installation for Web Server](#install_web)
- [Running the ROS App](#run_ros)
- [Running the Web App](#run_web)
- [Current issues and future work](#issues)

---

<a name="intro"></a>
## Introduction
The basewall cleaner, is a holonomic robot with dimensions 281mm x 306mm x 141mm. It is intended to automate the cleaning of basewall boards in different rooms so that the cleaning staff can work more efficiently.
The robot is placed in a random room. The camera and lidar are then turned on so that the robot can detect the walls by using the lidar. The robot navigates through the room by following the wall on its right. The distance from the walls to the robot is 40 cm. At the same time as navigating the room, the robot cleans the basewall boards. The robot knows when it encounters an obstacle because the front of the robot will not be covered in its entirety like a wall. If the robot wants to go through a path too narrow to continue, has detected an obstacle or gets stuck for some unknown reason, it takes a picture of the relevant place which can be seen on the website for the cleaning staff. This allows them to know where the robot is positioned, and to help the robot out.

More information on the implementation is available [Here](https://github.com/fontysrobotics/ARMinor-2020-Base-Wall-Cleaner-Robot/tree/master/documents/report.pdf)

<a name="videos"></a>

**This project is one of the deliverables of the [ROSIN EP FREROS project] (https://www.rosin-project.eu/ftp/freros). It was developed for the Adaptive Robotics minor of Fontys University of Applied Sciences which aims to be one of the best ROS and Robotics education center's in the Netherlands.**

## Videos
Videos of the robot are available [Here](https://github.com/fontysrobotics/ARMinor-2020-Base-Wall-Cleaner-Robot/tree/master/documents/videos)

---

<a name="install_ros"></a>
## Installation for ROS machine

### Install ROS
Install ROS by following the instruction at `http://wiki.ros.org/kinetic/Installation/Ubuntu`

### Clone git repo
Clone this git repo into your home directory by using the following commands

```
>> cd ~
>> git clone https://github.com/fontysrobotics/ARMinor-2020-Base-Wall-Cleaner-Robot
>> mv ARMinor-2020-Base-Wall-Cleaner-Robot ar_base_wall
```
### Set up bash profile
Set up your bash profile by running the setup script
```
>> cd ~/ar_base_wall/scripts
>> bash SetupBashProfile.bash
```

---

<a name="install_web"></a>
## Installation for Web Server
### Clone git repo
Clone this git repo into your home directory by using the following commands

```
>> cd ~
>> git clone https://github.com/fontysrobotics/ARMinor-2020-Base-Wall-Cleaner-Robot
>> mv ARMinor-2020-Base-Wall-Cleaner-Robot ar_base_wall
```
### Install web dependencies
Install web dependencies by running the install script
```
>> cd ~/ar_base_wall/scripts
>> bash SetupWebApp.bash
```
<a name="run_ros"></a>
## Running the ROS app
> Each of the following commands should be run in a separate terminal window

### Run the simulation
This starts Gazebo and loads the robot model and the room
```
>> bash RunSimulation.bash
```

### Run Rosbridge
Rosbridge is required for communication between the web app and the ros app
```
>> bash RunRosbridge.bash
```

### Run Robot Application
Start the robot application, the robot will be in an idle state until it receives the start command from the web application
```
>> bash RunRobot.bash
```

---

<a name="run_web"></a>
## Running the web app
Ensure that the IP addres in ~/ar_base_wall/web_application/static/js/main.js points to the IP of the ROS machine

### Run the web app
```
>> cd ~/ar_base_wall/scripts
>> bash RunWebApp.bash
```

### Open website in browser
Browse to http://localhost:5000 to open the website

---

<a name="issues"></a>
## Current issues and future work

### Current issues
- When the robot is unable to find a wall it will use a vortex maneuver to move in an increasingly large circle until it detects a wall. This is an ineffecient way of finding a wall and the Lidar could be utilized better to either determine where the closest wall is or pick a direction in which to move.
- Using the Turtlebot3 makes it difficult to navigate closer (10cm) to the wall, another robot with a different wheel-setup could allow this.

### Future work
- The current program is fully simulated, it needs to be implemented on a physical robot.
- A cleaning arm must be chosen and attached to the robot (the robot does currently have the option of facing a wall to either the left or the right).
- Currently the Lidar is only used for close range detection, it could be adapted to use both close and far range detection in order to detect obstacles early and still be aware of dynamic object at close range.
