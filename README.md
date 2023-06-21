# AMIGA control package

This repo contains eveything to use AMIGA (**A**ssistive **M**obile **I**nteractive **G**rasping **A**gent), in the real world or in simulation.
2 versions are available:
 - ROS Melodic (depreciated), on branch [melodic](https://github.com/ImperialCollegeLondon/amiga_main/tree/melodic)
 - **ROS Noetic**, on branch [main](https://github.com/ImperialCollegeLondon/amiga_main) 

Different features are available on each branch (Melodic TODO @NicoLingg; RR = real robot, G = Gazebo):
Feature | RR base ctrl | RR arm ctrl | RR grasping | G base ctrl | G arm ctrl | G grasping 
--- | --- | --- | --- |--- |--- |--- 
Melodic | :question: | :heavy_check_mark: | :heavy_check_mark: | :x: | :x: | :x: 
Noetic | :heavy_check_mark: [here](https://github.com/ImperialCollegeLondon/rnet-wheelchair-docker) | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :x: 


# Turning everything on
1) Turn on the laptop
2) Turn on the base (orange switch on the side of the base)
3) Make sure the black toggle switch on the side of the box with the orange switch is facing down 
4) Press the round button close to the orange one to turn the arm on

# Initial Setup

## TigerVNC
AMIGA has an onboard laptop that can be used through a VNC, *i.e.* a virtual remote desktop. This laptop has all the docker components, which means that you won't need to install anything onyour own computer (such as the 20GB docker images we are using). These steps only need to be done once when you are a new user, to create your own VNC.
1) Make sure you have an SSH key on your computer, and that it is installed on the laptop (`ssh-copy-id`, username is `amiga` and the passwowrd is the lab's password)
2) SSH into the laptop
3) Run `cp toggle_vnc.sh .[YOURNAME].sh`. This will create a copy of the VNC script for you.
4) Edit the resulting file (for instance using `vim .[YOURNAME].sh`) and change the `VNC_PORT=XXX`; put a random number between 20000 and 30000. This will be your VNC port.
5) Run `chmod +x .[YOURNAME].sh` to make the script executable
6) On your computer, edit your `~/.ssh/config` file to set up the port forwarding. Add the following lines:
```
Host amiga_laptop
    HostName [IP OF THE LAPTOP]
    User amiga
    LocalForward [YOUR VNC PORT] localhost:[YOUR VNC PORT]
```
You are now ready to use the VNC!

## Your copy of the code
Since we all share a user, you will need to create a folder with your name in the home, and put your code there. SSH in the laptop, and run:
```bash
mkdir [YOURNAME]
cd [YOURNAME]
git clone  git@github.com:ImperialCollegeLondon/amiga_main.git
cd amiga_main
git checkout -b [YOUR BRANCH]  # Avoid using your name; use something describing your work instead ("coooking", "manipulation"...)

# Create your own repo for your work on github in the IC organisation

# Then link it to a local repo
mkdir [YOUR REPO] # (amiga_cooking, amiga_kinematics...)
cd [YOUR REPO]
git init
touch README.md
git add README.md
git commit -m "first commit"
git branch -M main
git remote add origin [YOUR REPO URL]
git push -u origin main

# Then add it as a submodule to the main repo
cd ..
rm -Rf [YOUR REPO]
git submodule add [YOUR REPO URL]

# Initialise all submodules
git submodule update --init --recursive

cp .env.example .env
vim .env  # Set the right variables here


docker login  
# Use the lab's account:
# personalroboticsimperial
# [lab's password]
```

Your environment is now setup!


# Working with the robot
0) Turn on everything (see beginning of the README)
1) SSH into the laptop using `ssh amiga_laptop`
2) On the laptop, start the vnc using `./.[YOURNAME].sh`
3) On your computer, start the VNC client `vncviewer`
4) Connect to `localhost:[YOUR VNC PORT]`
5) The password is the lab's password

You then have a remote desktop to the laptop, and you can use it as if you were in front of it.

6) Open a terminal (click in the top left on "Applications", then click on the terminal icon)
7) Navigate to your code folder (`cd [YOURNAME]/amiga_main`)
8) Run `make build-external` to build your version of the docker image


You're ready to run modules! The available modules are (listed in the order I usually run them):
- `description`: publishes the robot's description on `/robot_description`. This will launch joint state publishers for the arm if `ENABLE_ARM` is set to `false` in `.env`
- `arm-driver`: starts the UR10e driver. Make sure the arm is in `remote` mode before running this module, otherwise you will be able to see the joint states, but not send commands.
- `restart-obj-detect`: launches the gripper camera driver and the object detection node. This takes ~1min to start.
- `arm-moveit`: this turns on the arm, unlocks the joints, and initialises the gripper. It take ~1 min to complete. You can also use `arm-moveit-simple` if the arm is already on and the gripper already initialised, or `arm-moveit-gazebo` if you are working in simulation.
- `rviz`: launches RViz with the correct configuration, based on what you put in `.env`
- `manip-server`: launches the MoveIt high level wrapper. If RViz is running with the "External communication" box checked, goals will be published to RViz.

Additionnaly, a few other `make` commands are available to call specific services: 
- `init- / open- / close-gripper`: self explanatory
- `reset-arm`: to unlock protective stop
- `exec`: to enter the container
- `stop`: to stop the container

Finally, every person using the robot should have one or several `make` commands for their work. So far, we have:
- `cook` for [@cedricgoubard](https://github.com/cedricgoubard)'s work on trust-aware assistive cooking

# Detailed Architecture

## Hardware

AMIGA is made of 4 main parts:
- The **ARNIE base**, common to most robots in our lab. ARNIE is our appellation for the common bundle made of:
  - the Quickie wheelchair base
  - a Raspberry Pi enabling us to send ROS commands to Quickie
  - A ZED mini camera + 2 RPlidars (previously an Intel T265 + 2 Intel L515)
  - a Jetson Xavier connected to those sensors
- The UR10e arm, with a Robotiq 3f gripper, an ZED2 camera mounted on the gripper, and a Jetson Orin connected to the camera
- An laptop mounted at the back
- A router and a switch connected everything together

The load distribution is as follows:
- **Laptop**: MoveIt, MoveBase, RViz, UR10e driver
- **Jetson Xavier**: ZED Mini & RPLidars drivers, SLAM
- **Jetson Orin**: ZED2 driver, object detection

Any one of them can be the ROS master (defined in `.env`).

# Software

## VNC

The main way to interact with the robot is through a VNC (*i.e.* a remote desktop.) on the laptop. To use it, follow the steps in the "How to use" section.

## Docker
Each computer (the laptop and the 2 Jetsons) runs everything in a docker container. The same approach is taken everywhere: all `make` commands will start the container if it is not already running, and then `exec` inside to run their code.

The associated `Dockerfiles` can be found in the `dockerfiles` folder. All base images are ones we created in our [PRL-Dockerfiles](https://github.com/ImperialCollegeLondon/PRL-Dockerfiles) repo; if you need those images, either use `docker login` with the lab's account to be able to pull them from the Docker Hub, or build them yourself.

## Make

We use GNU Make to do everything; while it is usually used to build C/C++ code, we just use it to simplify the commands we have to run. The `Makefile` is in the root of the repo, and contains all the commands we need.

## Quick overview of the submodules
In alphabetical order:
- `amiga_collisionIK` contains the CollisionIK configuration files for our robot. This is used by `relaxed_ik`, which itself is used by `amiga_moveit_config` and MoveIt.
- `amig_cooking_assistant` has [@cedricgoubard](https://github.com/cedricgoubard)'s code for the cooking assistant.
- `amiga_description` self explanatory. It builds upon the ARNIE URDF from `arnie_description`.
- `amiga_driver` is the main entry point: it has launch files for the robot's driver, MoveIt, and object detection.
- `amiga_moveit_config`: self explanatory.
- `amiga_moveit_wrapper`: the high level wrapper for MoveIt (sometimes called the "Manipulation Server"). It offers plenty of useful services.
- `amiga_sim`: contains the worlds and launch files to work with Gazebo.
- `arnie_description`: self explanatory.
- `arnie_localisation`: camera / LIDAR drivers and SLAM (RTABMAP).
- `arnie_main`: main entry point for everything related to SLAM / navigation.
- `arnie_navigation`: MoveBase configuration. ROS node to interface with the RNET driver. Everything related to move_base should be moved in here at some point.
- `collision_ik_moveit_wrapper`: C++ node to enable collisionIK as a MoveIt plugin.
- `prl_grasping`: attempt at a unified grasping framework. We only use its `check_planarity` node for now.
- `prl_ur_kinematics`: alternative IK solver for the UR10e. We don't use it anymore, but it is still useful to have it around.
- `relaxed_ik`: the actual implementation of CollisionIK/RealxedIK. It is a ROS node, but we use it as a library in `collision_ik_moveit_wrapper`.
- `robotiq`: driver and description material for the gripper
- `universal_robot`: old driver and description material for the UR10e arm
- `Universal_Robots_ROS_Driver`: new driver for the UR10e arm. This is the one we use, but it relies on some things from `universal_robot`.
- `ur_msgs`: a fixed fork of a package required for the UR10e driver

## Time synchronisation
To optimise communication between the computers, we use `chrony` to set the laptop as the preferred NTP server. 

# Misc notes
- This `amiga_main` package was previously called `ur10e_robotiq`; this name may still appear in some places.

## UR10e problems
- The arm uses a lot of power; if it starts misbehaving, make sure the robot charger is plugged in.
- The teach pendant is the tactile screen that can be connected to the UR10e's control box. It can be unabled in the settings.
- The robot's safety and manual mode passwords are the lab's password.
- When the arm bumps into something, it etiher goes into:
  - Protective stop: the arm is locked, and you can't move it. To unlock it, wait 5 seconds and run `make reset-arm`.
  - Fault state: the arm is locked, and you can't move it. To unlock it, you need to either call the `restart_safety` service or acknowledge the fault on the teach pendant, and restart everything.

