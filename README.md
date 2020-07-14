# Quadrotor Work
## Description
During the summer months of 2020, I was supposed to work in Dr. Laurent Burlion's lab to research the advanced control of drones and test algorithms to see if an automonomous quadrotor can fly faster through a course than a manned drone. Originally, the plan was to build the drone according to the University of Zurich's Robotics and Perception Group's suggested framework which they had tested and released in their research papers (cited below). Due to COVID-19, the project was modified to accomodate remote work. Instead of building the entire drone, I was tasked with setting up the ROS/Gazebo environment to run simulations and start putting together some hardware parts that are safe for home use. The ROS/Gazebo framework that was implemented can be found at https://github.com/uzh-rpg/rpg_quadrotor_control. The code I wrote and the hardware I worked on will be documented in this repository. 

M. Faessler, A. Franchi, and D. Scaramuzza, "Differential Flatness of Quadrotor Dynamics Subject to Rotor Drag for Accurate Tracking of High-Speed Trajectories," IEEE Robot. Autom. Lett. (RA-L), vol. 3, no. 2, pp. 620–626, Apr. 2018. 

M. Faessler, D. Falanga, and D. Scaramuzza, "Thrust Mixing, Saturation, and Body-Rate Control for Accurate Aggressive Quadrotor Flight," IEEE Robot. Autom. Lett. (RA-L), vol. 2, no. 2, pp. 476–482, Apr. 2017.

## Code
### QuadrotorSim.m
This file contains a set of algorithms and equations transcribed from the Robotics and Perception Group's theory papers. The algorithms calculate the drone's movement and describe it using either euler angles, a quaternion, and a rotation matrix. The algorithms can convert between the three and also model the quadrotor drone.
## Hardware
The main computer used for the quadrotor is an Odroid Xu4 show below.
![Odroid](/Images/IMG_4231.HEIC)

The Odroid was booted with an eMMC running Ubuntu Mate 18.04. On it, ROS Melodic and Gazebo9 were installed to be able to run simulations and handle the code for when the drone is built. 

The flight controller used for the drone was a Lumenier F4 AIO which was soldered to an XT60 wall adapter as a subsitute for 3s LiPo battery for home use.
![Flight Controller](/Images/IMG_2046.HEIC)
![Soldering](/Images/IMG_7479.HEIC)
