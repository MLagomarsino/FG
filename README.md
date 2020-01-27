# Football Game
## Experimental Robotics Laboratory - Lab. 2

The goal of this second lab is to implement an algorithm to make a robot able to detect a ball and kick it inside the football goal.
The ball is of a given color and it is recognized through a monocular camera (calibrated) mounted on the robot. 
The files for the camera calibration have been added in the apposite folders of the Raspberry.
The calibration has been done for two resolutions: 640x480 and 320x200.
The dimensions of the football field are given, and the position of the football goal is fixed.
The position of the robot wrt the world is known, thanks to markers placed in the environment.
Thanks to these informations it possible to compute a desired position and orientation for the robot to properly being able to 
kick the ball inside the football goal.
The robot is omnidirectional.


## Authors
| Name | E-mail | ID |
|------|--------|--------|
| Luna Gava| lunagava@me.com | 4206721 |
| Lucrezia Grassi | lucre.grassi@gmail.com | 4223595 |
| Marta Lagomarsino | marta.lago@hotmail.it | 4213518 |

## How to run the project
To simulate the behaviour of the robot, launch the simulation on Gazebo writing the following command:
```
roslaunch football_game gazebo.launch <arguments>
```
where arguments should be the coordinates x and y of the starting intial position of the robot;
  
For example, a possible command is:
```
   roslaunch football_game gazebo.launch r_x:=0 r_y:=0
```
In order to publish the position of the ball, open a new terminal and type:
```
	rostopic pub -r 1 geometry_msgs/Point '5.0' '0.0' '6.0'
```
Once Gazebo is open, press play to start the simulation: the robot will reach a goal position and orientation
which allows it to kick the ball inside the football goal.

To run the code on the real robot:
Open a terminal and connect via ssh to the Raspberry, and launch the raspicam node by typing:
```
roslaunch raspicam_node camerav2_320x200_30fps.launch enable_raw:=true
```
In another terminal, go inside the project folder and run:
```
python ball_tracking.py
```
In another terminal, go inside the project folder and run:
```
python compute_vel.py
```

In a terminal on your computer, launch the roscore by typing:
```
roscore &
```
On the same terminal, publish the velocities of the robot:
```
rostopic pub -r 1 geometry_msgs/Twist -- '[1.0, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
```



## Doxygen documentation 
To compile the code documentation, it is required to run the following command in the workspace:
```
doxygen Lab2_documentation
``` 
Enter the newly generated folder */doxygen_documentation/html* and open *index.html*.
