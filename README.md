# Assignment 1 Research Track 1 
This repository contains two ROS nodes that interact with the turtlesim environment:
- UI Node: Allows the user to control a turtle's movement using velocity inputs.
- Distance Node: Monitors the distance between the turtles and stops their motion if they are too close to each other or the boundaries.

# Requirements
Before using the package, make sure you have ROS (any compatible version) installed and then the Turtlesim package, which comes preinstalled with ROS. 
You must also have a working catkin workspace to compile and run the package.
If you do not already have a workspace, create a new one and link it to the source with the following commands:
```
mkdir -p ~/ros_ws/src
gedit ~/.bashrc
```
and add this line source ```/root/ros_ws/devel/setup.bash``` to the bashrc file, where /root/ros_ws is the path of the folder. Then, close and reopen the terminal to make the changes effective.

# Package installation
1. Go inside the src folder ```cd ~/ros_ws/src```
2. Clone the repository into your workspace: ```git clone https://github.com/chiarabuono/assignment1_rt.git ```
3. Build the workspace in the main folder:
```
cd ~/ros_ws
catkin_make
```
# How to Run the package
1. Run Turtlesim Node with ```roslaunch turtlesim turtlesim_node ```
2. In a new terminal, run UI Node with ```rosrun assignment1_rt UI_node```
3. In a new terminal, run Distance Node with ```rosrun assignment1_rt distance_node```

# Node Descriptions

## UI Node
This node allows the user to select a turtle and move it with a velocity of their choice. At first the user is asked to select between:
- Turtle 1 (digit 1);
- Turtle 2 (digit 2);
- Exit the node (digit q).

Any other value will prompt the user to reselect between the previous choises. Then, the user can choose the velocity of the turtle:

- Linear velocity along the x-axis.
- Linear velocity along the y-axis.
- Angular velocity around the z-axis.

Once given all the inputs, the turtle will move with these velocities for one second and then stop. Only now the possibility to repeat the procedure is given again. 

## Distance Node

The distance node monitors the positions of both turtles and takes action if the turtles are too close to each other or to boundaries within a predefined threshold distance.
In such a case, their movement is stopped to avoid collision.

Due to the reaction time of the node, the turtles will stop and stop moving once the chosen threshold is exceeded. To avoid this problem, the node checks the previous position of the turtles and allows their movement if they move away from the boundary or the other turtle.