# Exercise 2 (FANUC-URDF)

1. Derive DH parameters for the Fanuc robot.
2. Translate DH parameters to URDF and visualize the Fanuc robot in RViz with urdf_tutorial display.launch.
3. Create a fanuc_moveit_config package and visualize the robot in RViz.
4. Visualize TFs in RViz and write one or more ROS nodes to compute and print the TF of the end-effector in all the reference frames of all joints. From the TF, compute the translational vector, the rotation matrix, the Euler angles and the axis-angle representation.
5. Verify the results by comparing with manual/MATLAB calculations.

## Point 1

> Derive DH parameters for the Fanuc robot.

* DH parameters are available in this [pdf](https://github.com/andrewvali/unisa_robotic/blob/main/ex2_transforms/src/fanuc_description/doc/fanuc_dh.pdf).

## Point 2

> Translate DH parameters to URDF and visualize the Fanuc robot in RViz with urdf_tutorial display.launch.

The URDF file is [here](https://github.com/andrewvali/unisa_robotic/blob/main/ex2_transforms/src/fanuc_description/robot/fanuc_m20ia.xacro).

## Point 3

> Create a fanuc_moveit_config package and visualize the robot in RViz.

The fanuc_moveit_config package is [here](https://github.com/andrewvali/unisa_robotic/tree/main/ex2_transforms/src/fanuc_moveit_config).

## Point 4

> Visualize TFs in RViz and write one or more ROS nodes to compute and print the TF of the end-effector in all the reference frames of all joints. From the TF, compute the translational vector, the rotation matrix, the Euler angles and the axis-angle representation.

The ros node implemented to solve this point is in tf_parser package, in particular, in the [fanuc_listener_node](https://github.com/andrewvali/unisa_robotic/tree/main/ex2_transforms/src/tf_parser/src/fanuc_listener_node) subfolder.  

## Point 5

> Verify the results by comparing with manual/MATLAB calculations.

Work in progress.

## Usage

To visualize the robot model, run this command: 
```bash
roslaunch fanuc_description fanuc.launch
``` 

To calculate TFs, run this command: 
```bash
roslaunch tf_parser fanuc_listener.launch
```