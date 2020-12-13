# Exercise3 FK-IK

1. Implement a service server that computes the direct kinematics of a robot and a service client that uses this service and prints the solution to stdout
2. Verify the result by comparing it with the service /compute_fk of the move_group node
3. Implement an action server that computes all the inverse kinematic solutions of a robot (one by one) and an action client that uses this action and prints the solutions to stdout (one by one, as their are received). The action server does not send the same solution twice and stops when all solutions have been found. At that time, they are returned all together
4. Repeat the experiment at point 3 by neglecting joint limits
5. Visualize the IK solutions in RViz

## Point 1

> Implement a service server that computes the direct kinematics of a robot and a service client that uses this service and prints the solution to stdout

* The service is called [ComputeFK.srv](https://github.com/andrewvali/unisa_robotic/blob/main/ex3_kinematics/src/kinematics_service_msgs/srv/ComputeFK.srv).
* The service server is implemented in the [fk_server_node](https://github.com/andrewvali/unisa_robotic/blob/main/ex3_kinematics/src/kinematics_service/src/fk_server_node/main.cpp)
* The service client is implemented in the [fk_client_node](https://github.com/andrewvali/unisa_robotic/tree/main/ex3_kinematics/src/kinematics_service/src/fk_client_node)

## Point 2

> Verify the result by comparing it with the service /compute_fk of the move_group node

This implementation is at the end of service client. It is available [here](https://github.com/andrewvali/unisa_robotic/tree/main/ex3_kinematics/src/kinematics_service/src/fk_client_node)

## Point 3

> Implement an action server that computes all the inverse kinematic solutions of a robot (one by one) and an action client that uses this action and prints the solutions to stdout (one by one, as their are received). The action server does not send the same solution twice and stops when all solutions have been found. At that time, they are returned all together

## Point 4

> Repeat the experiment at point 3 by neglecting joint limits

## Point 5

> Visualize the IK solutions in RViz

## Usage

### Forward Kinematics

To compute FK run `moveit_config` package and then run this command: 

```bash
    roslaunch kinematics_service fk.launch
```