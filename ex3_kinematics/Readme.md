# Exercise3 FK-IK

1. Implement a service server that computes the direct kinematics of a robot and a service client that uses this service and prints the solution to stdout
2. Verify the result by comparing it with the service /compute_fk of the move_group node
3. Implement an action server that computes all the inverse kinematic solutions of a robot (one by one) and an action client that uses this action and prints the solutions to stdout (one by one, as their are received). The action server does not send the same solution twice and stops when all solutions have been found. At that time, they are returned all together
4. Repeat the experiment at point 3 by neglecting joint limits
5. Visualize the IK solutions in RViz
