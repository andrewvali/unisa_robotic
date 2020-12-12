# Execise 1 (Publisher-Subscriber)

Implement a publisher that simulates a set of sensors for encoder readings (position of 6 joints) and a subscriber that simulates a set of controllers that prints such readings to stdout.

## Packages

The project consists in two packages:

* [sensor_controller_pkg](https://github.com/andrewvali/unisa_robotic/tree/main/ex1_sensor_controller/src/sensor_controller_pkg) contains:
  * The 2 files representing the publisher and subscriber implementation, in the [src](https://github.com/andrewvali/unisa_robotic/tree/main/ex1_sensor_controller/src/sensor_controller_pkg/src) subfolder.
  * A [.launch file](https://github.com/andrewvali/unisa_robotic/tree/main/ex1_sensor_controller/src/sensor_controller_pkg/launch) using for the execution of 2 nodes.
* [sensor_controller_msgs](https://github.com/andrewvali/unisa_robotic/tree/main/ex1_sensor_controller/src/sensor_controller_msgs) contains the definition of [SensorMsg](https://github.com/andrewvali/unisa_robotic/blob/main/ex1_sensor_controller/src/sensor_controller_msgs/msg/SensorMsg.msg) and [JointPosition](https://github.com/andrewvali/unisa_robotic/blob/main/ex1_sensor_controller/src/sensor_controller_msgs/msg/JointPosition.msg) message types

#### Encoders Node

The [encoders node](https://github.com/andrewvali/unisa_robotic/blob/main/ex1_sensor_controller/src/sensor_controller_pkg/src/encoders_node/encoders.cpp) simulates a set of 6 sensors for encoder readings. It sends periodically information about the position of 6 joints on a specific topic.

#### Controller Node

The [controller node](https://github.com/andrewvali/unisa_robotic/blob/main/ex1_sensor_controller/src/sensor_controller_pkg/src/controller_node/controller.cpp) simulates a set of 6 controllers for the joints. It reads the information about the position of 6 joints and prints these informations.

### Messages

The informations about the position of 6 joints are containded in [SensorMsg](https://github.com/andrewvali/unisa_robotic/blob/main/ex1_sensor_controller/src/sensor_controller_msgs/msg/SensorMsg.msg) message type.

#### SensorMsg

The [SensorMsg](https://github.com/andrewvali/unisa_robotic/blob/main/ex1_sensor_controller/src/sensor_controller_msgs/msg/SensorMsg.msg) message type contains an array of [JointPosition](https://github.com/andrewvali/unisa_robotic/blob/main/ex1_sensor_controller/src/sensor_controller_msgs/msg/JointPosition.msg).

#### JointPosition

The [JointPosition](https://github.com/andrewvali/unisa_robotic/blob/main/ex1_sensor_controller/src/sensor_controller_msgs/msg/JointPosition.msg) message type represents a sensor for a joint. It contains 3 fields:

* type_joint: a string that describes the type of the joint:
  * Revolute
  * Prismatic.
* axis: a [Point](http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/Point.html) representing the direction of the joint axis.
* angle: for revolute joints, in range ]0, 2pi[.
* translation: for prismatic joints, in range ]0, 1[.

## Note
* For `Revolute` joint there is only rotation, so the default value of translation is 0.
* For `Prismatic` joint there is only translation, so the default value of rotation is 0.

## Usage

* To run the whole program, run this command: `roslaunch sensor_controller_pkg sensor_controller.launch`.
* To run only the publisher (encoders), run this command: `rosrun sensor_controller_pkg encoders_node`.
* To run only the subscriber (controller) run this command: `rosrun sensor_controller_pkg controller_node`.
