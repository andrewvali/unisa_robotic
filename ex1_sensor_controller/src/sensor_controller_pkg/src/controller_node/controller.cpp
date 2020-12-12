#include <ros/ros.h>
#include "sensor_controller_msgs/SensorMsg.h"

void callback(const sensor_controller_msgs::SensorMsg & msg)
{
    ROS_INFO_STREAM(
        std::endl << "Joint positions received: \n\n" << std::endl 
        << "Joint 1, Type Joint: " << msg.sensors[0].type_joint.c_str() << std::endl
        << "Position is: " <<  msg.sensors[0].position << std::endl
        << "with respect to Axis: [" << msg.sensors[0].axis.x << ", " << msg.sensors[0].axis.y << ", " << msg.sensors[0].axis.z << std::endl
        << "\nJoint 2, Type Joint: " << msg.sensors[1].type_joint.c_str() << std::endl
        << "Position is: " <<  msg.sensors[1].position << std::endl
        << "with respect to Axis: [" << msg.sensors[1].axis.x << ", " << msg.sensors[1].axis.y << ", " << msg.sensors[1].axis.z << std::endl
        << "\nJoint 3, Type Joint: " << msg.sensors[2].type_joint.c_str() << std::endl
        << "Position is: " <<  msg.sensors[2].position << std::endl
        << "with respect to Axis: [" << msg.sensors[2].axis.x << ", " << msg.sensors[2].axis.y << ", " << msg.sensors[2].axis.z << std::endl
        << "\nJoint 4, Type Joint: " << msg.sensors[3].type_joint.c_str() << std::endl
        << "Position is: " <<  msg.sensors[3].position << std::endl
        << "with respect to Axis: [" << msg.sensors[3].axis.x << ", " << msg.sensors[3].axis.y << ", " << msg.sensors[3].axis.z << std::endl
        << "\nJoint 5, Type Joint: " << msg.sensors[4].type_joint.c_str() << std::endl
        << "Position is: " <<  msg.sensors[4].position << std::endl
        << "with respect to Axis: [" << msg.sensors[4].axis.x << ", " << msg.sensors[4].axis.y << ", " << msg.sensors[4].axis.z << std::endl
        << "\nJoint 6, Type Joint: " << msg.sensors[5].type_joint.c_str() << std::endl
        << "Position is: " <<  msg.sensors[5].position << std::endl
        << "with respect to Axis: [" << msg.sensors[5].axis.x << ", " << msg.sensors[5].axis.y << ", " << msg.sensors[5].axis.z << std::endl);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/sensors_topic", 1, callback);

  ros::spin();
  ros::shutdown();
  return 0;

}