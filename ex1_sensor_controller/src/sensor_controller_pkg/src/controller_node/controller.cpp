#include <ros/ros.h>
#include "sensor_controller_msgs/SensorMsg.h"

void callback(const sensor_controller_msgs::SensorMsg & msg)
{
    ROS_INFO_STREAM(
        std::endl << "Joint positions received: \n\n" << std::endl 
        << "Joint 1, Type Joint: " << msg.sensors[0].type_joint.c_str() << std::endl
        << "Axis: x=" << msg.sensors[0].axis.x << "   y=" << msg.sensors[0].axis.y << "   z=" << msg.sensors[0].axis.z << std::endl
        << "Angle: " <<  msg.sensors[0].angle << std::endl
        << "Translation: " <<  msg.sensors[0].translation << std::endl
        << "\nJoint 2, Type Joint: " << msg.sensors[1].type_joint.c_str() << std::endl
        << "Axis: x=" << msg.sensors[1].axis.x << "   y=" << msg.sensors[1].axis.y << "   z=" << msg.sensors[1].axis.z << std::endl
        << "Angle: " <<  msg.sensors[1].angle << std::endl
        << "Translation: " <<  msg.sensors[1].translation << std::endl
        << "\nJoint 3, Type Joint: " << msg.sensors[2].type_joint.c_str() << std::endl
        << "Axis: x=" << msg.sensors[2].axis.x << "   y=" << msg.sensors[2].axis.y << "   z=" << msg.sensors[2].axis.z << std::endl
        << "Angle: " <<  msg.sensors[2].angle << std::endl
        << "Translation: " <<  msg.sensors[2].translation << std::endl
        << "\nJoint 4, Type Joint: " << msg.sensors[3].type_joint.c_str() << std::endl
        << "Axis: x=" << msg.sensors[3].axis.x << "   y=" << msg.sensors[3].axis.y << "   z=" << msg.sensors[3].axis.z << std::endl
        << "Angle: " <<  msg.sensors[3].angle << std::endl
        << "Translation: " <<  msg.sensors[3].translation << std::endl
        << "\nJoint 5, Type Joint: " << msg.sensors[4].type_joint.c_str() << std::endl
        << "Axis: x=" << msg.sensors[4].axis.x << "   y=" << msg.sensors[4].axis.y << "   z=" << msg.sensors[4].axis.z << std::endl
        << "Angle: " <<  msg.sensors[4].angle << std::endl
        << "Translation: " <<  msg.sensors[4].translation << std::endl
        << "\nJoint 6, Type Joint: " << msg.sensors[5].type_joint.c_str() << std::endl
        << "Axis: x=" << msg.sensors[5].axis.x << "   y=" << msg.sensors[5].axis.y << "   z=" << msg.sensors[5].axis.z << std::endl
        << "Angle: " <<  msg.sensors[5].angle << std::endl
        << "Translation: " <<  msg.sensors[5].translation << std::endl);
}

int main(int argc, char **argv)
{
  // ROS objects
  ros::init(argc, argv, "controller_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/sensors_topic", 1, callback);

  ros::spin();
  ros::shutdown();
  return 0;

}