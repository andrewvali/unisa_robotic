#include <ros/ros.h>
#include <sensor_controller_msgs/SensorMsg.h>
#include <string>
#include <iostream>
#include <math.h>

int main(int argc, char** argv){
    ros::init(argc,argv,"sensor_controller_node");
    ros::NodeHandle nh;
    ros::Publisher publisher = nh.advertise<sensor_controller_msgs::SensorMsg>("/sensors_topic",1);
    ros::Rate loop_rate(0.8);

    sensor_controller_msgs::SensorMsg joint_position;
    joint_position.sensors.resize(6);

    while(ros::ok){
        for(int i=0;i<6;i++){
            joint_position.sensors[i].axis.x = (float) rand()/RAND_MAX;
            joint_position.sensors[i].axis.y = (float) rand()/RAND_MAX;
            joint_position.sensors[i].axis.z = (float) rand()/RAND_MAX;
            if(i==0 || i==3 || i==5){
                joint_position.sensors[i].position = ((float) rand()/RAND_MAX)*(2*M_PI);
                joint_position.sensors[i].type_joint = "Revolute";
            }else{
                joint_position.sensors[i].position = (float) rand()/RAND_MAX;
                joint_position.sensors[i].type_joint = "Prismatic";
            }
        }

        publisher.publish(joint_position);
        ros::spinOnce();
        loop_rate.sleep();
    }
    ros::shutdown();
    return 0;
    
}