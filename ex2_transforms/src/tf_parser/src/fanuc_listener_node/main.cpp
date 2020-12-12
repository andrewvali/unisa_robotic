#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <std_msgs/String.h>

int main(int argc,char **argv){

    std::string target_frame;
    std::string source_frame = "flange";
    

    tf2::Quaternion quaternion; // Transform rotation
    geometry_msgs::Vector3 translation; // Transform translation
    tf2::Vector3 axis; // Axis representation

    tf2Scalar angle;        // Angle representation
    tf2Scalar roll,pitch,yaw;  // Euler Angles

    // Declare a Transform in which the transformations are stored
    geometry_msgs::TransformStamped transform;

    //init Node
    ros::init(argc,argv,"fanuc_listener");
    ros::NodeHandle nh;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    ros::Rate rate(1.0);
    while(nh.ok()){
        try{
            for(int i=0; i<6; i++){
                
                std::stringstream current_link;
                if(i == 0){
                    current_link << "base_link";
                }else{
                    // Aosign current link
                    current_link << "link" << i;
                }
                target_frame = current_link.str();
                
                transform = tfBuffer.lookupTransform(target_frame,source_frame,ros::Time(0));
                // Take the translation from TransformStamped
                translation = transform.transform.translation;
                // Take the rotation and convert in a quaternion
                tf2::fromMsg(transform.transform.rotation , quaternion);
                // Take Angle from quaternion
                angle = quaternion.getAngle();
                // Take Axis from quaternion
                axis = quaternion.getAxis();
                // Take rotation matrix from quaternion
                tf2::Matrix3x3 rotation_matrix(quaternion);
                // Take Euler angles from quaternion
                rotation_matrix.getRPY(roll,pitch,yaw);
                
                std::ostringstream os;

                // Print link information            
                os << std::endl << std::endl << "************ Transformation from " << transform.header.frame_id << " to " << transform.child_frame_id << " ************" << std::endl;

                os << std::endl << "------- Translation -------" << std::endl;
                os << translation << std::endl;

                os << "------- Quaternion -------" << std::endl;
                os << "[" << quaternion.getX() << ", " << quaternion.getY() << ", " << quaternion.getZ() << ", " << quaternion.getW() << "]" << std::endl << std::endl;
                
                os << "------- Axis/angle -------" << std::endl;
                os << "Axis = [" << axis.getX() << ", " << axis.getY() << ", " << axis.getZ() << "]" << std::endl;
                os << "Angle = " << angle << std::endl;


                os << std::endl << "------- Rotation matrix -------" << std::endl;
                for (int i=0; i<3; i++)
                    os << "[" << rotation_matrix[i][0] << ", " << rotation_matrix[i][1] << ", " << rotation_matrix[i][2] << "]" << std::endl;


                os << std::endl << "------- Euler angles (RPY) -------" << std::endl;
                os << "[ " << roll << ", " << pitch << ", " << yaw << " ]" << std::endl;

                ROS_INFO_STREAM(os.str());
        }

                
    
        }catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        rate.sleep();
    }
        
    return 0;
}