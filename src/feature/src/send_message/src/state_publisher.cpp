#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <string>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 100);
    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(5);
    
    const double degree = M_PI / 180;

    // robot state
    double angle = 0;
    // message declarations
    sensor_msgs::JointState joint_state;
    std::vector<std::string> joints_names = {"head_yaw", "head_pitch", "left_shoulder_pitch", "left_shoulder_roll", "left_shoulder_twirl", "left_elbow_pitch", "left_hand_twirl", "left_finger_pitch", "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_twirl", "right_elbow_pitch", "right_hand_twirl", "right_finger_pitch", "pelvis_pitch", "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle_pitch", "left_ankle_roll", "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle_pitch", "right_ankle_roll"};
    joint_state.name.resize(joints_names.size());
    joint_state.position.resize(joints_names.size());
    for (int i = 0; i < joint_state.name.size(); ++i)
    {
        joint_state.name[i] = joints_names[i];
        joint_state.position[i] = 0; // absolute zero OR origin zero from xml??
    }
    while (ros::ok())
    {

        // update joint_state
        joint_state.header.stamp = ros::Time::now();

        for (int i = 0; i < 2; ++i)
        {
            joint_state.position[i] += 10 * degree; // absolute zero OR origin zero from xml??

        } // send the joint state and transform
        joint_pub.publish(joint_state);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
