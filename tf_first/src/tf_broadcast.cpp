#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tsuchida_init");
    tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped static_tf;
    ros::NodeHandle pnh("~");
    double x, y, z, roll, pitch, yaw;
    std::string child_frame, parent_frame;
    pnh.getParam("x", x);
    pnh.getParam("y", y);
    pnh.getParam("z", z);
    pnh.getParam("roll", roll);
    pnh.getParam("pitch", pitch);
    pnh.getParam("yaw", yaw);
    pnh.getParam("child_frame", child_frame);
    pnh.getParam("parent_frame", parent_frame);
    static_tf.header.frame_id = parent_frame;
    static_tf.header.stamp = ros::Time::now();
    static_tf.child_frame_id = child_frame;
    static_tf.transform.translation.x = x;
    static_tf.transform.translation.y = y;
    static_tf.transform.translation.z = z;
    tf2::Quaternion quat;
    quat.setRPY(roll, pitch, yaw);
    // tf2::toMsg(quat, static_tf.transform.rotation);
    tf2::convert(quat, static_tf.transform.rotation);
    // static_tf.transform.rotation.x = quat[0];
    // static_tf.transform.rotation.y = quat[1];
    // static_tf.transform.rotation.z = quat[2];
    // static_tf.transform.rotation.w = quat[3];
    ros::Rate loop(10);
    while (ros::ok()) {
        br.sendTransform(static_tf);
        loop.sleep();
    }
    // br.sendTransform(static_tf);
    
    return 0;
}