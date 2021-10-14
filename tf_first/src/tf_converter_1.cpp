#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <tf2/convert.h>

tf2_ros::Buffer *buffer;
tf2_ros::TransformListener *listener;

geometry_msgs::TransformStamped get_tf(std::string target, std::string source)
{
    geometry_msgs::TransformStamped final_tf;
    while (1)
    {
        try 
        {
            final_tf = buffer->lookupTransform(source, target, ros::Time(0));
            ROS_INFO_ONCE("get tf");
            break;
        }
        catch (tf2::TransformException &e)
        {
            ROS_WARN_STREAM(e.what());
            ros::Duration(0.1).sleep();
            continue;
        }
    }
    return final_tf;
}

tf2::Quaternion convert_quat(tf2::Quaternion q_ori, tf2::Quaternion q_moto, double angle)
{
    tf2::Quaternion q_after, q_final;
    q_after = q_moto * q_ori * q_moto.inverse();
    tf2::Vector3 vec(q_after[0], q_after[1], q_after[2]);
    q_final.setRotation(vec, angle);
    return q_final;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "iyana1_tf");
    buffer = new tf2_ros::Buffer();
    listener = new tf2_ros::TransformListener(*buffer);
    geometry_msgs::TransformStamped original_tf, out_tf_1, out_tf_2, out_tf_3, out_tf_4, out_tf_5, out_tf_6;
    std::string ori_target, ori_source;
    ros::NodeHandle pnh("~");
    pnh.getParam("ori_target", ori_target);
    pnh.getParam("ori_source", ori_source);
    original_tf = get_tf(ori_target, ori_source);
    
    tf2::Quaternion ori_quat;
    tf2::convert(original_tf.transform.rotation, ori_quat);
    original_tf.header.frame_id = ori_source;
    tf2::Quaternion q_x(1, 0, 0, 0), q_y(0, 1, 0, 0), q_z(0, 0, 1, 0);
    out_tf_1 = original_tf;
    out_tf_2 = original_tf;
    out_tf_3 = original_tf;
    out_tf_4 = original_tf;
    out_tf_5 = original_tf;
    out_tf_6 = original_tf;
    
    tf2::convert(convert_quat(q_x, ori_quat, M_PI / 3) * ori_quat, out_tf_1.transform.rotation);
    out_tf_1.transform.translation.x = out_tf_1.transform.translation.x + 0.3;
    out_tf_1.child_frame_id = "out_1";
    tf2::convert(convert_quat(q_x, ori_quat, M_PI * 2 / 3)* ori_quat, out_tf_2.transform.rotation);
    out_tf_2.transform.translation.x = out_tf_2.transform.translation.x - 1;
    out_tf_2.child_frame_id = "out_2";
    out_tf_3.transform.translation.x = out_tf_3.transform.translation.x + 2;
    out_tf_3.child_frame_id = "out_3";
    out_tf_4.transform.translation.x = out_tf_4.transform.translation.x + 1.5;
    out_tf_4.child_frame_id = "out_4";
    out_tf_5.transform.translation.x = out_tf_5.transform.translation.x + 1;
    out_tf_5.child_frame_id = "out_5";
    out_tf_6.transform.translation.x = out_tf_6.transform.translation.x + 3;
    out_tf_6.child_frame_id = "out_6";

    double roll, pitch, yaw;
    tf2::getEulerYPR(ori_quat, yaw, pitch, roll);
    tf2::Quaternion q_zero(0, 0, 0, 1), q_yaw, q_roll, q_pitch, q_all;
    q_yaw.setRPY(0, 0, yaw);
    q_roll.setRPY(roll, 0, 0);
    q_pitch.setRPY(0, pitch, 0);
    q_all.setRPY(roll, pitch, yaw);
    tf2::convert(q_yaw * q_zero, out_tf_3.transform.rotation);
    tf2::convert(q_pitch * q_yaw * q_zero, out_tf_4.transform.rotation);
    tf2::convert(q_roll * q_pitch * q_yaw * q_zero, out_tf_5.transform.rotation);
    tf2::convert(q_all * q_zero, out_tf_6.transform.rotation);
    tf2::Vector3 vec;
    


   
    tf2::Quaternion q_con, q_naka;
    q_con = convert_quat(q_z, ori_quat, M_PI / 2);
    q_naka = q_con * ori_quat;
    q_con = q_con.inverse() * ori_quat * q_con;
    tf2::convert(q_con, out_tf_1.transform.rotation);
    tf2::convert(q_naka, out_tf_2.transform.rotation);

    ros::Rate loop(10);
    tf2_ros::TransformBroadcaster br_1, br_2, br_3, br_4, br_5, br_6;
    while (ros::ok())
    {
        out_tf_1.header.stamp = ros::Time::now();
        out_tf_2.header.stamp = ros::Time::now();
        out_tf_3.header.stamp = ros::Time::now();
        out_tf_4.header.stamp = ros::Time::now();
        out_tf_5.header.stamp = ros::Time::now();
        out_tf_6.header.stamp = ros::Time::now();
        br_1.sendTransform(out_tf_1);
        br_2.sendTransform(out_tf_2);
        br_3.sendTransform(out_tf_3);
        br_4.sendTransform(out_tf_4);
        br_5.sendTransform(out_tf_5);
        br_6.sendTransform(out_tf_6);
        loop.sleep();
    }
    return 0;
}