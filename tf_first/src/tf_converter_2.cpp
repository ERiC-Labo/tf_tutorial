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

tf2::Vector3 get_vec(tf2::Quaternion quat_pose, tf2::Quaternion q_moto_jiku)
{
    tf2::Quaternion q_after;
    q_after = quat_pose * q_moto_jiku * quat_pose.inverse();
    tf2::Vector3 vec(q_after[0], q_after[1], q_after[2]);
    return vec;
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
    
    out_tf_1.transform.translation.x = out_tf_1.transform.translation.x + 0.3;
    out_tf_1.child_frame_id = "out_1";
    
    tf2::Quaternion q_ken, q_af;
    tf2::Vector3 vec;
    q_ken = convert_quat(q_z, ori_quat, 0.001) * ori_quat;
    int count = 0;
    do  {
        q_ken = convert_quat(q_z, ori_quat, 0.002) * q_ken;
        vec = get_vec(q_ken, q_x);
        // ROS_INFO_STREAM("*******");
        // ROS_INFO_STREAM("x: " << vec.x() << "   y: " << vec.y() << "   z: " << vec.z());
        count++;
    } while (!(vec.y() <= 0.004 && vec.y() >= -0.004 && vec.x() > 0));
    
    

    std::cout << "*****************" << std::endl;
    std::cout << count <<std::endl;
    
    
    tf2::convert(q_ken, out_tf_1.transform.rotation);
   
    ros::Rate loop(10);
    tf2_ros::TransformBroadcaster br_1, br_2, br_3, br_4, br_5, br_6;
    while (ros::ok())
    {
        out_tf_1.header.stamp = ros::Time::now();
        br_1.sendTransform(out_tf_1);
        // br_2.sendTransform(out_tf_2);
        // br_3.sendTransform(out_tf_3);
        // br_4.sendTransform(out_tf_4);
        // br_5.sendTransform(out_tf_5);
        // br_6.sendTransform(out_tf_6);
        loop.sleep();
    }
    return 0;
}