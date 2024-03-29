# tf_tutorial
### 必要なincludeファイル
```
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <math.h>
```

### 取得したtfはgeometry_msgs/TransformStamped型になる
```
geometry_msgs::TransformStamped get_tf; //最終的なtf
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener tfListener(tfBuffer);
while (1) {
    try
    {
        get_tf = tfBuffer_.lookupTransform(source_frame, target_frame, ros::Time(0));
        ROS_INFO_ONCE("I got a transform");
        break;
    }  
    catch (tf2::TransformException &e)
    {
        ROS_WARN_STREAM(e.what());
        ros::Duration(0.1).sleep();
        continue;
    }
}

```
### 回転軸の求め方(青色の軸)
```
tf2::Quaternion q_zero(0, 0, 1, 0), q_moto, quat_after;
tf2::convert(get_tf.transform.rotation, q_moto);
q_ato = q_moto * q_zero * q_moto.inverse();
tf2::Vector3 kaiten_jiku(q_ato[0], q_ato[1], q_ato[2]);
```

### 求めた回転軸の周りにpi回転のクオータニオン
```
tf2::Quaternion quat_1;
quat_1.setRotation(kaiten_jiku, M_PI);
```

### 回転軸の周りにpi回転
```
tf2::Quaternion q_final;
q_final = quat_1 * q_moto;
```
### オイラー角を独自に設定して回転
```
tf2::Quaternion q_2, q_euler;
q_euler.setRPY(M_PI/4, 3*M_PI/3, 2*M_PI/3);
q_2 = q_euler * q_moto;
```
### ベクトルの回転
```
double x, y, z; //回転前のベクトルの要素
double x_1, y_1, z_1; //回転後のベクトルの要素
tf2::Quaternion q_moto(x, y, z, 0), quat, quat_after;
tf2::convert(get_tf.transform.rotation, quat);
q_after = quat * q_moto * quat.inverse();
x_1 = q_after[0];
y_1 = q_after[1];
z_1 = q_after[2];
```
### 青軸の方向に10cm手前のtfを出現させる
```
//object_tfが把持対象物のtf、goal_tfがグリッパの目的地のtfとする
geometry_msgs::TransformStamped goal_tf, object_tf;
tf2::Quaternion q_zero(0, 0, 0.01, 0), q_convert, q_pose_convert, q_trans_ato, q_pose_ato;
tf2::convert(object_tf.transform.rotation, q_convert);
q_trans_ato = q_convert * q_zero * q_convert.inverse();
goal_tf.transform.translation.x = q_trans_ato[0] + object_tf.transform.translation.x;
goal_tf.transform.translation.y = q_trans_ato[1] + object_tf.transform.translation.y;
goal_tf.transform.translation.z = q_trans_ato[2] + object_tf.transform.translation.z;

q_pose_convert.setRPY(0, M_PI, 0);
q_pose_ato = q_pose_convert * q_convert;
tf2::convert(q_pose_ato, goal_tf.transform.rotation);
```


