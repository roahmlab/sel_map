

// #include "lowlevelapi.h"
#include "ros/ros.h"
// #include "sensor_msgs/Imu.h"
#include "tf2_msgs/TFMessage.h"
#include "tf2_ros/transform_listener.h"

#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/Quaternion.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Odometry.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <iostream>
#include <functional>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
using namespace Eigen;
using namespace std;


// sensor_msgs::Imu msg;
// tf2_msgs::TFMessage tf_msg;
geometry_msgs::TransformStamped tf_msg;
geometry_msgs::PoseStamped cam_pose_msg;
nav_msgs::Odometry odom_imu;
geometry_msgs::TransformStamped base_to_map_tf_msg;


tf2_ros::Buffer tfBuffer;




void callback(const nav_msgs::Odometry::ConstPtr& msg, ros::Publisher& cam_pose_pub)
{

  // const std::string frame_to = "camera_link";
  // const std::string frame_from = "camera_color_optical_frame";

  // Eigen::Transform<double, 3, Eigen::Isometry>& T;

  // Eigen::Vector3d trans_robot_cam (0.127, 0.0175, -0.1878);
  // Eigen::Quaterniond rot_robot_cam (0.9624552,0.0, 0.2714404, 0.0 );

  // for azure
  Eigen::Vector3d trans_robot_cam (0.1278, 0.0632, -0.3232);
  Eigen::Quaterniond rot_robot_cam (0.4971991,-0.3710001, 0.7441105, 0.2478942 );

  // cout << "w"<< rot_robot_cam.w()<< "y"<<rot_robot_cam.y() << endl;
  
  try
  {
    // tf_msg = tfBuffer.lookupTransform(frame_from, frame_to,ros::Time(0));
    // Eigen::Quaterniond rot (tf_msg.transform.rotation.w, tf_msg.transform.rotation.x,
    //               tf_msg.transform.rotation.y, tf_msg.transform.rotation.z);

    // Eigen::Vector3d trans (tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.transform.translation.z);
  cam_pose_msg.header = msg->header;
  cam_pose_msg.header.frame_id = "odom2";
  cam_pose_msg.pose = msg->pose.pose;

  cam_pose_msg.pose.position.x += trans_robot_cam[0];
  cam_pose_msg.pose.position.y += trans_robot_cam[1];
  cam_pose_msg.pose.position.z += trans_robot_cam[2];

  Eigen::Quaterniond rot_robot (cam_pose_msg.pose.orientation.w,cam_pose_msg.pose.orientation.x,cam_pose_msg.pose.orientation.y,cam_pose_msg.pose.orientation.z);

  Eigen::Quaterniond cam_orient = rot_robot*rot_robot_cam;
  
  cam_pose_msg.pose.orientation.x = cam_orient.x();
  cam_pose_msg.pose.orientation.y = cam_orient.y();
  cam_pose_msg.pose.orientation.z = cam_orient.z();
  cam_pose_msg.pose.orientation.w = cam_orient.w();

  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    // ros::Duration(1.0).sleep();
    return;
  }



  // ROS_INFO_STREAM("orient"<< cam_orient);


  cam_pose_pub.publish(cam_pose_msg);


}

int main(int argc, char* argv[])
{

    // init ros
  ros::init(argc, argv, "odom_adapt");
  ros::NodeHandle n;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Publisher cam_pose_pub = n.advertise<geometry_msgs::PoseStamped>("cam_pose", 1);


  auto odom_cb = std::bind(callback, std::placeholders::_1, std::ref(cam_pose_pub));
  ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("/odometry/imu", 1, odom_cb);
  ros::spin();
  return 0;
}
