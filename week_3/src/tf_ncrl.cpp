/*
 *  test the tf_class
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "tf_conversions/tf_eigen.h"
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cmath>
#include "ncrl_lio/ncrl_tf.h"
#include <string>

enum Flag{
  LIDAR_INIT,
  LIDAR,
  IMU_INIT,
  IMU,
  INERTIA,
  WORLD
};

FrameID FrameFlag[6] = {"LIDAR_INIT", "LIDAR", "IMU_INIT", "IMU", "INERTIA", "WORLD"};

void transpoint_test(int count){
  static tf::TransformBroadcaster br;
  auto now = ros::Time::now();

  ncrl_tf::Point p;
  ncrl_tf::setPointFrame(p, FrameFlag[Flag::LIDAR]);

  p.point << 2 * sin(count * 0.1 * M_PI),
             2 * cos(count * 0.1 * M_PI),
             1;
  ncrl_tf::Trans tf, tf2;
//    ncrl_tf::setTransFrame(tf, FrameFlag[Flag::IMU], FrameFlag[Flag::LIDAR]);
  ncrl_tf::setTransFrame(tf, FrameFlag[Flag::IMU], FrameFlag[Flag::LIDAR]);
  tf.v << 1, 2, 0;

  Eigen::Vector3d euler(double(count) * 0.5 * M_PI, 0.0, double(count) * 0.25 * M_PI);
//    Eigen::Vector3d euler(0,0,0);
  tf.q = ncrl_tf::Euler2Q(euler);

  tf::Transform point_L, lidar2imu;

  Eigen::Quaterniond qIdenetity(1, 0, 0, 0);
  ncrl_tf::setTfTrans(point_L, qIdenetity, p.point);
  ncrl_tf::setTfTrans(lidar2imu, tf.q, tf.v);
  br.sendTransform(tf::StampedTransform(point_L, now, p.frame, "Point"));
  br.sendTransform(tf::StampedTransform(lidar2imu, now, tf.start_frame, tf.end_frame));
  if(ncrl_tf::TransPoint(tf, p)){
    tf::Transform point_I;
    ncrl_tf::setTfTrans(point_I, qIdenetity, p.point);
    br.sendTransform(tf::StampedTransform(point_I, now, p.frame, "Point2"));
  }
}

void trans_odometry_test(int count){
  static tf::TransformBroadcaster br;
  auto now = ros::Time::now();
  ncrl_tf::Trans tlidar, timu, tlidarimu, tlidarimuInit;

  ncrl_tf::setTransFrame(tlidar, FrameFlag[Flag::LIDAR_INIT], FrameFlag[Flag::LIDAR]);
  ncrl_tf::setTransFrame(timu, FrameFlag[Flag::IMU_INIT], FrameFlag[Flag::IMU]);
  ncrl_tf::setTransFrame(tlidarimu, FrameFlag[Flag::IMU], FrameFlag[Flag::LIDAR]);
  ncrl_tf::setTransFrame(tlidarimuInit, FrameFlag[Flag::IMU_INIT], FrameFlag[Flag::LIDAR_INIT]);

  Eigen::Vector3d temp(0, 0, count * 0.25 * M_PI);
  Eigen::Vector3d temp2(0, 0, M_PI);
  ncrl_tf::setTrans(tlidar,
                      ncrl_tf::Euler2Q(temp),
                      Eigen::Vector3d(2 * sin(count * 0.1 * M_PI), 2 * cos(count * 0.1 * M_PI), 1));
  ncrl_tf::setTrans(tlidarimu,
                      ncrl_tf::Euler2Q(temp2),
                      Eigen::Vector3d(1, 2, -1));
  tlidarimuInit.q = tlidarimu.q;
  tlidarimuInit.v = tlidarimu.v;

  if(ncrl_tf::TransOdometry(tlidarimu, tlidar, timu)){
    tf::Transform LInit2L, IInit2I, L2I, LI2II;
    ncrl_tf::setTfTrans(LInit2L, tlidar.q, tlidar.v);
    ncrl_tf::setTfTrans(IInit2I, timu.q, timu.v);
    ncrl_tf::setTfTrans(L2I, tlidarimu.q, tlidarimu.v);
    ncrl_tf::setTfTrans(LI2II, tlidarimuInit.q, tlidarimuInit.v);
    br.sendTransform(tf::StampedTransform(LInit2L, now, tlidar.start_frame, tlidar.end_frame));
    br.sendTransform(tf::StampedTransform(IInit2I, now, timu.start_frame, timu.end_frame));
    br.sendTransform(tf::StampedTransform(L2I, now, tlidarimu.start_frame, tlidarimu.end_frame));
    br.sendTransform(tf::StampedTransform(LI2II, now, tlidarimuInit.start_frame, tlidarimuInit.end_frame));
  }
}

void delta_trans_test(int count){
  static tf::TransformBroadcaster br;
  auto now = ros::Time::now();

  ncrl_tf::Trans t_i, t_j, t_ij;
  ncrl_tf::setTransFrame(t_i, FrameFlag[Flag::LIDAR], "I");
  ncrl_tf::setTransFrame(t_j, FrameFlag[Flag::LIDAR], "J");
  Eigen::Vector3d temp(0, 0, count * 0.25 * M_PI);
  Eigen::Vector3d temp2(0, 0, M_PI);
  ncrl_tf::setTrans(t_i,
                      ncrl_tf::Euler2Q(temp),
                      Eigen::Vector3d(2 * sin(count * 0.1 * M_PI), 2 * cos(count * 0.1 * M_PI), 1));
  ncrl_tf::setTrans(t_j,
                      ncrl_tf::Euler2Q(temp2),
                      Eigen::Vector3d(1, 2, -1));
  if (ncrl_tf::deltaTrans(t_ij, t_i, t_j)){
    tf::Transform tij, ti, tj;
    ncrl_tf::setTfTrans(tij, t_ij.q, t_ij.v);
    ncrl_tf::setTfTrans(ti, t_i.q, t_i.v);
    ncrl_tf::setTfTrans(tj, t_j.q, t_j.v);
    br.sendTransform(tf::StampedTransform(tij, now, t_ij.start_frame, "test"));
    br.sendTransform(tf::StampedTransform(ti, now, t_i.start_frame, t_i.end_frame));
    br.sendTransform(tf::StampedTransform(tj, now, t_j.start_frame, t_j.end_frame));
  }
}

void accum_trans_test(int count){
  static tf::TransformBroadcaster br;
  auto now = ros::Time::now();

  int state_num = count % 4 + 3;
  state_num = 6;
  ncrl_tf::Trans T[state_num];
  ncrl_tf::Trans T_test[state_num - 1];

  // setting frame
  for (int i = 0; i < state_num; i++){
    if (i == 0)
      ncrl_tf::setTransFrame(T[i], "init", std::to_string(i));
    else
      ncrl_tf::setTransFrame(T[i], std::to_string(i-1), std::to_string(i));
  }

  // setting transform
  Eigen::Vector3d euler(0,0, double(360/state_num) * M_PI / 180.0);
  Eigen::Vector3d trans(1, 0, 0);
  for (int i = 0; i < state_num; i++){
    ncrl_tf::setTrans(T[i], ncrl_tf::Euler2Q(euler), trans);
  }

  bool check = true;
  for (int i = 0; i < state_num - 1; i++){
    if (i == 0){
      if (!ncrl_tf::accumTrans(T_test[i], T[i], T[i+1]))
        check = false;
    } else {
      if (!ncrl_tf::accumTrans(T_test[i], T_test[i-1], T[i+1]))
        check = false;
    }
  }

  // setting test frame
  for (int i = 0; i < state_num - 1; i++){
    T_test[i].end_frame = "test" + T_test[i].end_frame;
  }

  if (check){
    tf::Transform TF[state_num];
    tf::Transform TF_test[state_num - 1];
    for (int i = 0; i < state_num; i++){
      ncrl_tf::setTfTrans(TF[i], T[i].q, T[i].v);
      br.sendTransform(tf::StampedTransform(TF[i], now, T[i].start_frame, T[i].end_frame));
    }

    for (int i = 0; i < state_num - 1; i++){
      ncrl_tf::setTfTrans(TF_test[i], T_test[i].q, T_test[i].v);
      br.sendTransform(tf::StampedTransform(TF_test[i], now, T_test[i].start_frame, T_test[i].end_frame));
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_class_test");
  ros::NodeHandle nh;

  ROS_INFO("start tf class test");

  int count = 0;

  ros::Rate r(1);
  while(ros::ok()){
//    transpoint_test(count);
//    trans_odometry_test(count);
//    delta_trans_test(count);
    accum_trans_test(count);
    count++;
    ros::spinOnce();
    r.sleep();
  }
}
