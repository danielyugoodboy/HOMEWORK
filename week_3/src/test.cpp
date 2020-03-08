#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>
#include <eigen3/Eigen/Dense>

void broadcastTF(const ros::TimerEvent& timer_event)
{
  static tf::TransformBroadcaster br;
  tf::Transform tf_A_B, tf_B_C;
  tf::Quaternion q_A_B,  q_B_C;
  tf::Vector3     v_A_B, v_B_C;
  v_A_B.setValue(1.0, 0.0, 0.0);
  q_A_B.setRPY(0, 0, M_PI);

  v_A_C.setValue(0.0, 0, 0.5);
  q_A_C.setRPY(0, 0, M_PI);

  tf_A_B.setOrigin(v_A_B);
  tf_A_B.setRotation(q_A_B);
  tf_A_C.setOrigin(v_B_C);
  tf_A_C.setRotation(q_B_C);

  br.sendTransform(tf::StampedTransform(tf_A_B, // transform
                                        ros::Time::now(), // timestamp with this transform
                                        "A", // paranet frame ID
                                        "B")); // child frame ID

  br.sendTransform(tf::StampedTransform(tf_B_C,
                                        ros::Time::now(),
                                        "A",
                                        "C"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ncrl_tf_learning");
  ros::NodeHandle nh;
  // Create timer with 2.0 Hz
  ros::Timer timer = nh.createTimer(ros::Duration(0.5), broadcastTF);
  while (ros::ok()){ros::spinOnce();}
  return 0;
}
