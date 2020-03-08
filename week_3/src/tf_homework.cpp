#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>
#include <eigen3/Eigen/Dense>

void broadcastTF(const ros::TimerEvent& timer_event)
{
  //
  Eigen::Quaterniond q_A_B(0, 0, std::sqrt(2) /2 , std::sqrt(2) /2);
  Eigen::Vector3d v_A_B(1, 1, 0);

  static tf::TransformBroadcaster br;
  tf::Transform tf_AA;
  tf::Quaternion tf_q(q_A_B.x(),q_A_B.y(),q_A_B.z(),q_A_B.w());
  tf::Vector3 tf_v(v_A_B(0), v_A_B(1), v_A_B(2));
  //std::cout<<"quaternion"<<q_A_B.x()<<"\t"<<q_A_B.y()<<"\t"<<q_A_B.z()<<std::endl;

  tf_AA.setOrigin(tf_v);
  tf_AA.setRotation(tf_q);
  br.sendTransform(tf::StampedTransform(tf_AA
                                        ,ros::Time::now()
                                        ,"A"
                                        ,"B"));

  tf::Transform tf_euler;
  tf::Quaternion euler_q_A_B;
  Eigen::Vector3d euler_A_B = q_A_B.toRotationMatrix().eulerAngles(2, 1, 0);

  euler_q_A_B.setRPY(euler_A_B(2),euler_A_B(1), euler_A_B(0));
  std::cout<<"euler"<<euler_A_B(2)<<"\t"<<euler_A_B(1)<<"\t"<<euler_A_B(0)<<std::endl;
  tf_euler.setOrigin(tf_v);
  tf_euler.setRotation(euler_q_A_B);

  br.sendTransform(tf::StampedTransform(tf_euler
                                        ,ros::Time::now()
                                        ,"A1"
                                        ,"B1"));


  //check quaternion and RPY rotation matrix
  //quaternion
  Eigen::Matrix3d R_quaternion = q_A_B.toRotationMatrix();

  //rotation
  Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(euler_A_B(2),Eigen::Vector3d::UnitX()));
  Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(euler_A_B(1),Eigen::Vector3d::UnitY()));
  Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(euler_A_B(0),Eigen::Vector3d::UnitZ()));

  Eigen::Matrix3d rotation_matrix;
  rotation_matrix=yawAngle*pitchAngle*rollAngle;

  std::cout<<"R_quaternion"<<R_quaternion<<std::endl;
  std::cout<<"R_euler"<<rotation_matrix<<std::endl;

  //RPY change into quaternion
  Eigen::Quaterniond quaternion;
  quaternion=yawAngle*pitchAngle*rollAngle;
  std::cout<<"quaternion"<<quaternion.coeffs()<<std::endl;
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
