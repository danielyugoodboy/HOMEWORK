#include <ros/ros.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ncrl_tf_listener");
  ros::NodeHandle nh;

  tf::TransformListener listener1;
  tf::TransformListener listener2;
  tf::TransformListener listener3;
  tf::StampedTransform trans_A1_A2;
  tf::StampedTransform trans_A1_A3;
  tf::StampedTransform trans_A1_A4;

  try{
    listener1.waitForTransform("A1", "A2", ros::Time(0), ros::Duration(3.0));
    listener1.lookupTransform("A1", // source_frame
                             "A2", // target_frame
                             ros::Time(0),
                             trans_A1_A2);
    listener2.waitForTransform("A1", "A3", ros::Time(0), ros::Duration(3.0));
    listener2.lookupTransform("A1", // source_frame
                             "A3", // target_frame
                             ros::Time(0),
                             trans_A1_A3);
    listener3.waitForTransform("A1", "A4", ros::Time(0), ros::Duration(3.0));
    listener3.lookupTransform("A1", // source_frame
                             "A4", // target_frame
                             ros::Time(0),
                             trans_A1_A4);
    }
  catch (tf::TransformException ex){
    //ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
  // Convert quaternion to RPY
  tf::Quaternion q1(trans_A1_A2.getRotation().x(),
                   trans_A1_A2.getRotation().y(),
                   trans_A1_A2.getRotation().z(),
                   trans_A1_A2.getRotation().w());
  tf::Matrix3x3 mat1(q1);
  double roll, pitch, yaw;
  mat1.getRPY(roll, pitch, yaw);
  printf("A1 -> A2 rotation : %f \t %f \t %f \n", roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI);

  tf::Transform tf_A2_A3 = trans_A1_A2.inverse() * trans_A1_A3;
  double x, y, z;
  x = tf_A2_A3.getOrigin().getX();
  y = tf_A2_A3.getOrigin().getY();
  z = tf_A2_A3.getOrigin().getZ();
  printf("A2 -> A3 translation : %f \t %f \t %f \n",x, y, z);

  tf::Quaternion q2(tf_A2_A3.getRotation().x(),
                   tf_A2_A3.getRotation().y(),
                   tf_A2_A3.getRotation().z(),
                   tf_A2_A3.getRotation().w());
  tf::Matrix3x3 mat2(q2);
  mat2.getRPY(roll, pitch, yaw);
  printf("A2 -> A3 rotation : %f \t %f \t %f \n", roll*180/M_PI, pitch*180/M_PI, yaw*180/M_PI);
  return 0;
}
