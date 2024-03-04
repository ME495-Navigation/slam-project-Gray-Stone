#include "leo_ros_utils/math_helper.hpp"
#include <tf2/tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>

namespace leo_ros_utils {

geometry_msgs::msg::Pose Convert(turtlelib::Transform2D trans2d) {

    geometry_msgs::msg::Pose pose;
    pose.position.x = trans2d.translation().x;
    pose.position.y = trans2d.translation().y;
    pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, trans2d.rotation());
    pose.orientation.w = q.w();
    pose.orientation.x = q.x();
    pose.orientation.z = q.z();
    pose.orientation.y = q.y();

    return pose;
}

turtlelib::Transform2D ConvertBack(geometry_msgs::msg::Pose pose) {
  double roll;
  double yaw;
  double pitch;

  // The fromMsg for quaternion can't be find by linker. 
  tf2::getEulerYPR(tf2::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z,
                                   pose.orientation.w),
                   yaw, pitch, roll);

  return turtlelib::Transform2D{{pose.position.x, pose.position.y}, yaw};
}

  geometry_msgs::msg::Transform
  Convert(geometry_msgs::msg::Pose pose) {

    geometry_msgs::msg::Transform tf;
    tf.translation.x = pose.position.x;
    tf.translation.y = pose.position.y;
    tf.translation.z = pose.position.z;

    tf.rotation = pose.orientation;

    return tf;
  }
}