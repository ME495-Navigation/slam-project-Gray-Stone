#include "nuturtle_control/ros_math_helper.hpp"




namespace leo_ros_helper {

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


geometry_msgs::msg::Transform Convert( geometry_msgs::msg::Pose pose){

geometry_msgs::msg::Transform tf;
tf.translation.x = pose.position.x;
tf.translation.y = pose.position.y;
tf.translation.z = pose.position.z;

tf.rotation = pose.orientation;

return tf;
}
}