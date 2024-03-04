//! @file odometry calculation node
//! @brief Generate odometry base on wheel encoder change
// Parameters:

// Publishers:
//  tf : world to green odom to blue robot.

// Subscriber:
//  odom - nav_msgs::msg::Odometry : calculated odometry value

// Service Server:
//  initial_pose - nuturtle_control::srv::InitPose : Set the initial pose of the
//  robot when called.


#include <cstddef>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include <deque>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
#include <deque>
#include <rclcpp/exceptions/exceptions.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/detail/joint_state__traits.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <tf2_ros/transform_broadcaster.h>
#include <turtlelib/diff_drive.hpp>
#include <turtlelib/geometry2d.hpp>
#include <turtlelib/se2d.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <leo_ros_utils/math_helper.hpp>
#include <leo_ros_utils/param_helper.hpp>

#include <armadillo>

using leo_ros_utils::GetParam;

namespace  {

// These are copied from nusim.
const std::string kWorldFrame = "nusim/world";
constexpr static int32_t kFakeSenorStartingID = 150;
constexpr static int32_t kSlamMarkerStartingID = 300;
constexpr static size_t kRobotPathHistorySize = 10 ; // number of data points

constexpr size_t kMaxLandmarkSize = 3;
constexpr size_t kTotalStateSize = 3 + kMaxLandmarkSize * 2;

constexpr double kProcessNoise = 0.0000000000001;

arma::mat GetQMat() {
  arma::mat q_mat = arma::zeros(kTotalStateSize,kTotalStateSize);
  q_mat.at(0, 0) = kProcessNoise;
  q_mat.at(1, 1) = kProcessNoise;
  q_mat.at(2, 2) = kProcessNoise;
  return q_mat;
}

arma::mat GetSigmaZero() {

  arma::mat seg_0 = arma::zeros(kTotalStateSize,kTotalStateSize);
  // double large_number = std::numeric_limits<double>::max() / 10;
  double large_number = 2e10;
  seg_0.submat(3, 3, kTotalStateSize -1 , kTotalStateSize -1) =
      arma::eye(kMaxLandmarkSize * 2, kMaxLandmarkSize * 2) * large_number;
  return seg_0;
}
}

class Slam : public rclcpp::Node {
public:
  Slam()
      : Node("odometry"),
        body_id(GetParam<std::string>(*this, "body_id", "name of the body frame")),
        odom_id(GetParam<std::string>(*this, "odom_id", "name of the body frame")),
        covariance_sigma(GetSigmaZero()),
        combined_states(arma::zeros(3+kMaxLandmarkSize*2,1)),
        // We do pre sensor update, so R_mat is only 2x2 
        R_mat(arma::eye(2,2) * 0.000000001),
        initialized_landmark(kMaxLandmarkSize , false),
        tf_broadcaster(*this)
        {
    // Uncomment this to turn on debug level and enable debug statements
    // rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

    path_publisher_ = create_publisher<nav_msgs::msg::Path>("green/path", 10);
    sensor_estimate_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("estimate_sensor", 10);

    // Listen to odom

    // odom topic is always odom
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&Slam::OdomCb, this, std::placeholders::_1));

    // Listen to fake sensor

    fake_sensor_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "/fake_sensor", 10, std::bind(&Slam::SensorCb, this, std::placeholders::_1));
  }

  void OdomCb(const nav_msgs::msg::Odometry &msg) { new_odom = msg; }

  void SensorCb(const visualization_msgs::msg::MarkerArray& msg ){

    // Static last_used_odom,
    static turtlelib::Transform2D T_odom_oldrobot;
    // update slam's robot pose guess with the delta in odom (odom is accurate with delta)
    turtlelib::Transform2D  T_odom_newrobot = leo_ros_utils::ConvertBack(new_odom.pose.pose);
    // Want T_old_new = T_old_odom * T_odom_new = T_odom_old.inv() * T_odom_new
    turtlelib::Transform2D T_old_new_robot = T_odom_oldrobot.inv() * T_odom_newrobot;

    static size_t iter = 0;
    iter ++;

    // if (iter == 10){
    //   exit(0);
    // }

    RCLCPP_ERROR_STREAM(get_logger() , "\n\n=====================   New Cycle\n");
    //***************************************
    // Prediction half of SLAM

    // Predict state
     turtlelib::Transform2D guessed_robot_world =
         GetCurrentStateTF(combined_states) * T_old_new_robot;
     combined_states.at(0) = guessed_robot_world.translation().x;
     combined_states.at(1) = guessed_robot_world.translation().y;
     combined_states.at(2) = guessed_robot_world.rotation();

     auto current_bot_tf = GetCurrentStateTF(combined_states);
     // predict A matrix
     arma::mat a_mat = arma::eye(kTotalStateSize, kTotalStateSize);
     a_mat.at(1, 0) = -T_old_new_robot.translation().y;
     a_mat.at(2, 0) = T_old_new_robot.translation().x;

     covariance_sigma = a_mat * covariance_sigma * a_mat.t() + GetQMat();
    RCLCPP_ERROR_STREAM(get_logger(),"predicted bot tf\n "<<current_bot_tf);
    RCLCPP_ERROR_STREAM(get_logger(),"a_mat\n "<<a_mat);
    RCLCPP_ERROR_STREAM(get_logger(),"covariance sigma\n "<<covariance_sigma);

     //***************************************
     // Measurement update half of SLAM

    static std::array<turtlelib::Point2D,kMaxLandmarkSize> landmark_init_loc;
     for (const auto &marker : msg.markers) {
       // Note: This XY still needs a transform, it's in robot's frame,
       // But the math we use needs it in global frame's delta xy.
       // They are a rotation off.

      // Data formatting
      auto [marker_world_p, marker_index] = StripMarker(marker, current_bot_tf);

      RCLCPP_INFO_STREAM(get_logger(),
                         "\n---------------->   Processing Marker " << marker_index << " At world: " << marker_world_p);

      // Init with trusting the sensor value.
      if (! initialized_landmark[marker_index]){
        combined_states.at(3+marker_index*2) = marker_world_p.x;
        combined_states.at(3+marker_index*2+1) = marker_world_p.y;
        initialized_landmark[marker_index] = true;
        RCLCPP_INFO_STREAM(get_logger(), "Marker " << marker_index << " Init for the first time");
        landmark_init_loc[marker_index] = marker_world_p;
      }


      // Get predict and measured marker to robot xy
      RCLCPP_ERROR_STREAM(get_logger(), "Combined state\n" << combined_states);
      auto predict_landmark_delta =
          GetDeltaXY(turtlelib::Point2D{combined_states.at(3 + marker_index * 2),
                                        combined_states.at(3 + marker_index * 2 + 1)},
                     current_bot_tf.translation());
      RCLCPP_ERROR_STREAM(get_logger(), "Predicted Marker delta " << predict_landmark_delta);
      auto H_j_mat = GetH_j(marker_index, predict_landmark_delta);
      RCLCPP_ERROR_STREAM(get_logger(), "H_j \n" << H_j_mat);



      RCLCPP_ERROR_STREAM(get_logger(), "H_j_mat * covariance_sigma * H_j_mat.t() + R_mat \n" << H_j_mat * covariance_sigma * H_j_mat.t() + R_mat);
      RCLCPP_ERROR_STREAM(get_logger(), "arma::inv(H_j_mat * covariance_sigma * H_j_mat.t() + R_mat) \n" << (H_j_mat * covariance_sigma * H_j_mat.t() + R_mat).i());
      arma::mat K_j_mat = covariance_sigma * H_j_mat.t() *
                          (H_j_mat * covariance_sigma * H_j_mat.t() + R_mat).i();

      RCLCPP_ERROR_STREAM(get_logger(), "K_j \n" << K_j_mat);

      auto measured_landmark_delta = GetDeltaXY(marker_world_p, current_bot_tf.translation());

      RCLCPP_ERROR_STREAM(get_logger(), "measured_landmark_delta " << measured_landmark_delta);

      RCLCPP_ERROR_STREAM(get_logger(), "K_j * err \n" << K_j_mat * ((Point2Col(measured_landmark_delta)) -
                                                     (Point2Col(predict_landmark_delta))));
                                                     
      combined_states = combined_states + K_j_mat * ((Point2Col(measured_landmark_delta)) -
                                                     (Point2Col(predict_landmark_delta)));
      RCLCPP_ERROR_STREAM(get_logger(), "Combined state\n" << combined_states);

      covariance_sigma =
          (arma::eye(kTotalStateSize, kTotalStateSize) - K_j_mat * H_j_mat) * covariance_sigma;
      RCLCPP_ERROR_STREAM(get_logger(), "covariance sigma\n " << covariance_sigma);
      // PublishObsLocation(landmark_init_loc[marker_index], marker_index);
      // PublishObsLocation(marker_world_p, marker_index);
      PublishObsLocation({combined_states.at(3 + marker_index * 2),
                          combined_states.at(3 + marker_index * 2 + 1)},
                         marker_index, kWorldFrame);
     }

    //  End of slam math




    T_odom_oldrobot = T_odom_newrobot;

    turtlelib::Transform2D new_robot_world{{combined_states.at(0), combined_states.at(1)},
                                           combined_states.at(2)};

    PublishWorldOdomRobot(new_robot_world, T_odom_newrobot, new_odom.header.stamp);
    
    // static bool init_obs = false;
    // if (init_obs){
    // init_obs= true;

    // }

  }


  // void Predict(turtlelib::Transform2D T_delta){
    
  // }


  


  // This gives a delta x delta y from robot to landmark
  turtlelib::Point2D GetDeltaXY(turtlelib::Point2D landmark_world_xy , turtlelib::Vector2D bot_xy){
    auto[landmark_x, landmark_y] = landmark_world_xy;
    return {landmark_x - bot_xy.x, landmark_y - bot_xy.y};
  }

  // landmark_index is zero indexed 
  //! @brief - Get the big H_j matrix for landmark J, given it's relative location from robot 
  //! @param landmark_index - index of the landmark, zero indexed (max n-1 for n landmarks)
  //! @param landmark_robot_xy- relative location of landmark relative to robot
  arma::mat GetH_j(size_t landmark_index ,turtlelib::Point2D landmark_robot_xy ){
    auto [dx,dy] = landmark_robot_xy;
    double d = dx*dx +dy*dy;
    double d_rt = std::sqrt(d);
    arma::mat first({
      {0 , -dx/d_rt , -dy/d_rt},
      {-1, dy/d , -dx/d}
    });

    arma::mat second({
      {dx/d_rt , dy/d_rt},
      {-dy/d , dx/d}
    });

    return arma::join_rows(first, arma::zeros(2, 2 * (landmark_index)), second,
                           arma::zeros(2, 2 * (kMaxLandmarkSize - 1 - landmark_index)));
  }

  // #############################
  // Data type Helpers 
  // #############################

  turtlelib::Transform2D GetCurrentStateTF(arma::mat current_state){
    return {{current_state.at(0), current_state.at(1)}, current_state.at(2)};
  }

  //! @brief return stripped marker info
  //! @return tuple of : Marker's location in world, and Marker's id. 
  std::tuple<turtlelib::Point2D , size_t> StripMarker( visualization_msgs::msg::Marker marker , turtlelib::Transform2D bot_pose){
    return {bot_pose(turtlelib::Point2D{marker.pose.position.x, marker.pose.position.y}) , marker.id - kFakeSenorStartingID };
  }

  arma::mat Point2Col(turtlelib::Point2D xy) { return arma::Col<double>{xy.x, xy.y}; }

  void PublishWorldOdomRobot(turtlelib::Transform2D T_world_robot,
                             turtlelib::Transform2D T_odom_robot,
                             builtin_interfaces::msg::Time stamp) {
    auto T_world_odom = T_world_robot * (T_odom_robot.inv());

    geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped.header.frame_id = odom_id;
    tf_stamped.child_frame_id = body_id;
    tf_stamped.header.stamp = stamp;
    tf_stamped.transform = leo_ros_utils::Convert(leo_ros_utils::Convert(T_odom_robot));
    tf_broadcaster.sendTransform(tf_stamped);

    tf_stamped.header.frame_id = kWorldFrame;
    tf_stamped.child_frame_id = odom_id;
    tf_stamped.transform = leo_ros_utils::Convert(leo_ros_utils::Convert(T_world_odom));
    tf_broadcaster.sendTransform(tf_stamped);

    geometry_msgs::msg::PoseStamped new_pose;
    new_pose.header.frame_id = kWorldFrame;
    new_pose.header.stamp = stamp;
    new_pose.pose.position.x = T_world_robot.translation().x;
    new_pose.pose.position.y = T_world_robot.translation().y;
    bot_path_history.push_back(new_pose);
    if (bot_path_history.size() >= kRobotPathHistorySize) {
      bot_path_history.pop_front();
    }

    nav_msgs::msg::Path path_msg;
    path_msg.header = new_pose.header;

    path_msg.poses = std::vector<geometry_msgs::msg::PoseStamped>{bot_path_history.begin(),
                                                                  bot_path_history.end()};

    path_publisher_->publish(path_msg);
  }

  void PublishObsLocation(turtlelib::Point2D loc , size_t LandmarkIndex, std::string frame_name) {
    visualization_msgs::msg::MarkerArray msg;
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = frame_name;
    mk.header.stamp = get_clock()->now();
    mk.id = kSlamMarkerStartingID + LandmarkIndex;

    RCLCPP_ERROR_STREAM(get_logger(), "Publishing loc "<< loc );
    mk.pose.position.x = loc.x;
    mk.pose.position.y = loc.y;
    mk.scale.z = 0.8;
    mk.pose.position.z = 0.8 / 2;
    mk.color.g = 1.0;
    mk.color.a = 0.8;

    mk.type = mk.CYLINDER;
    mk.scale.x = 0.035 * 2;
    mk.scale.y = 0.035 * 2;
    msg.markers.push_back(mk);
    sensor_estimate_pub_->publish(msg);
  }

private:
  std::string body_id;
  std::string odom_id;

  nav_msgs::msg::Odometry new_odom;

  std::deque<geometry_msgs::msg::PoseStamped> bot_path_history = std::deque<geometry_msgs::msg::PoseStamped>(kRobotPathHistorySize);

  // turtlelib::Transform2D guessed_robot_world;

  // combined covariance segma_t
  arma::mat covariance_sigma;
  arma::mat combined_states;
  arma::mat R_mat;
  std::vector<bool> initialized_landmark;
  // ROS IDL stuff
  tf2_ros::TransformBroadcaster tf_broadcaster;
  std::pair<double, turtlelib::Transform2D> last_stamped_tf2d;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr sensor_estimate_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr fake_sensor_sub_;

  

};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  //   rclcpp::Node::SharedPtr node_ptr =
  //   std::make_shared<rclcpp::Node>("turtle_control") ; TurtleControl
  //   t_ctrl{node_ptr};
  rclcpp::spin(std::make_shared<Slam>());
  rclcpp::shutdown();
  return 0;
}