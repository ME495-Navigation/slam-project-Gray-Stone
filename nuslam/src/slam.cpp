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

#include <builtin_interfaces/msg/time.hpp>
#include <cstddef>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>

#include <deque>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nuturtlebot_msgs/msg/sensor_data.hpp>
#include <nuturtlebot_msgs/msg/wheel_commands.hpp>
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
#include <tf2/LinearMath/Quaternion.h>
using leo_ros_utils::GetParam;

namespace {

// These are copied from nusim.
const std::string kWorldFrame = "nusim/world";
constexpr static int32_t kFakeSenorStartingID = 50;
constexpr static int32_t kSlamMarkerStartingID = 100;
constexpr static int32_t kMeasureSensorPolarID = 200;
constexpr static int32_t kPredictSensorPolarID = 230;
constexpr static int32_t kActualSensorPolarID = 260;

constexpr static size_t kRobotPathHistorySize = 10; // number of data points

constexpr size_t kMaxLandmarkSize = 3;
constexpr size_t kTotalStateSize = 3 + kMaxLandmarkSize * 2;

constexpr double kProcessNoise = 1e-4;
constexpr double kSensorNoise = 1e-4;

arma::mat GetQMat() {
  arma::mat q_mat = arma::zeros(kTotalStateSize, kTotalStateSize);
  q_mat.at(0, 0) = kProcessNoise;
  q_mat.at(1, 1) = kProcessNoise;
  q_mat.at(2, 2) = kProcessNoise;
  return q_mat;
}

arma::mat GetSigmaZero() {

  arma::mat seg_0 = arma::zeros(kTotalStateSize, kTotalStateSize);
  double large_number = 2e10;
  seg_0.submat(3, 3, kTotalStateSize - 1, kTotalStateSize - 1) =
      arma::eye(kMaxLandmarkSize * 2, kMaxLandmarkSize * 2) * large_number;
  return seg_0;
}

std::ostream &operator<<(std::ostream &os, const std::tuple<double, double> &p) {
  auto [x, y] = p;
  os << "[" << x << y << "]";
  return os;
}

} // namespace

class Slam : public rclcpp::Node {
public:
  Slam()
      : Node("odometry"),
        body_id(GetParam<std::string>(*this, "body_id", "name of the body frame")),
        odom_id(GetParam<std::string>(*this, "odom_id", "name of the body frame")),
        covariance_sigma(GetSigmaZero()), combined_states(arma::zeros(3 + kMaxLandmarkSize * 2, 1)),
        // We do pre sensor update, so R_mat is only 2x2
        R_mat(arma::eye(2, 2) * kSensorNoise), initialized_landmark(kMaxLandmarkSize, false),
        tf_broadcaster(*this) {
    // Uncomment this to turn on debug level and enable debug statements
    // rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    combined_states.at(0) = 0.1;
    combined_states.at(1) = 0.1;
    path_publisher_ = create_publisher<nav_msgs::msg::Path>("green/path", 10);
    sensor_estimate_pub_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("estimate_sensor", 10);
    debug_sensor_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("debug_sensor", 10);

    // Listen to odom

    // odom topic is always odom
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&Slam::OdomCb, this, std::placeholders::_1));

    // Listen to fake sensor

    fake_sensor_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "/fake_sensor", 10, std::bind(&Slam::SensorCb, this, std::placeholders::_1));
  }

  void OdomCb(const nav_msgs::msg::Odometry &msg) { new_odom = msg; }

  void SensorCb(const visualization_msgs::msg::MarkerArray &msg) {

    // Static last_used_odom,
    static turtlelib::Transform2D T_odom_oldrobot;
    // update slam's robot pose guess with the delta in odom (odom is accurate with delta)
    turtlelib::Transform2D T_odom_newrobot = leo_ros_utils::ConvertBack(new_odom.pose.pose);
    // Want T_old_new = T_old_odom * T_odom_new = T_odom_old.inv() * T_odom_new
    turtlelib::Transform2D T_old_new_robot = T_odom_oldrobot.inv() * T_odom_newrobot;

    RCLCPP_ERROR_STREAM(get_logger(), "\n\n=====================   New Cycle\n");
    //***************************************
    // Prediction half of SLAM

    // Predict state
    turtlelib::Transform2D guessed_robot_world = GetCurrentStateTF() * T_old_new_robot;
    SetCurrentStateTF(guessed_robot_world);

    // predict A matrix
    arma::mat a_mat = arma::eye(kTotalStateSize, kTotalStateSize);
    a_mat.at(1, 0) = -T_old_new_robot.translation().y;
    a_mat.at(2, 0) = T_old_new_robot.translation().x;

    covariance_sigma = a_mat * covariance_sigma * a_mat.t() + GetQMat();

    auto current_bot_tf = GetCurrentStateTF();
    RCLCPP_ERROR_STREAM(get_logger(), "predicted bot tf\n " << current_bot_tf);
    RCLCPP_ERROR_STREAM(get_logger(), "a_mat\n " << a_mat);
    RCLCPP_ERROR_STREAM(get_logger(), "covariance sigma\n " << covariance_sigma);

    //***************************************
    // Measurement update half of SLAM
    visualization_msgs::msg::MarkerArray arrow_msgs;

    static std::array<turtlelib::Point2D, kMaxLandmarkSize> landmark_init_loc;
    for (const auto &marker : msg.markers) {
      // Skip not used markers
      if (marker.action == marker.DELETE) {
        continue;
      }
      // Data formatting
      auto [marker_world_p, marker_index] = StripMarker(marker, current_bot_tf);

      RCLCPP_INFO_STREAM(get_logger(), "\n---------------->   Processing Marker "
                                           << marker_index << " At world: " << marker_world_p);

      // Init with trusting the sensor value.
      if (!initialized_landmark[marker_index]) {
        combined_states.at(3 + marker_index * 2) = marker_world_p.x + 1e-2;
        combined_states.at(3 + marker_index * 2 + 1) = marker_world_p.y + 1e-2;
        initialized_landmark[marker_index] = true;
        RCLCPP_INFO_STREAM(get_logger(), "Marker " << marker_index << " Init for the first time");
        landmark_init_loc[marker_index] = marker_world_p;
      }

      // Get predict and measured marker to robot xy
      RCLCPP_ERROR_STREAM(get_logger(), "Combined state\n" << combined_states);
      turtlelib::Point2D predict_landmark_world = GetCurrentLandmarkWold(marker_index);

      auto predict_landmark_polar = World2RelativePolar(predict_landmark_world, current_bot_tf);
      arrow_msgs.markers.push_back(
          MakeArrowMarker(predict_landmark_polar, marker_index, kPredictSensorPolarID));

      auto H_j_mat = GetH_j(marker_index, predict_landmark_world, current_bot_tf);
      arma::mat K_j_mat =
          covariance_sigma * H_j_mat.t() * (H_j_mat * covariance_sigma * H_j_mat.t() + R_mat).i();


      auto measured_landmark_polar = World2RelativePolar(marker_world_p, current_bot_tf);
      arrow_msgs.markers.push_back(
          MakeArrowMarker(measured_landmark_polar, marker_index, kMeasureSensorPolarID));
      arrow_msgs.markers.push_back(MakeArrowMarker(measured_landmark_polar, marker_index,
                                                   kActualSensorPolarID, marker.header.frame_id));
      auto [predict_range, predict_bearing] = predict_landmark_polar;
      auto [measured_range, measured_bearing] = measured_landmark_polar;
      arma::Col<double> err{measured_range - predict_range,
                            turtlelib::normalize_angle(measured_bearing - predict_bearing)};

      arma::Col<double> correction = K_j_mat * (err);

      combined_states = combined_states + correction;
      combined_states.at(0) = turtlelib::normalize_angle(combined_states.at(0));
      covariance_sigma =
          (arma::eye(kTotalStateSize, kTotalStateSize) - K_j_mat * H_j_mat) * covariance_sigma;

      RCLCPP_ERROR_STREAM(get_logger(), "H_j \n" << H_j_mat);
      // RCLCPP_ERROR_STREAM(get_logger(), "H_j_mat * covariance_sigma * H_j_mat.t() + R_mat \n"
      //                                       << H_j_mat * covariance_sigma * H_j_mat.t() + R_mat);
      RCLCPP_ERROR_STREAM(get_logger(), "R_mat \n" << R_mat);
      RCLCPP_ERROR_STREAM(get_logger(),
                          "arma::inv(H_j_mat * covariance_sigma * H_j_mat.t() + R_mat) \n"
                              << (H_j_mat * covariance_sigma * H_j_mat.t() + R_mat).i());

      RCLCPP_ERROR_STREAM(get_logger(), "\e[0;31m K_j \n" << K_j_mat << " \e[0m");
      RCLCPP_ERROR_STREAM(get_logger(), " predict_landmark_polar " << predict_landmark_polar);
      RCLCPP_ERROR_STREAM(get_logger(), " measured_landmark_polar " << measured_landmark_polar);

      RCLCPP_ERROR_STREAM(get_logger(), "err normalized \n" << err);
      RCLCPP_ERROR_STREAM(get_logger(), "K_j * err transpose \n" << (K_j_mat * (err)).t());
      RCLCPP_ERROR_STREAM(get_logger(), "Combined state transposed \n" << combined_states.t());
      RCLCPP_ERROR_STREAM(get_logger(), "covariance sigma\n " << covariance_sigma);
      PublishObsLocation(
          {combined_states.at(3 + marker_index * 2), combined_states.at(3 + marker_index * 2 + 1)},
          marker_index, kWorldFrame);
    }
    PublishPredictTF(current_bot_tf);

    debug_sensor_pub_->publish(arrow_msgs);
    //  End of slam math

    T_odom_oldrobot = T_odom_newrobot;
    PublishWorldOdomRobot(GetCurrentStateTF(), T_odom_newrobot, new_odom.header.stamp);
  }
  // This gives a delta x delta y from robot to landmark
  turtlelib::Point2D GetDeltaXY(turtlelib::Point2D landmark_world_xy, turtlelib::Vector2D bot_xy) {
    auto [landmark_x, landmark_y] = landmark_world_xy;
    return {landmark_x - bot_xy.x, landmark_y - bot_xy.y};
  }

  std::tuple<double, double> World2RelativePolar(turtlelib::Point2D landmark_world_xy,
                                                 turtlelib::Transform2D bot_pose) {
    double dx = landmark_world_xy.x - bot_pose.translation().x;
    double dy = landmark_world_xy.y - bot_pose.translation().y;
    double range = std::sqrt(dx * dx + dy * dy);
    double bearing = turtlelib::normalize_angle(atan2(dy, dx) - bot_pose.rotation());
    return {range, bearing};
  }

  // landmark_index is zero indexed
  //! @brief - Get the big H_j matrix for landmark J, given it's relative location from robot
  //! @param landmark_index - index of the landmark, zero indexed (max n-1 for n landmarks)
  //! @param landmark_robot_xy- relative location of landmark relative to robot
  arma::mat GetH_j(size_t landmark_index, turtlelib::Point2D predict_global_pose , turtlelib::Transform2D bot_pose) {
     
    auto [gx,gy] = predict_global_pose;
    double dx = gx - bot_pose.translation().x;
    double dy = gy - bot_pose.translation().y;

    double d = dx * dx + dy * dy;
    double d_rt = std::sqrt(d);

    RCLCPP_ERROR_STREAM(get_logger(),
                        "dx " << dx << " dy " << dy << " d_sq " << d << " d_rt " << d_rt);

    arma::mat first({{0, -dx / d_rt, -dy / d_rt}, {-1, dy / d, -dx / d}});

    arma::mat second({{dx / d_rt, dy / d_rt}, {-dy / d, dx / d}});

    return arma::join_rows(first, arma::zeros(2, 2 * (landmark_index)), second,
                           arma::zeros(2, 2 * (kMaxLandmarkSize - 1 - landmark_index)));
  }

  // #############################
  // Data type Helpers
  // #############################

  turtlelib::Transform2D GetCurrentStateTF() {
    return {{combined_states.at(1), combined_states.at(2)}, combined_states.at(0)};
  }
  void SetCurrentStateTF(turtlelib::Transform2D bot_pose) {
    combined_states.at(0) = turtlelib::normalize_angle(bot_pose.rotation());
    combined_states.at(1) = bot_pose.translation().x;
    combined_states.at(2) = bot_pose.translation().y;
  }

  turtlelib::Point2D GetCurrentLandmarkWold(size_t landmark_id) {
    return {combined_states.at(3 + landmark_id * 2), combined_states.at(3 + landmark_id * 2 + 1)};
  }

  //! @brief return stripped marker info
  //! @return tuple of : Marker's location in world, and Marker's id.
  std::tuple<turtlelib::Point2D, size_t> StripMarker(visualization_msgs::msg::Marker marker,
                                                     turtlelib::Transform2D bot_pose) {
    return {bot_pose(turtlelib::Point2D{marker.pose.position.x, marker.pose.position.y}),
            marker.id - kFakeSenorStartingID};
  }

  template <typename S> arma::mat Struct2Col(S xy) {
    auto [x, y] = xy;
    return arma::Col<double>{x, y};
  }

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

  void PublishObsLocation(turtlelib::Point2D loc, size_t LandmarkIndex, std::string frame_name) {
    visualization_msgs::msg::MarkerArray msg;
    visualization_msgs::msg::Marker mk;
    mk.header.frame_id = frame_name;
    mk.header.stamp = get_clock()->now();
    mk.id = kSlamMarkerStartingID + LandmarkIndex;

    RCLCPP_ERROR_STREAM(get_logger(), "Publishing loc " << loc);
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

  void PublishPredictTF(turtlelib::Transform2D T_world_robot_predict) {
    geometry_msgs::msg::TransformStamped tf_stamped;
    tf_stamped.header.frame_id = kWorldFrame;
    tf_stamped.child_frame_id = "green/base_predict";
    tf_stamped.header.stamp = get_clock()->now();
    tf_stamped.transform = leo_ros_utils::Convert(leo_ros_utils::Convert(T_world_robot_predict));
    tf_broadcaster.sendTransform(tf_stamped);
  }

  visualization_msgs::msg::Marker MakeArrowMarker(std::tuple<double, double> range_bearing,
                                                  size_t marker_id, size_t starting_id,
                                                  std::string frame_id = "green/base_predict") {

    auto [range, bearing] = range_bearing;
    visualization_msgs::msg::Marker arr;
    arr.type = arr.ARROW;
    arr.header.stamp = get_clock()->now();
    arr.header.frame_id = frame_id;
    arr.id = starting_id + marker_id;
    // Position/Orientation
    // Pivot point is around the tip of its tail. Identity orientation points it along the +X axis.
    // scale.x is the arrow length, scale.y is the arrow width and scale.z is the arrow height.

    tf2::Quaternion target_q;
    // Here is the magic, the description of setEuler is mis-leading!
    target_q.setRPY(0.0, 0.0, bearing);
    // Example of a little angle down, Use this for toggle switch.
    // target_q.setRPY(0.0, angle_down, z_angle);

    arr.pose.orientation.w = target_q.w();
    arr.pose.orientation.x = target_q.x();
    arr.pose.orientation.y = target_q.y();
    arr.pose.orientation.z = target_q.z();
    arr.scale.x = range;

    const double base_thickness = 0.02;
    arr.scale.y = base_thickness;
    arr.scale.z = base_thickness;
    arr.color.a = 0.8;
    arr.color.g = 0.8;
    if (starting_id == kMeasureSensorPolarID) {
      arr.scale.z = base_thickness * 3;
      arr.color.r = 0.8;
    }
    if (starting_id == kPredictSensorPolarID) {
      arr.scale.y = base_thickness * 3;
      arr.color.b = 0.8;
    }
    if (starting_id == kActualSensorPolarID) {
      arr.color.g = 0.0;
      arr.color.r = 1.0;
    }
    return arr;
  }

private:
  std::string body_id;
  std::string odom_id;

  nav_msgs::msg::Odometry new_odom;

  std::deque<geometry_msgs::msg::PoseStamped> bot_path_history =
      std::deque<geometry_msgs::msg::PoseStamped>(kRobotPathHistorySize);

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
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_sensor_pub_;
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