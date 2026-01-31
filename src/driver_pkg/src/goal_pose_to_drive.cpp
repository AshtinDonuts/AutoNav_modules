#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/int16.hpp>

#include "rover_messages/msg/drive.hpp"

#include <cmath>
#include <algorithm>

using std::placeholders::_1;

class GoalPoseToDrive : public rclcpp::Node {
public:
  GoalPoseToDrive() : Node("goal_pose_to_drive") {
    // Parameters
    wheelbase_ = this->declare_parameter("wheelbase", 1.0);
    steer_angle_limit_ = this->declare_parameter("steer_limit", 0.6);
    goal_tolerance_ = this->declare_parameter("goal_tolerance", 0.15);

    // Subscribers
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 10,
        std::bind(&GoalPoseToDrive::goalCallback, this, _1));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/zed/zed_node/pose", 10,
        std::bind(&GoalPoseToDrive::poseCallback, this, _1));

    // Publishers
    drive_pub_ = this->create_publisher<rover_messages::msg::Drive>(
        "core/drive/controller_input", 10);

    drive_mode_pub_ = this->create_publisher<std_msgs::msg::Int16>(
        "base_station/drive/drive_mode", 10);

    // Control loop
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&GoalPoseToDrive::controlLoop, this));

    RCLCPP_INFO(this->get_logger(), "goal_pose_to_drive started");
  }

private:
  // ---------------- Callbacks ----------------

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goal_x_ = msg->pose.position.x;
    goal_y_ = msg->pose.position.y;
    goal_active_ = true;

    RCLCPP_INFO(this->get_logger(),
                "New goal: (%.2f, %.2f)", goal_x_, goal_y_);
  }

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    pose_x_ = msg->pose.position.x;
    pose_y_ = msg->pose.position.y;

    const auto &q = msg->pose.orientation;
    double siny = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    pose_yaw_ = std::atan2(siny, cosy);

    pose_received_ = true;
  }

  // ---------------- Control ----------------

  void controlLoop() {
    if (!goal_active_ || !pose_received_)
      return;

    rover_messages::msg::Drive drive;
    std_msgs::msg::Int16 mode;

    double dx = goal_x_ - pose_x_;
    double dy = goal_y_ - pose_y_;
    double distance = std::sqrt(dx * dx + dy * dy);
    double target_yaw = std::atan2(dy, dx);
    double yaw_error = normalizeAngle(target_yaw - pose_yaw_);

    // -------- GOAL REACHED --------
    if (distance < goal_tolerance_) {
      drive.velocity = 0.0;
      drive.steer_angle = 0.0;
      drive.npt_ang_vel = 0.0;

      drive_pub_->publish(drive);
      goal_active_ = false;

      RCLCPP_INFO(this->get_logger(), "Goal reached");
      return;
    }

    // -------- TURN IN PLACE (NPT) --------
    if (std::abs(yaw_error) > 0.4 && distance < 0.5) {
      mode.data = 3;

      drive.velocity = 0.0;
      drive.steer_angle = 0.0;
      drive.npt_ang_vel = 1.5 * yaw_error;
    }
    // -------- ACKERMANN --------
    else {
      mode.data = 2;

      double v = std::clamp(0.8 * distance, 0.0, 1.0);
      double w = 1.2 * yaw_error;

      drive.velocity = v;
      drive.npt_ang_vel = 0.0;

      double steer = 0.0;
      if (std::abs(v) > 0.05) {
        steer = std::atan(wheelbase_ * w / v);
      }

      drive.steer_angle =
          std::clamp(steer, -steer_angle_limit_, steer_angle_limit_);
    }

    drive_mode_pub_->publish(mode);
    drive_pub_->publish(drive);
  }

  double normalizeAngle(double a) {
    while (a > M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  // ---------------- ROS ----------------
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<rover_messages::msg::Drive>::SharedPtr drive_pub_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr drive_mode_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---------------- State ----------------
  bool goal_active_{false};
  bool pose_received_{false};

  double goal_x_{0.0}, goal_y_{0.0};
  double pose_x_{0.0}, pose_y_{0.0}, pose_yaw_{0.0};

  // ---------------- Params ----------------
  double wheelbase_;
  double steer_angle_limit_;
  double goal_tolerance_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalPoseToDrive>());
  rclcpp::shutdown();
  return 0;
}
