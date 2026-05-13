#include <chrono>
#include <memory>
#include <vector>
#include <random>
#include <algorithm>
#include <cmath>
#include <limits>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>

#include <Eigen/Dense>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace std::chrono_literals;

class AquabotController : public rclcpp::Node {
public:
  AquabotController() : Node("aquabot_controller") {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/aquabot/odom",
      10,
      std::bind(&AquabotController::odom_callback, this, std::placeholders::_1)
    );

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/aquabot/goal_pose",
      10,
      std::bind(&AquabotController::goal_callback, this, std::placeholders::_1)
    );

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/aquabot/reference_path",
      10,
      std::bind(&AquabotController::path_callback, this, std::placeholders::_1)
    );

    pub_fl_ = this->create_publisher<std_msgs::msg::Float64>(
      "/aquabot/thrusters/left/thrust", 10
    );

    pub_fr_ = this->create_publisher<std_msgs::msg::Float64>(
      "/aquabot/thrusters/right/thrust", 10
    );

    pub_al_ = this->create_publisher<std_msgs::msg::Float64>(
      "/aquabot/thrusters/left/cmd_pos", 10
    );

    pub_ar_ = this->create_publisher<std_msgs::msg::Float64>(
      "/aquabot/thrusters/right/cmd_pos", 10
    );

    current_state_ = Eigen::VectorXd::Zero(6);
    goal_pose_ = Eigen::Vector3d::Zero();
    U_ = Eigen::MatrixXd::Zero(4, horizon_);
    last_cmd_ = Eigen::Vector4d::Zero();

    timer_ = this->create_wall_timer(
      100ms,
      std::bind(&AquabotController::control_loop, this)
    );

    RCLCPP_INFO(this->get_logger(), "Aquabot MPPI controller started.");
  }

private:
  static constexpr double PI = 3.14159265358979323846;

  double wrap_angle(double angle) const {
    return std::atan2(std::sin(angle), std::cos(angle));
  }

  double yaw_from_quaternion(const geometry_msgs::msg::Quaternion& q_msg) const {
    tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
  }

  void publish_stop() {
    std_msgs::msg::Float64 zero_msg;
    zero_msg.data = 0.0;

    pub_fl_->publish(zero_msg);
    pub_fr_->publish(zero_msg);
    pub_al_->publish(zero_msg);
    pub_ar_->publish(zero_msg);

    last_cmd_.setZero();
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    has_odom_ = true;

    current_state_(0) = msg->pose.pose.position.x;
    current_state_(1) = msg->pose.pose.position.y;
    current_state_(2) = yaw_from_quaternion(msg->pose.pose.orientation);

    current_state_(3) = msg->twist.twist.linear.x;
    current_state_(4) = msg->twist.twist.linear.y;
    current_state_(5) = msg->twist.twist.angular.z;
  }

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    goal_pose_(0) = msg->pose.position.x;
    goal_pose_(1) = msg->pose.position.y;
    goal_pose_(2) = yaw_from_quaternion(msg->pose.orientation);
    goal_start_pose_(0) = current_state_(0);
    goal_start_pose_(1) = current_state_(1);
    goal_start_pose_(2) = current_state_(2);

    goal_received_ = true;

    U_.setZero();
    last_cmd_.setZero();

    RCLCPP_INFO(
      this->get_logger(),
      "New RViz Goal: x = %.2f, y = %.2f, yaw = %.2f rad",
      goal_pose_(0),
      goal_pose_(1),
      goal_pose_(2)
    );
  }

  std::vector<Eigen::Vector3d> resample_path(
    const std::vector<Eigen::Vector2d>& raw_points,
    double ds
  ) const {
    std::vector<Eigen::Vector3d> resampled;

    if (raw_points.size() < 2) {
      return resampled;
    }

    for (size_t i = 0; i < raw_points.size() - 1; i++) {
      Eigen::Vector2d p0 = raw_points[i];
      Eigen::Vector2d p1 = raw_points[i + 1];

      Eigen::Vector2d segment = p1 - p0;
      double length = segment.norm();

      if (length < 1e-6) {
        continue;
      }

      Eigen::Vector2d direction = segment / length;
      double yaw = std::atan2(direction(1), direction(0));

      if (resampled.empty()) {
        resampled.push_back(Eigen::Vector3d(p0(0), p0(1), yaw));
      }

      double travelled = ds;

      while (travelled < length) {
        Eigen::Vector2d p = p0 + travelled * direction;
        resampled.push_back(Eigen::Vector3d(p(0), p(1), yaw));
        travelled += ds;
      }
    }

    Eigen::Vector2d last = raw_points.back();
    Eigen::Vector2d before_last = raw_points[raw_points.size() - 2];

    double final_yaw = std::atan2(
      last(1) - before_last(1),
      last(0) - before_last(0)
    );

    resampled.push_back(Eigen::Vector3d(last(0), last(1), final_yaw));

    return resampled;
  }

  void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    reference_path_.clear();

    if (msg->poses.size() < 2) {
      has_path_ = false;
      RCLCPP_WARN(this->get_logger(), "Received path is too short.");
      return;
    }

    std::vector<Eigen::Vector2d> raw_points;
    raw_points.reserve(msg->poses.size());

    for (const auto& pose_stamped : msg->poses) {
      raw_points.push_back(Eigen::Vector2d(
        pose_stamped.pose.position.x,
        pose_stamped.pose.position.y
      ));
    }

    reference_path_ = resample_path(raw_points, path_ds_);

    if (reference_path_.empty()) {
      has_path_ = false;
      RCLCPP_WARN(this->get_logger(), "Path resampling failed.");
      return;
    }

    has_path_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "Received path with %zu raw poses. Resampled to %zu poses.",
      msg->poses.size(),
      reference_path_.size()
    );
  }

  void clamp_control(Eigen::Vector4d& u) const {
    u(0) = std::clamp(u(0), -1.0, 1.0);
    u(1) = std::clamp(u(1), -1.0, 1.0);

    u(2) = std::clamp(u(2), -max_angle_, max_angle_);
    u(3) = std::clamp(u(3), -max_angle_, max_angle_);
  }

  Eigen::Vector4d rate_limit_control(
    const Eigen::Vector4d& desired,
    const Eigen::Vector4d& previous
  ) const {
    Eigen::Vector4d limited = desired;

    limited(0) = previous(0) + std::clamp(
      desired(0) - previous(0),
      -max_thrust_rate_normalized_,
      max_thrust_rate_normalized_
    );

    limited(1) = previous(1) + std::clamp(
      desired(1) - previous(1),
      -max_thrust_rate_normalized_,
      max_thrust_rate_normalized_
    );

    limited(2) = previous(2) + std::clamp(
      desired(2) - previous(2),
      -max_angle_rate_,
      max_angle_rate_
    );

    limited(3) = previous(3) + std::clamp(
      desired(3) - previous(3),
      -max_angle_rate_,
      max_angle_rate_
    );

    clamp_control(limited);

    return limited;
  }

  Eigen::VectorXd predict_state(
    const Eigen::VectorXd& state,
    const Eigen::Vector4d& control
  ) const {
    double x = state(0);
    double y = state(1);
    double yaw = state(2);
    double u = state(3);
    double v = state(4);
    double r = state(5);

    double F_L = control(0) * max_thrust_;
    double F_R = control(1) * max_thrust_;
    double alpha_L = control(2);
    double alpha_R = control(3);

    double Fx_L = F_L * std::cos(alpha_L);
    double Fy_L = F_L * std::sin(alpha_L);

    double Fx_R = F_R * std::cos(alpha_R);
    double Fy_R = F_R * std::sin(alpha_R);

    double x_L = -L_ / 2.0;
    double y_L =  W_ / 2.0;

    double x_R = -L_ / 2.0;
    double y_R = -W_ / 2.0;

    double tau_X = Fx_L + Fx_R;
    double tau_Y = Fy_L + Fy_R;

    double tau_N = x_L * Fy_L - y_L * Fx_L
                 + x_R * Fy_R - y_R * Fx_R;

    double u_dot = (tau_X - d_u_ * u) / m_u_;
    double v_dot = (tau_Y - d_v_ * v) / m_v_;
    double r_dot = (tau_N - d_r_ * r) / m_r_;

    double x_dot = u * std::cos(yaw) - v * std::sin(yaw);
    double y_dot = u * std::sin(yaw) + v * std::cos(yaw);
    double yaw_dot = r;

    Eigen::VectorXd next_state(6);

    next_state(0) = x + x_dot * dt_;
    next_state(1) = y + y_dot * dt_;
    next_state(2) = wrap_angle(yaw + yaw_dot * dt_);

    next_state(3) = u + u_dot * dt_;
    next_state(4) = v + v_dot * dt_;
    next_state(5) = r + r_dot * dt_;

    return next_state;
  }

  size_t nearest_path_index() const {
    if (reference_path_.empty()) {
      return 0;
    }

    double best_dist = std::numeric_limits<double>::max();
    size_t best_index = 0;

    double x = current_state_(0);
    double y = current_state_(1);

    for (size_t i = 0; i < reference_path_.size(); i++) {
      double dx = reference_path_[i](0) - x;
      double dy = reference_path_[i](1) - y;
      double d2 = dx * dx + dy * dy;

      if (d2 < best_dist) {
        best_dist = d2;
        best_index = i;
      }
    }

    return best_index;
  }

  Eigen::Vector3d reference_for_step(
    int h,
    const Eigen::VectorXd& predicted_state
  ) const {
    if (has_path_ && !reference_path_.empty()) {
      size_t start_idx = nearest_path_index();

      double lookahead_distance = desired_speed_ * dt_ * static_cast<double>(h);
      size_t offset = static_cast<size_t>(
        std::round(lookahead_distance / path_ds_)
      );

      size_t idx = start_idx + offset;

      if (idx >= reference_path_.size()) {
        idx = reference_path_.size() - 1;
      }

      return reference_path_[idx];
    }

    Eigen::Vector3d ref;
    double sx = goal_start_pose_(0);
    double sy = goal_start_pose_(1);

    double gx = goal_pose_(0);
    double gy = goal_pose_(1);

    double line_dx = gx - sx;
    double line_dy = gy - sy;
    double line_len = std::hypot(line_dx, line_dy);

    if (line_len < 1e-6) {
      ref(0) = gx;
      ref(1) = gy;
      ref(2) = goal_pose_(2);
      return ref;
    }

    double dir_x = line_dx / line_len;
    double dir_y = line_dy / line_len;

    double px = predicted_state(0);
    double py = predicted_state(1);

    // Project predicted boat position onto the start-goal line.
    double progress = (px - sx) * dir_x + (py - sy) * dir_y;
    progress = std::clamp(progress, 0.0, line_len);

    // Future reference point along the line.
    double lookahead = goal_lookahead_ + desired_speed_ * dt_ * static_cast<double>(h);
    double ref_progress = std::clamp(progress + lookahead, 0.0, line_len);

    ref(0) = sx + ref_progress * dir_x;
    ref(1) = sy + ref_progress * dir_y;

    // Heading along the line, not random final RViz orientation.
    ref(2) = std::atan2(dir_y, dir_x);

    return ref;
  }

  double stage_cost(
    const Eigen::VectorXd& state,
    const Eigen::Vector3d& ref,
    const Eigen::Vector4d& control,
    const Eigen::Vector4d& previous_control
  ) const {
    double x = state(0);
    double y = state(1);
    double yaw = state(2);
    double u = state(3);
    double v = state(4);
    double r = state(5);

    double ref_x = ref(0);
    double ref_y = ref(1);
    double ref_yaw = ref(2);

    double dx = x - ref_x;
    double dy = y - ref_y;

    double e_long =  std::cos(ref_yaw) * dx + std::sin(ref_yaw) * dy;
    double e_lat  = -std::sin(ref_yaw) * dx + std::cos(ref_yaw) * dy;

    double yaw_error = wrap_angle(ref_yaw - yaw);

    double thrust_L = control(0);
    double thrust_R = control(1);
    double alpha_L = control(2);
    double alpha_R = control(3);

    double dthrust_L = control(0) - previous_control(0);
    double dthrust_R = control(1) - previous_control(1);
    double dangle_L = control(2) - previous_control(2);
    double dangle_R = control(3) - previous_control(3);

    double dist_to_goal = std::hypot(goal_pose_(0) - x, goal_pose_(1) - y);

    double cost = 0.0;

    cost += q_long_ * e_long * e_long;
    cost += q_lat_  * e_lat  * e_lat;
    cost += q_yaw_  * yaw_error * yaw_error;

    cost += q_sway_vel_ * v * v;
    cost += q_yaw_rate_ * r * r;

    cost += r_thrust_ * (thrust_L * thrust_L + thrust_R * thrust_R);
    cost += r_angle_  * (alpha_L * alpha_L + alpha_R * alpha_R);

    cost += r_smooth_thrust_ * (
      dthrust_L * dthrust_L + dthrust_R * dthrust_R
    );

    cost += r_smooth_angle_ * (
      dangle_L * dangle_L + dangle_R * dangle_R
    );

    double thrust_diff = thrust_L - thrust_R;
    double angle_diff = alpha_L - alpha_R;

    cost += r_sym_thrust_ * thrust_diff * thrust_diff;
    cost += r_sym_angle_  * angle_diff  * angle_diff;

    if (dist_to_goal > 3.0) {
      if (thrust_L < 0.0) {
        cost += reverse_penalty_ * thrust_L * thrust_L;
      }

      if (thrust_R < 0.0) {
        cost += reverse_penalty_ * thrust_R * thrust_R;
      }
    }

    if (dist_to_goal < 4.0) {
      cost += near_goal_u_penalty_ * u * u;
      cost += near_goal_v_penalty_ * v * v;
      cost += near_goal_r_penalty_ * r * r;
    }

    return cost;
  }

  double terminal_cost(const Eigen::VectorXd& state) const {
    double dx = goal_pose_(0) - state(0);
    double dy = goal_pose_(1) - state(1);
    double dist = std::hypot(dx, dy);

    double yaw_error = wrap_angle(goal_pose_(2) - state(2));

    double u = state(3);
    double v = state(4);
    double r = state(5);

    double cost = 0.0;

    cost += terminal_dist_weight_ * dist * dist;
    if (dist < 1.5) {
      cost += terminal_yaw_weight_ * yaw_error * yaw_error;
    }
    if (dist < 5.0) {
      cost += 800.0 * u * u;
      cost += 900.0 * v * v;
      cost += 400.0 * r * r;
    }

    return cost;
  }

  void clamp_nominal_sequence() {
    for (int h = 0; h < horizon_; h++) {
      U_(0, h) = std::clamp(U_(0, h), -1.0, 1.0);
      U_(1, h) = std::clamp(U_(1, h), -1.0, 1.0);

      U_(2, h) = std::clamp(U_(2, h), -max_angle_, max_angle_);
      U_(3, h) = std::clamp(U_(3, h), -max_angle_, max_angle_);
    }
  }

  Eigen::Vector4d sample_noise() {
    Eigen::Vector4d noise;

    noise(0) = sigma_thrust_ * normal_(rng_);
    noise(1) = sigma_thrust_ * normal_(rng_);

    noise(2) = sigma_angle_ * normal_(rng_);
    noise(3) = sigma_angle_ * normal_(rng_);

    return noise;
  }

  void seed_nominal_controls_if_needed() {
    if (U_.norm() > 1e-6) {
      return;
    }

    double dx = goal_pose_(0) - current_state_(0);
    double dy = goal_pose_(1) - current_state_(1);
    double dist = std::hypot(dx, dy);

    double desired_yaw = std::atan2(dy, dx);
    double yaw_error = wrap_angle(desired_yaw - current_state_(2));

    double base = std::clamp(0.025 * dist, 0.03, 0.18);

    if (std::abs(yaw_error) > 1.0) {
      base = 0.06;
    }

    double turn = std::clamp(0.18 * yaw_error, -0.18, 0.18);

    for (int h = 0; h < horizon_; h++) {
      U_(0, h) = std::clamp(base - turn, -1.0, 1.0);
      U_(1, h) = std::clamp(base + turn, -1.0, 1.0);

      U_(2, h) = 0.0;
      U_(3, h) = 0.0;
    }
  }

  void control_loop() {
    if (!has_odom_ || !goal_received_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        2000,
        "Waiting for odometry and RViz goal..."
      );
      return;
    }

    double dx_goal = goal_pose_(0) - current_state_(0);
    double dy_goal = goal_pose_(1) - current_state_(1);
    double dist_to_goal = std::hypot(dx_goal, dy_goal);

    double speed = std::hypot(current_state_(3), current_state_(4));
    double yaw_rate = std::abs(current_state_(5));
    double yaw_error_final = std::abs(wrap_angle(goal_pose_(2) - current_state_(2)));

    if (
      dist_to_goal < goal_pos_tolerance_ &&
      speed < goal_speed_tolerance_ &&
      yaw_rate < goal_yaw_rate_tolerance_
    ) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Goal reached and boat stopped."
      );

      publish_stop();
      goal_received_ = false;
      U_.setZero();
      return;
    }

    seed_nominal_controls_if_needed();

    std::vector<double> rollout_costs(K_, 0.0);
    std::vector<Eigen::MatrixXd> rollout_noises(
      K_,
      Eigen::MatrixXd::Zero(4, horizon_)
    );

    for (int k = 0; k < K_; k++) {
      Eigen::VectorXd predicted_state = current_state_;
      Eigen::Vector4d previous_control = last_cmd_;

      double total_cost = 0.0;

      for (int h = 0; h < horizon_; h++) {
        Eigen::Vector4d u_nominal = U_.col(h);

        Eigen::Vector4d noise = sample_noise();
        Eigen::Vector4d desired_control = u_nominal + noise;

        clamp_control(desired_control);

        Eigen::Vector4d noisy_control = rate_limit_control(
          desired_control,
          previous_control
        );

        rollout_noises[k].col(h) = noisy_control - u_nominal;

        predicted_state = predict_state(predicted_state, noisy_control);

        Eigen::Vector3d ref = reference_for_step(h, predicted_state);

        total_cost += stage_cost(
          predicted_state,
          ref,
          noisy_control,
          previous_control
        );

        previous_control = noisy_control;
      }

      total_cost += terminal_cost(predicted_state);

      rollout_costs[k] = total_cost;
    }

    double min_cost = *std::min_element(
      rollout_costs.begin(),
      rollout_costs.end()
    );

    std::vector<double> weights(K_, 0.0);
    double sum_weights = 0.0;

    for (int k = 0; k < K_; k++) {
      double exponent = -(rollout_costs[k] - min_cost) / lambda_;
      exponent = std::clamp(exponent, -60.0, 60.0);

      weights[k] = std::exp(exponent);
      sum_weights += weights[k];
    }

    if (sum_weights < 1e-9) {
      RCLCPP_WARN(this->get_logger(), "MPPI weights collapsed. Skipping update.");
      return;
    }

    Eigen::MatrixXd delta_U = Eigen::MatrixXd::Zero(4, horizon_);

    for (int k = 0; k < K_; k++) {
      weights[k] /= sum_weights;
      delta_U += weights[k] * rollout_noises[k];
    }

    U_ += delta_U;
    clamp_nominal_sequence();

    Eigen::Vector4d desired_cmd = U_.col(0);
    clamp_control(desired_cmd);

    Eigen::Vector4d cmd = rate_limit_control(desired_cmd, last_cmd_);

    std_msgs::msg::Float64 msg_fl;
    std_msgs::msg::Float64 msg_fr;
    std_msgs::msg::Float64 msg_al;
    std_msgs::msg::Float64 msg_ar;

    msg_fl.data = cmd(0) * max_thrust_;
    msg_fr.data = cmd(1) * max_thrust_;
    msg_al.data = cmd(2);
    msg_ar.data = cmd(3);

    pub_fl_->publish(msg_fl);
    pub_fr_->publish(msg_fr);
    pub_al_->publish(msg_al);
    pub_ar_->publish(msg_ar);

    last_cmd_ = cmd;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(),
      *this->get_clock(),
      500,
      "MPPI cmd | L: %.1f N, R: %.1f N | aL: %.2f rad, aR: %.2f rad | dist: %.2f m",
      msg_fl.data,
      msg_fr.data,
      msg_al.data,
      msg_ar.data,
      dist_to_goal
    );

    for (int h = 0; h < horizon_ - 1; h++) {
      U_.col(h) = U_.col(h + 1);
    }

    U_.col(horizon_ - 1).setZero();
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fl_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_fr_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_al_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_ar_;

  rclcpp::TimerBase::SharedPtr timer_;

  bool has_odom_ = false;
  bool goal_received_ = false;
  bool has_path_ = false;

  Eigen::VectorXd current_state_;
  Eigen::Vector3d goal_pose_;

  std::vector<Eigen::Vector3d> reference_path_;

  const double dt_ = 0.1;
  const int horizon_ = 60;
  const int K_ = 500;

  Eigen::MatrixXd U_;
  Eigen::Vector4d last_cmd_;
  Eigen::Vector3d goal_start_pose_;

  const double lambda_ = 120.0;

  std::mt19937 rng_{std::random_device{}()};
  std::normal_distribution<double> normal_{0.0, 1.0};

  const double sigma_thrust_ = 0.10;
  const double sigma_angle_ = 0.04;

  const double max_thrust_ = 5000.0;
  const double max_angle_ = PI / 4.0;

  const double max_thrust_rate_normalized_ = 0.08;
  const double max_angle_rate_ = 0.08;

  const double L_ = 6.0;
  const double W_ = 1.2;

  const double m_u_ = 1000.0;
  const double m_v_ = 1000.0;
  const double m_r_ = 446.0;

  const double d_u_ = 182.0;
  const double d_v_ = 183.0;
  const double d_r_ = 1199.0;

  const double path_ds_ = 0.4;
  const double desired_speed_ = 1.0;

  const double q_long_ = 10.0;
  const double q_lat_ = 250.0;
  const double q_yaw_ = 200.0;

  const double q_sway_vel_ = 350.0;
  const double q_yaw_rate_ = 60.0;

  const double r_thrust_ = 2.0;
  const double r_angle_ = 30.0;

  const double r_smooth_thrust_ = 80.0;
  const double r_smooth_angle_ = 250.0;

  const double r_sym_thrust_ = 8.0;
  const double r_sym_angle_ = 20.0;

  const double reverse_penalty_ = 200.0;

  const double near_goal_u_penalty_ = 500.0;
  const double near_goal_v_penalty_ = 700.0;
  const double near_goal_r_penalty_ = 300.0;

  const double terminal_dist_weight_ = 80.0;
  const double terminal_yaw_weight_ = 0.0;

  const double final_yaw_distance_ = 2.5;
  const double goal_lookahead_ = 1.5;

  const double goal_pos_tolerance_ = 1.2;
  const double goal_speed_tolerance_ = 0.30;
  const double goal_yaw_rate_tolerance_ = 0.20;
  const double goal_yaw_tolerance_ = 0.25;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AquabotController>());
  rclcpp::shutdown();
  return 0;
}