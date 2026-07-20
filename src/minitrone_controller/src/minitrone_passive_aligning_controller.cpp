#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <minitrone_interfaces/msg/minitrone_state.hpp>
#include <minitrone_interfaces/msg/wrench.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace
{
constexpr double kPi = 3.14159265358979323846;

double moveToward(double current, double target, double max_step)
{
  const double step = std::max(0.0, max_step);
  return current < target ? std::min(current + step, target) :
         std::max(current - step, target);
}
}  // namespace

class PassiveAligningController : public rclcpp::Node
{
public:
  PassiveAligningController()
  : rclcpp::Node("minitrone_passive_aligning_controller")
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_wrench_topic", "/minitrone/wrench_cmd");
    const auto output_topic = declare_parameter<std::string>(
      "output_wrench_topic", "/minitrone/wrench_passive_align");

    enabled_ = declare_parameter<bool>("enabled", false);
    normal_ << declare_parameter<double>("n_face_b_x", 1.0),
      declare_parameter<double>("n_face_b_y", 0.0),
      declare_parameter<double>("n_face_b_z", 0.0);
    if (normal_.norm() < 1e-6) {
      normal_ = Eigen::Vector3d::UnitX();
    }
    normal_.normalize();

    f_des_ = declare_parameter<double>("f_normal_des", 5.0);
    f_step_ = std::abs(declare_parameter<double>("f_normal_step", 0.1));
    f_min_ = declare_parameter<double>("f_normal_min", 0.0);
    f_max_ = declare_parameter<double>("f_normal_max", 20.0);
    if (f_min_ > f_max_) std::swap(f_min_, f_max_);
    f_des_ = std::clamp(f_des_, f_min_, f_max_);
    f_rate_ = std::abs(declare_parameter<double>("f_ref_rate_max", 1.0));
    force_kp_ = std::max(0.0, declare_parameter<double>("force_kp", 0.5));
    force_ki_ = std::max(0.0, declare_parameter<double>("force_ki", 0.0));
    integral_limit_ = std::abs(declare_parameter<double>("force_integral_limit", 2.0));
    force_cmd_min_ = declare_parameter<double>("force_cmd_min", 0.0);
    force_cmd_max_ = declare_parameter<double>("force_cmd_max", 20.0);
    if (force_cmd_min_ > force_cmd_max_) std::swap(force_cmd_min_, force_cmd_max_);
    cutoff_hz_ = std::max(0.0, declare_parameter<double>("force_lpf_cutoff_hz", 10.0));
    timeout_sec_ = std::max(0.0, declare_parameter<double>("wrench_timeout_sec", 0.1));

    passive_scale_ = std::clamp(
      declare_parameter<double>("passive_axis_control_scale", 0.0), 0.0, 1.0);
    damping_ <<
      std::max(0.0, declare_parameter<double>("passive_damping_roll", 0.0)),
      std::max(0.0, declare_parameter<double>("passive_damping_pitch", 0.05)),
      std::max(0.0, declare_parameter<double>("passive_damping_yaw", 0.05));
    moment_limit_ = std::abs(declare_parameter<double>("passive_moment_limit", 0.5));
    transition_sec_ = std::max(0.0, declare_parameter<double>("transition_time_sec", 0.3));

    input_sub_ = create_subscription<minitrone_interfaces::msg::Wrench>(
      input_topic, 10, std::bind(&PassiveAligningController::onInput, this, std::placeholders::_1));
    state_sub_ = create_subscription<minitrone_interfaces::msg::MinitroneState>(
      "/minitrone/state", 10,
      [this](const minitrone_interfaces::msg::MinitroneState::SharedPtr msg) {
        omega_ << msg->w_rpy[0], msg->w_rpy[1], msg->w_rpy[2];
      });
    external_sub_ = create_subscription<minitrone_interfaces::msg::Wrench>(
      "/minitrone/external_wrench_hat_second_order", 10,
      [this](const minitrone_interfaces::msg::Wrench::SharedPtr msg) {
        external_force_ << msg->force[0], msg->force[1], msg->force[2];
        have_external_ = true;
        last_external_time_ = now();
      });
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/minitrone/passive_align_enable", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {setEnabled(msg->data);});

    output_pub_ = create_publisher<minitrone_interfaces::msg::Wrench>(output_topic, 10);
    active_pub_ = create_publisher<std_msgs::msg::Bool>("/minitrone/passive_align_active", 10);
    desired_pub_ = create_publisher<std_msgs::msg::Float64>("/minitrone/passive_align_des_force", 10);
    measured_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/minitrone/passive_align_measured_force", 10);

    parameter_handle_ = add_on_set_parameters_callback(
      std::bind(&PassiveAligningController::onParameters, this, std::placeholders::_1));
    setupKeyboard();
    keyboard_timer_ = create_wall_timer(
      std::chrono::milliseconds(20), std::bind(&PassiveAligningController::pollKeyboard, this));
    last_output_time_ = now();
    publishStatus();
    RCLCPP_INFO(get_logger(), "Passive aligning ready. L=ON/OFF, U/J=force +/-");
  }

  ~PassiveAligningController() override {restoreKeyboard();}

private:
  void onInput(const minitrone_interfaces::msg::Wrench::SharedPtr msg)
  {
    const auto t = now();
    double dt = (t - last_output_time_).seconds();
    last_output_time_ = t;
    if (!(dt > 0.0) || dt > 0.2) dt = 1.0 / 400.0;

    const double blend_target = enabled_ ? 1.0 : 0.0;
    blend_ = transition_sec_ <= 1e-6 ? blend_target :
      moveToward(blend_, blend_target, dt / transition_sec_);
    f_ref_ = moveToward(f_ref_, enabled_ ? f_des_ : 0.0, f_rate_ * dt);

    Eigen::Vector3d force(msg->force[0], msg->force[1], msg->force[2]);
    Eigen::Vector3d moment(msg->moment[0], msg->moment[1], msg->moment[2]);
    const Eigen::Matrix3d pn = normal_ * normal_.transpose();
    const Eigen::Matrix3d pt = Eigen::Matrix3d::Identity() - pn;
    Eigen::Vector3d hybrid_force = force;

    const bool external_fresh = have_external_ &&
      (timeout_sec_ <= 0.0 || (t - last_external_time_).seconds() <= timeout_sec_);
    if (external_fresh) {
      const double measured = std::max(0.0, -normal_.dot(external_force_));
      if (!filter_initialized_) {
        filtered_force_ = measured;
        filter_initialized_ = true;
      } else {
        const double alpha = cutoff_hz_ <= 0.0 ? 1.0 : std::clamp(
          1.0 - std::exp(-2.0 * kPi * cutoff_hz_ * dt), 0.0, 1.0);
        filtered_force_ += alpha * (measured - filtered_force_);
      }
      const double error = f_ref_ - filtered_force_;
      force_integral_ = std::clamp(
        force_integral_ + error * dt, -integral_limit_, integral_limit_);
      const double normal_cmd = std::clamp(
        f_ref_ + force_kp_ * error + force_ki_ * force_integral_,
        force_cmd_min_, force_cmd_max_);
      hybrid_force = pt * force + normal_cmd * normal_;
    } else if (enabled_) {
      force_integral_ = 0.0;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "external wrench stale: upstream normal force is passed through");
    }

    Eigen::Vector3d damping = -(damping_.asDiagonal() * (pt * omega_));
    damping = pt * damping;
    for (int i = 0; i < 3; ++i) {
      damping[i] = std::clamp(damping[i], -moment_limit_, moment_limit_);
    }
    const Eigen::Vector3d hybrid_moment =
      pn * moment + passive_scale_ * pt * moment + damping;
    const Eigen::Vector3d force_out = (1.0 - blend_) * force + blend_ * hybrid_force;
    const Eigen::Vector3d moment_out = (1.0 - blend_) * moment + blend_ * hybrid_moment;

    minitrone_interfaces::msg::Wrench out;
    for (int i = 0; i < 3; ++i) {
      out.force[i] = static_cast<float>(force_out[i]);
      out.moment[i] = static_cast<float>(moment_out[i]);
    }
    output_pub_->publish(out);
    publishStatus();
  }

  void setEnabled(bool value)
  {
    if (enabled_ == value) return;
    enabled_ = value;
    force_integral_ = 0.0;
    if (enabled_ && filter_initialized_) {
      f_ref_ = std::clamp(filtered_force_, f_min_, f_des_);
    }
    RCLCPP_INFO(get_logger(), "PASSIVE ALIGN %s", enabled_ ? "ON" : "OFF");
    publishStatus();
  }

  rcl_interfaces::msg::SetParametersResult onParameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & p : params) {
      const auto & name = p.get_name();
      if (name == "enabled") setEnabled(p.as_bool());
      else if (name == "f_normal_des") f_des_ = std::clamp(p.as_double(), f_min_, f_max_);
      else if (name == "force_kp") force_kp_ = std::max(0.0, p.as_double());
      else if (name == "force_ki") force_ki_ = std::max(0.0, p.as_double());
      else if (name == "passive_axis_control_scale") passive_scale_ = std::clamp(p.as_double(), 0.0, 1.0);
      else if (name == "passive_damping_pitch") damping_.y() = std::max(0.0, p.as_double());
      else if (name == "passive_damping_yaw") damping_.z() = std::max(0.0, p.as_double());
    }
    return result;
  }

  void publishStatus()
  {
    std_msgs::msg::Bool active;
    active.data = enabled_;
    active_pub_->publish(active);
    std_msgs::msg::Float64 desired;
    desired.data = f_des_;
    desired_pub_->publish(desired);
    std_msgs::msg::Float64 measured;
    measured.data = filtered_force_;
    measured_pub_->publish(measured);
  }

  void setupKeyboard()
  {
    keyboard_enabled_ = isatty(STDIN_FILENO);
    if (!keyboard_enabled_ || tcgetattr(STDIN_FILENO, &old_termios_) != 0) return;
    termios raw = old_termios_;
    raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) termios_configured_ = true;
  }

  void restoreKeyboard()
  {
    if (termios_configured_) tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
  }

  void pollKeyboard()
  {
    if (!keyboard_enabled_) return;
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    timeval timeout{0, 0};
    if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &timeout) <= 0) return;
    char key = '\0';
    if (read(STDIN_FILENO, &key, 1) != 1) return;
    if (key == 'l' || key == 'L') setEnabled(!enabled_);
    else if (key == 'u' || key == 'U') f_des_ = std::clamp(f_des_ + f_step_, f_min_, f_max_);
    else if (key == 'j' || key == 'J') f_des_ = std::clamp(f_des_ - f_step_, f_min_, f_max_);
  }

  rclcpp::Subscription<minitrone_interfaces::msg::Wrench>::SharedPtr input_sub_, external_sub_;
  rclcpp::Subscription<minitrone_interfaces::msg::MinitroneState>::SharedPtr state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::Publisher<minitrone_interfaces::msg::Wrench>::SharedPtr output_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr desired_pub_, measured_pub_;
  rclcpp::TimerBase::SharedPtr keyboard_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_handle_;

  Eigen::Vector3d normal_{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d damping_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d omega_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d external_force_{Eigen::Vector3d::Zero()};
  double f_des_{5.0}, f_step_{0.1}, f_min_{0.0}, f_max_{20.0}, f_rate_{1.0};
  double force_kp_{0.5}, force_ki_{0.0}, integral_limit_{2.0}, force_integral_{0.0};
  double force_cmd_min_{0.0}, force_cmd_max_{20.0}, cutoff_hz_{10.0};
  double timeout_sec_{0.1}, passive_scale_{0.0}, moment_limit_{0.5}, transition_sec_{0.3};
  double blend_{0.0}, f_ref_{0.0}, filtered_force_{0.0};
  rclcpp::Time last_output_time_, last_external_time_;
  bool enabled_{false}, have_external_{false}, filter_initialized_{false};
  bool keyboard_enabled_{false}, termios_configured_{false};
  termios old_termios_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PassiveAligningController>());
  rclcpp::shutdown();
  return 0;
}
