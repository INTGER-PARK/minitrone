#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <minitrone_interfaces/msg/attitude_cmd.hpp>
#include <minitrone_interfaces/msg/cmd.hpp>
#include <minitrone_interfaces/msg/minitrone_state.hpp>
#include <minitrone_interfaces/msg/wrench.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
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
constexpr double kDegToRad = kPi / 180.0;
constexpr double kRadToDeg = 180.0 / kPi;
constexpr std::size_t kDof = 6;

const std::array<std::string, kDof> kAxisNames = {
  "x", "y", "z", "roll", "pitch", "yaw"};

Eigen::Matrix3d rotationWorldFromBody(const Eigen::Vector3d & rpy)
{
  const double r = rpy.x();
  const double p = rpy.y();
  const double y = rpy.z();

  const double sr = std::sin(r);
  const double cr = std::cos(r);
  const double sp = std::sin(p);
  const double cp = std::cos(p);
  const double sy = std::sin(y);
  const double cy = std::cos(y);

  Eigen::Matrix3d r_wb;
  r_wb <<
    cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
    sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
    -sp, cp * sr, cp * cr;
  return r_wb;
}

Eigen::Matrix3d rotationExp(const Eigen::Vector3d & phi)
{
  const double angle = phi.norm();
  if (angle < 1e-10) {
    Eigen::Matrix3d hat;
    hat <<
      0.0, -phi.z(), phi.y(),
      phi.z(), 0.0, -phi.x(),
      -phi.y(), phi.x(), 0.0;
    return Eigen::Matrix3d::Identity() + hat;
  }
  return Eigen::AngleAxisd(angle, phi / angle).toRotationMatrix();
}

Eigen::Vector3d rpyFromRotation(const Eigen::Matrix3d & r)
{
  // ZYX convention: R = Rz(yaw) * Ry(pitch) * Rx(roll)
  const double pitch = std::asin(std::clamp(-r(2, 0), -1.0, 1.0));
  const double cp = std::cos(pitch);

  double roll = 0.0;
  double yaw = 0.0;
  if (std::abs(cp) > 1e-7) {
    roll = std::atan2(r(2, 1), r(2, 2));
    yaw = std::atan2(r(1, 0), r(0, 0));
  } else {
    // Near pitch = +-90 deg, choose roll = 0 and preserve a valid yaw.
    roll = 0.0;
    yaw = std::atan2(-r(0, 1), r(1, 1));
  }
  return Eigen::Vector3d(roll, pitch, yaw);
}

double unwrapNear(double angle, double reference)
{
  return reference + std::atan2(std::sin(angle - reference), std::cos(angle - reference));
}

double moveToward(double current, double target, double max_step)
{
  const double step = std::max(0.0, max_step);
  if (current < target) {
    return std::min(current + step, target);
  }
  return std::max(current - step, target);
}
}  // namespace

class AdmittanceController6Dof : public rclcpp::Node
{
public:
  enum class Mode
  {
    PASSTHROUGH,
    ADMITTANCE,
    HOLD
  };

  AdmittanceController6Dof()
  : rclcpp::Node("minitrone_admittance_controller_6dof")
  {
    // ------------------------------------------------------------------------
    // Global and per-axis enable settings
    // ------------------------------------------------------------------------
    const bool initially_enabled = declare_parameter<bool>("enabled", false);
    for (std::size_t i = 0; i < kDof; ++i) {
      axis_enabled_[i] = declare_parameter<bool>("enable_" + kAxisNames[i], true);
    }

    // Contact surface direction in BODY frame: robot -> contact surface.
    n_face_body_ <<
      declare_parameter<double>("n_face_b_x", 1.0),
      declare_parameter<double>("n_face_b_y", 0.0),
      declare_parameter<double>("n_face_b_z", 0.0);
    if (n_face_body_.norm() < 1e-6) {
      n_face_body_ = Eigen::Vector3d::UnitX();
    }
    n_face_body_.normalize();

    // ------------------------------------------------------------------------
    // Normal-force tracking
    // ------------------------------------------------------------------------
    f_normal_des_ = declare_parameter<double>("f_normal_des", 5.0);
    f_normal_step_ = std::abs(declare_parameter<double>("f_normal_step", 0.1));
    f_normal_min_ = declare_parameter<double>("f_normal_min", 0.0);
    f_normal_max_ = declare_parameter<double>("f_normal_max", 20.0);
    f_ref_rate_max_ = std::abs(declare_parameter<double>("f_ref_rate_max", 1.0));
    normal_force_integral_gain_ =
      declare_parameter<double>("normal_force_integral_gain", 0.0);
    normal_force_integral_limit_ = std::abs(
      declare_parameter<double>("normal_force_integral_limit", 1.0));

    if (f_normal_min_ > f_normal_max_) {
      std::swap(f_normal_min_, f_normal_max_);
    }
    f_normal_des_ = std::clamp(f_normal_des_, f_normal_min_, f_normal_max_);

    // ------------------------------------------------------------------------
    // Six-axis virtual M, D, K.
    // First 3 axes: [m, m/s] related translational motion.
    // Last 3 axes: [rad, rad/s] related rotational motion.
    // ------------------------------------------------------------------------
    const std::array<double, kDof> m_default = {1.0, 1.0, 1.0, 0.20, 0.20, 0.30};
    const std::array<double, kDof> d_default = {20.0, 20.0, 20.0, 2.0, 2.0, 2.0};
    const std::array<double, kDof> k_default = {0.0, 0.0, 0.0, 1.0, 1.0, 1.0};

    for (std::size_t i = 0; i < kDof; ++i) {
      adm_m_[i] = std::max(
        std::abs(declare_parameter<double>("adm_m_" + kAxisNames[i], m_default[i])),
        1e-6);
      adm_d_[i] = std::max(
        0.0, declare_parameter<double>("adm_d_" + kAxisNames[i], d_default[i]));
      adm_k_[i] = std::max(
        0.0, declare_parameter<double>("adm_k_" + kAxisNames[i], k_default[i]));
    }

    // ------------------------------------------------------------------------
    // Independent LPF cutoff frequency for Fx,Fy,Fz,Mx,My,Mz.
    // ------------------------------------------------------------------------
    for (std::size_t i = 0; i < kDof; ++i) {
      cutoff_hz_[i] = std::max(
        0.0, declare_parameter<double>("cutoff_" + kAxisNames[i] + "_hz", 10.0));
    }

    // Input deadband and saturation.
    const std::array<double, kDof> deadband_default = {
      0.05, 0.05, 0.05, 0.01, 0.01, 0.01};
    const std::array<double, kDof> input_max_default = {
      2.0, 2.0, 2.0, 0.50, 0.50, 0.50};

    // Virtual acceleration, velocity, and displacement limits.
    const std::array<double, kDof> ddq_max_default = {
      0.10, 0.10, 0.10,
      30.0 * kDegToRad, 30.0 * kDegToRad, 30.0 * kDegToRad};
    const std::array<double, kDof> dq_max_default = {
      0.05, 0.05, 0.05,
      10.0 * kDegToRad, 10.0 * kDegToRad, 10.0 * kDegToRad};
    const std::array<double, kDof> q_max_default = {
      0.15, 0.15, 0.15,
      10.0 * kDegToRad, 10.0 * kDegToRad, 15.0 * kDegToRad};

    for (std::size_t i = 0; i < kDof; ++i) {
      input_deadband_[i] = std::abs(declare_parameter<double>(
        "input_deadband_" + kAxisNames[i], deadband_default[i]));
      input_max_[i] = std::abs(declare_parameter<double>(
        "input_max_" + kAxisNames[i], input_max_default[i]));
      ddq_max_[i] = std::abs(declare_parameter<double>(
        "ddq_max_" + kAxisNames[i], ddq_max_default[i]));
      dq_max_[i] = std::abs(declare_parameter<double>(
        "dq_max_" + kAxisNames[i], dq_max_default[i]));
      q_max_[i] = std::abs(declare_parameter<double>(
        "q_max_" + kAxisNames[i], q_max_default[i]));
    }

    wrench_timeout_sec_ = std::max(
      0.0, declare_parameter<double>("wrench_timeout_sec", 0.10));

    // ------------------------------------------------------------------------
    // ROS I/O
    // ------------------------------------------------------------------------
    sub_cmd_ = create_subscription<minitrone_interfaces::msg::Cmd>(
      "/minitrone/cmd", 10,
      std::bind(&AdmittanceController6Dof::onCmd, this, std::placeholders::_1));

    sub_att_cmd_ = create_subscription<minitrone_interfaces::msg::AttitudeCmd>(
      "/minitrone/att_cmd", 10,
      std::bind(&AdmittanceController6Dof::onAttCmd, this, std::placeholders::_1));

    sub_state_ = create_subscription<minitrone_interfaces::msg::MinitroneState>(
      "/minitrone/state", 10,
      std::bind(&AdmittanceController6Dof::onState, this, std::placeholders::_1));

    sub_external_wrench_ = create_subscription<minitrone_interfaces::msg::Wrench>(
      "/minitrone/external_wrench_hat_second_order", 10,
      std::bind(
        &AdmittanceController6Dof::onExternalWrench, this, std::placeholders::_1));

    // true: enter 6-DOF admittance, false: hold current measured pose.
    sub_enable_ = create_subscription<std_msgs::msg::Bool>(
      "/minitrone/admittance_enable", 10,
      std::bind(&AdmittanceController6Dof::onEnable, this, std::placeholders::_1));

    pub_cmd_ = create_publisher<minitrone_interfaces::msg::Cmd>(
      "/minitrone/cmd_admittance", 10);
    pub_att_cmd_ = create_publisher<minitrone_interfaces::msg::AttitudeCmd>(
      "/minitrone/att_cmd_admittance", 10);
    pub_active_ = create_publisher<std_msgs::msg::Bool>(
      "/minitrone/admittance_active", 10);
    pub_des_force_ = create_publisher<std_msgs::msg::Float64>(
      "/minitrone/admittance_des_force", 10);

    parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(
        &AdmittanceController6Dof::onSetParameters,
        this,
        std::placeholders::_1));

    setupKeyboard();
    keyboard_timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&AdmittanceController6Dof::pollKeyboard, this));

    last_control_time_ = now();
    pending_enable_ = initially_enabled;
    publishStatus();

    RCLCPP_INFO(
      get_logger(),
      "6-DOF admittance ready. I=toggle admittance/HOLD, "
      "P=PASSTHROUGH, U/J=normal force +/- %.2f N",
      f_normal_step_);
  }

  ~AdmittanceController6Dof() override
  {
    restoreKeyboard();
  }

private:
  void onCmd(const minitrone_interfaces::msg::Cmd::SharedPtr msg)
  {
    upstream_pos_cmd_ <<
      static_cast<double>(msg->pos_cmd[0]),
      static_cast<double>(msg->pos_cmd[1]),
      static_cast<double>(msg->pos_cmd[2]);
    have_cmd_ = true;
  }

  void onAttCmd(const minitrone_interfaces::msg::AttitudeCmd::SharedPtr msg)
  {
    upstream_att_cmd_rad_ <<
      static_cast<double>(msg->roll_ref) * kDegToRad,
      static_cast<double>(msg->pitch_ref) * kDegToRad,
      static_cast<double>(msg->yaw_ref) * kDegToRad;
    have_att_cmd_ = true;
  }

  void onExternalWrench(const minitrone_interfaces::msg::Wrench::SharedPtr msg)
  {
    external_force_body_ <<
      static_cast<double>(msg->force[0]),
      static_cast<double>(msg->force[1]),
      static_cast<double>(msg->force[2]);
    external_moment_body_ <<
      static_cast<double>(msg->moment[0]),
      static_cast<double>(msg->moment[1]),
      static_cast<double>(msg->moment[2]);
    have_external_wrench_ = true;
    last_wrench_time_ = now();
  }

  void onEnable(const std_msgs::msg::Bool::SharedPtr msg)
  {
    requestEnabled(msg->data);
  }

  void requestEnabled(bool enabled)
  {
    const auto result = set_parameter(rclcpp::Parameter("enabled", enabled));
    if (!result.successful) {
      RCLCPP_ERROR(
        get_logger(), "failed to set enabled=%s: %s",
        enabled ? "true" : "false", result.reason.c_str());
    }
  }

  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & parameter : parameters) {
      const std::string & name = parameter.get_name();
      if (name == "enabled") {
        setEnabledInternal(parameter.as_bool());
        continue;
      }
      if (name == "f_normal_des") {
        f_normal_des_ = std::clamp(
          parameter.as_double(), f_normal_min_, f_normal_max_);
        continue;
      }
      if (name == "f_ref_rate_max") {
        f_ref_rate_max_ = std::abs(parameter.as_double());
        continue;
      }
      if (name == "normal_force_integral_gain") {
        normal_force_integral_gain_ = parameter.as_double();
        continue;
      }

      for (std::size_t i = 0; i < kDof; ++i) {
        if (name == "enable_" + kAxisNames[i]) {
          axis_enabled_[i] = parameter.as_bool();
          if (!axis_enabled_[i]) {
            q_[i] = 0.0;
            dq_[i] = 0.0;
          }
        } else if (name == "adm_m_" + kAxisNames[i]) {
          adm_m_[i] = std::max(std::abs(parameter.as_double()), 1e-6);
        } else if (name == "adm_d_" + kAxisNames[i]) {
          adm_d_[i] = std::max(0.0, parameter.as_double());
        } else if (name == "adm_k_" + kAxisNames[i]) {
          adm_k_[i] = std::max(0.0, parameter.as_double());
        } else if (name == "cutoff_" + kAxisNames[i] + "_hz") {
          cutoff_hz_[i] = std::max(0.0, parameter.as_double());
        } else if (name == "input_deadband_" + kAxisNames[i]) {
          input_deadband_[i] = std::abs(parameter.as_double());
        } else if (name == "input_max_" + kAxisNames[i]) {
          input_max_[i] = std::abs(parameter.as_double());
        } else if (name == "ddq_max_" + kAxisNames[i]) {
          ddq_max_[i] = std::abs(parameter.as_double());
        } else if (name == "dq_max_" + kAxisNames[i]) {
          dq_max_[i] = std::abs(parameter.as_double());
        } else if (name == "q_max_" + kAxisNames[i]) {
          q_max_[i] = std::abs(parameter.as_double());
        }
      }
    }
    return result;
  }

  void onState(const minitrone_interfaces::msg::MinitroneState::SharedPtr msg)
  {
    pos_ <<
      static_cast<double>(msg->pos[0]),
      static_cast<double>(msg->pos[1]),
      static_cast<double>(msg->pos[2]);
    vel_ <<
      static_cast<double>(msg->vel[0]),
      static_cast<double>(msg->vel[1]),
      static_cast<double>(msg->vel[2]);
    rpy_ <<
      static_cast<double>(msg->rpy[0]),
      static_cast<double>(msg->rpy[1]),
      static_cast<double>(msg->rpy[2]);
    w_body_ <<
      static_cast<double>(msg->w_rpy[0]),
      static_cast<double>(msg->w_rpy[1]),
      static_cast<double>(msg->w_rpy[2]);
    have_state_ = true;

    if (pending_enable_) {
      pending_enable_ = false;
      enterAdmittanceMode();
    }
    if (mode_ == Mode::HOLD && !hold_initialized_) {
      captureCurrentHoldPose();
    }

    updateAndPublish();
  }

  void setEnabledInternal(bool enabled)
  {
    if (enabled && mode_ == Mode::ADMITTANCE && !pending_enable_) {
      return;
    }
    if (!enabled && mode_ == Mode::HOLD && !pending_enable_) {
      return;
    }

    if (enabled) {
      if (!have_state_) {
        pending_enable_ = true;
        RCLCPP_WARN(get_logger(), "admittance requested; waiting for state");
        return;
      }
      enterAdmittanceMode();
    } else {
      pending_enable_ = false;
      enterHoldMode();
    }
    publishStatus();
  }

  void enterAdmittanceMode()
  {
    if (!have_state_) {
      pending_enable_ = true;
      return;
    }

    mode_ = Mode::ADMITTANCE;
    const Eigen::Matrix3d r_wb = rotationWorldFromBody(rpy_);

    // A frame is frozen in WORLD at mode entry and initially aligned with BODY.
    r_wa_ = r_wb;
    n_contact_a_ = n_face_body_;

    base_pos_world_ = output_initialized_ ? last_pos_ref_world_ : pos_;
    base_rotation_world_ = output_initialized_ ?
      rotationWorldFromBody(last_att_ref_rad_) : r_wb;

    q_.setZero();
    dq_.setZero();
    wrench_filter_initialized_ = false;
    normal_force_integral_ = 0.0;

    const Eigen::Matrix<double, 6, 1> wrench_a = rawWrenchInA(r_wb);
    const double current_normal_force = std::max(
      0.0, -n_contact_a_.dot(wrench_a.head<3>()));
    f_normal_ref_active_ = std::clamp(
      current_normal_force,
      f_normal_min_,
      f_normal_des_);

    RCLCPP_INFO(
      get_logger(),
      "ADMITTANCE ON: normal force %.3f N -> desired %.3f N",
      current_normal_force,
      f_normal_des_);
  }

  void captureCurrentHoldPose()
  {
    hold_pos_world_ = pos_;
    hold_att_rad_ = rpy_;
    if (output_initialized_) {
      hold_att_rad_.z() = unwrapNear(hold_att_rad_.z(), last_att_ref_rad_.z());
    }
    hold_initialized_ = true;
  }

  void enterHoldMode()
  {
    mode_ = Mode::HOLD;
    hold_initialized_ = false;
    if (!have_state_) {
      return;
    }

    // Requirement: when admittance is disabled, the lower position/attitude
    // controller receives the pose measured at that exact moment, not an old
    // upstream reference.
    captureCurrentHoldPose();

    q_.setZero();
    dq_.setZero();
    normal_force_integral_ = 0.0;

    RCLCPP_INFO(
      get_logger(),
      "ADMITTANCE OFF -> HOLD at p=[%.3f %.3f %.3f], rpy=[%.2f %.2f %.2f] deg",
      hold_pos_world_.x(), hold_pos_world_.y(), hold_pos_world_.z(),
      hold_att_rad_.x() * kRadToDeg,
      hold_att_rad_.y() * kRadToDeg,
      hold_att_rad_.z() * kRadToDeg);
  }

  void enterPassthroughMode()
  {
    mode_ = Mode::PASSTHROUGH;
    pending_enable_ = false;
    q_.setZero();
    dq_.setZero();
    normal_force_integral_ = 0.0;
    RCLCPP_INFO(get_logger(), "PASSTHROUGH mode");
    publishStatus();
  }

  bool wrenchIsFresh(const rclcpp::Time & t_now) const
  {
    if (!have_external_wrench_) {
      return false;
    }
    if (wrench_timeout_sec_ <= 0.0) {
      return true;
    }
    return (t_now - last_wrench_time_).seconds() <= wrench_timeout_sec_;
  }

  Eigen::Matrix<double, 6, 1> rawWrenchInA(const Eigen::Matrix3d & r_wb) const
  {
    Eigen::Matrix<double, 6, 1> wrench_a;
    const Eigen::Matrix3d r_ab = r_wa_.transpose() * r_wb;
    wrench_a.head<3>() = r_ab * external_force_body_;
    wrench_a.tail<3>() = r_ab * external_moment_body_;
    return wrench_a;
  }

  void updateFilteredWrench(double dt, const Eigen::Matrix3d & r_wb)
  {
    const Eigen::Matrix<double, 6, 1> raw = rawWrenchInA(r_wb);
    if (!wrench_filter_initialized_) {
      wrench_filtered_a_ = raw;
      wrench_filter_initialized_ = true;
      return;
    }

    for (std::size_t i = 0; i < kDof; ++i) {
      if (cutoff_hz_[i] <= 0.0) {
        wrench_filtered_a_[i] = raw[i];
        continue;
      }
      const double alpha = 1.0 - std::exp(-2.0 * kPi * cutoff_hz_[i] * dt);
      wrench_filtered_a_[i] +=
        std::clamp(alpha, 0.0, 1.0) * (raw[i] - wrench_filtered_a_[i]);
    }
  }

  Eigen::Matrix<double, 6, 1> computeAdmittanceInput(double dt)
  {
    // External wrench convention:
    //   measured wrench = environment acting on robot.
    // Desired environment force for positive compression is -Fdes*n.
    // Therefore measured - desired gives +(Fdes-Fnormal)*n on the normal axis.
    Eigen::Matrix<double, 6, 1> desired_external_wrench_a =
      Eigen::Matrix<double, 6, 1>::Zero();
    desired_external_wrench_a.head<3>() =
      -f_normal_ref_active_ * n_contact_a_;

    Eigen::Matrix<double, 6, 1> input =
      wrench_filtered_a_ - desired_external_wrench_a;

    const double f_normal_hat = std::max(
      0.0, -n_contact_a_.dot(wrench_filtered_a_.head<3>()));
    const double normal_error = f_normal_ref_active_ - f_normal_hat;

    normal_force_integral_ += normal_error * dt;
    normal_force_integral_ = std::clamp(
      normal_force_integral_,
      -normal_force_integral_limit_,
      normal_force_integral_limit_);
    input.head<3>() +=
      normal_force_integral_gain_ * normal_force_integral_ * n_contact_a_;

    for (std::size_t i = 0; i < kDof; ++i) {
      if (!axis_enabled_[i]) {
        input[i] = 0.0;
        q_[i] = 0.0;
        dq_[i] = 0.0;
        continue;
      }
      if (std::abs(input[i]) <= input_deadband_[i]) {
        input[i] = 0.0;
      }
      input[i] = std::clamp(input[i], -input_max_[i], input_max_[i]);
    }
    return input;
  }

  void integrateAdmittance(const Eigen::Matrix<double, 6, 1> & input, double dt)
  {
    for (std::size_t i = 0; i < kDof; ++i) {
      if (!axis_enabled_[i]) {
        q_[i] = 0.0;
        dq_[i] = 0.0;
        continue;
      }

      double ddq =
        (input[i] - adm_d_[i] * dq_[i] - adm_k_[i] * q_[i]) / adm_m_[i];
      ddq = std::clamp(ddq, -ddq_max_[i], ddq_max_[i]);

      dq_[i] += ddq * dt;
      dq_[i] = std::clamp(dq_[i], -dq_max_[i], dq_max_[i]);

      q_[i] += dq_[i] * dt;
      if (q_[i] >= q_max_[i]) {
        q_[i] = q_max_[i];
        if (dq_[i] > 0.0) {
          dq_[i] = 0.0;
        }
      } else if (q_[i] <= -q_max_[i]) {
        q_[i] = -q_max_[i];
        if (dq_[i] < 0.0) {
          dq_[i] = 0.0;
        }
      }
    }
  }

  void computeOutputReference(
    Eigen::Vector3d & pos_ref_world,
    Eigen::Vector3d & att_ref_rad) const
  {
    if (mode_ == Mode::PASSTHROUGH) {
      pos_ref_world = have_cmd_ ? upstream_pos_cmd_ : pos_;
      att_ref_rad = have_att_cmd_ ? upstream_att_cmd_rad_ : rpy_;
      return;
    }

    if (mode_ == Mode::HOLD) {
      pos_ref_world = hold_initialized_ ? hold_pos_world_ : pos_;
      att_ref_rad = hold_initialized_ ? hold_att_rad_ : rpy_;
      return;
    }

    const Eigen::Vector3d delta_position_a = q_.head<3>();
    const Eigen::Vector3d delta_rotation_a = q_.tail<3>();

    pos_ref_world = base_pos_world_ + r_wa_ * delta_position_a;

    // The virtual angular displacement is expressed in A. Convert it to WORLD
    // and left-compose it with the base command orientation.
    const Eigen::Vector3d delta_rotation_world = r_wa_ * delta_rotation_a;
    const Eigen::Matrix3d r_ref =
      rotationExp(delta_rotation_world) * base_rotation_world_;
    att_ref_rad = rpyFromRotation(r_ref);

    if (output_initialized_) {
      att_ref_rad.x() = unwrapNear(att_ref_rad.x(), last_att_ref_rad_.x());
      att_ref_rad.y() = unwrapNear(att_ref_rad.y(), last_att_ref_rad_.y());
      att_ref_rad.z() = unwrapNear(att_ref_rad.z(), last_att_ref_rad_.z());
    }
  }

  void updateAndPublish()
  {
    const rclcpp::Time t_now = now();
    double dt = (t_now - last_control_time_).seconds();
    last_control_time_ = t_now;
    if (!(dt > 0.0) || dt > 0.2) {
      dt = 1.0 / 400.0;
    }

    if (mode_ == Mode::ADMITTANCE) {
      f_normal_ref_active_ = moveToward(
        f_normal_ref_active_,
        f_normal_des_,
        f_ref_rate_max_ * dt);

      if (wrenchIsFresh(t_now)) {
        const Eigen::Matrix3d r_wb = rotationWorldFromBody(rpy_);
        updateFilteredWrench(dt, r_wb);
        const Eigen::Matrix<double, 6, 1> input = computeAdmittanceInput(dt);
        integrateAdmittance(input, dt);
      } else {
        // Never interpret a stale/missing estimator value as zero contact force.
        // Freeze the virtual velocity instead of driving toward the wall.
        dq_.setZero();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "external wrench missing/stale: admittance state frozen");
      }
    }

    Eigen::Vector3d pos_ref_world;
    Eigen::Vector3d att_ref_rad;
    computeOutputReference(pos_ref_world, att_ref_rad);

    minitrone_interfaces::msg::Cmd cmd_msg;
    cmd_msg.pos_cmd[0] = static_cast<float>(pos_ref_world.x());
    cmd_msg.pos_cmd[1] = static_cast<float>(pos_ref_world.y());
    cmd_msg.pos_cmd[2] = static_cast<float>(pos_ref_world.z());

    minitrone_interfaces::msg::AttitudeCmd att_msg;
    att_msg.roll_ref = static_cast<float>(att_ref_rad.x() * kRadToDeg);
    att_msg.pitch_ref = static_cast<float>(att_ref_rad.y() * kRadToDeg);
    att_msg.yaw_ref = static_cast<float>(att_ref_rad.z() * kRadToDeg);

    pub_cmd_->publish(cmd_msg);
    pub_att_cmd_->publish(att_msg);

    last_pos_ref_world_ = pos_ref_world;
    last_att_ref_rad_ = att_ref_rad;
    output_initialized_ = true;
    publishStatus();
  }

  void publishStatus()
  {
    std_msgs::msg::Bool active_msg;
    active_msg.data = mode_ == Mode::ADMITTANCE;
    pub_active_->publish(active_msg);

    std_msgs::msg::Float64 force_msg;
    force_msg.data = f_normal_des_;
    pub_des_force_->publish(force_msg);
  }

  void adjustDesiredForce(double delta)
  {
    const double old_force = f_normal_des_;
    f_normal_des_ = std::clamp(
      f_normal_des_ + delta,
      f_normal_min_,
      f_normal_max_);
    RCLCPP_INFO(
      get_logger(),
      "desired normal force %.2f -> %.2f N",
      old_force,
      f_normal_des_);
    publishStatus();
  }

  void setupKeyboard()
  {
    keyboard_enabled_ = isatty(STDIN_FILENO);
    if (!keyboard_enabled_) {
      RCLCPP_WARN(get_logger(), "stdin is not a TTY; keyboard disabled");
      return;
    }

    if (tcgetattr(STDIN_FILENO, &old_termios_) != 0) {
      keyboard_enabled_ = false;
      RCLCPP_WARN(get_logger(), "failed to read terminal settings");
      return;
    }

    termios raw = old_termios_;
    raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
      keyboard_enabled_ = false;
      RCLCPP_WARN(get_logger(), "failed to set terminal raw mode");
      return;
    }
    termios_configured_ = true;
  }

  void restoreKeyboard()
  {
    if (termios_configured_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
      termios_configured_ = false;
    }
  }

  void pollKeyboard()
  {
    if (!keyboard_enabled_) {
      return;
    }

    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(STDIN_FILENO, &read_fds);
    timeval timeout{0, 0};

    const int ready = select(STDIN_FILENO + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ready <= 0 || !FD_ISSET(STDIN_FILENO, &read_fds)) {
      return;
    }

    char key = '\0';
    if (read(STDIN_FILENO, &key, 1) != 1) {
      return;
    }

    if (key == 'I' || key == 'i') {
      requestEnabled(mode_ != Mode::ADMITTANCE);
    } else if (key == 'P' || key == 'p') {
      requestEnabled(false);
      enterPassthroughMode();
    } else if (key == 'U' || key == 'u') {
      adjustDesiredForce(+f_normal_step_);
    } else if (key == 'J' || key == 'j') {
      adjustDesiredForce(-f_normal_step_);
    }
  }

  // ROS interfaces
  rclcpp::Subscription<minitrone_interfaces::msg::Cmd>::SharedPtr sub_cmd_;
  rclcpp::Subscription<minitrone_interfaces::msg::AttitudeCmd>::SharedPtr sub_att_cmd_;
  rclcpp::Subscription<minitrone_interfaces::msg::MinitroneState>::SharedPtr sub_state_;
  rclcpp::Subscription<minitrone_interfaces::msg::Wrench>::SharedPtr sub_external_wrench_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_enable_;

  rclcpp::Publisher<minitrone_interfaces::msg::Cmd>::SharedPtr pub_cmd_;
  rclcpp::Publisher<minitrone_interfaces::msg::AttitudeCmd>::SharedPtr pub_att_cmd_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_active_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_des_force_;

  rclcpp::TimerBase::SharedPtr keyboard_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  // Input commands and state
  Eigen::Vector3d upstream_pos_cmd_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d upstream_att_cmd_rad_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rpy_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d w_body_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d external_force_body_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d external_moment_body_{Eigen::Vector3d::Zero()};

  // Admittance frame and base pose
  Eigen::Matrix3d r_wa_{Eigen::Matrix3d::Identity()};
  Eigen::Matrix3d base_rotation_world_{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d n_face_body_{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d n_contact_a_{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d base_pos_world_{Eigen::Vector3d::Zero()};

  // HOLD pose and last published reference
  Eigen::Vector3d hold_pos_world_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d hold_att_rad_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_pos_ref_world_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_att_ref_rad_{Eigen::Vector3d::Zero()};

  // 6-DOF admittance state: [x,y,z,roll,pitch,yaw] in A frame
  Eigen::Matrix<double, 6, 1> q_{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> dq_{Eigen::Matrix<double, 6, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> wrench_filtered_a_{
    Eigen::Matrix<double, 6, 1>::Zero()};

  std::array<bool, kDof> axis_enabled_{};
  std::array<double, kDof> adm_m_{};
  std::array<double, kDof> adm_d_{};
  std::array<double, kDof> adm_k_{};
  std::array<double, kDof> cutoff_hz_{};
  std::array<double, kDof> input_deadband_{};
  std::array<double, kDof> input_max_{};
  std::array<double, kDof> ddq_max_{};
  std::array<double, kDof> dq_max_{};
  std::array<double, kDof> q_max_{};

  double f_normal_des_{5.0};
  double f_normal_step_{0.1};
  double f_normal_min_{0.0};
  double f_normal_max_{20.0};
  double f_ref_rate_max_{1.0};
  double f_normal_ref_active_{0.0};
  double normal_force_integral_gain_{0.0};
  double normal_force_integral_limit_{1.0};
  double normal_force_integral_{0.0};
  double wrench_timeout_sec_{0.10};

  rclcpp::Time last_control_time_;
  rclcpp::Time last_wrench_time_;

  Mode mode_{Mode::PASSTHROUGH};
  bool pending_enable_{false};
  bool have_cmd_{false};
  bool have_att_cmd_{false};
  bool have_state_{false};
  bool have_external_wrench_{false};
  bool wrench_filter_initialized_{false};
  bool output_initialized_{false};
  bool hold_initialized_{false};
  bool keyboard_enabled_{false};
  bool termios_configured_{false};
  termios old_termios_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdmittanceController6Dof>());
  rclcpp::shutdown();
  return 0;
}
