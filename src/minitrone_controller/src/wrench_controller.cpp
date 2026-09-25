#include <rclcpp/rclcpp.hpp>
#include <minitrone_interfaces/msg/cmd.hpp>
#include <minitrone_interfaces/msg/minitrone_state.hpp>
#include <minitrone_interfaces/msg/wrench.hpp>
#include <minitrone_interfaces/msg/attitude_cmd.hpp>
#include <std_msgs/msg/bool.hpp>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <functional>
#include <string>
#include <limits>
#include <stdexcept>

class WrenchController : public rclcpp::Node
{
public:
  static constexpr double deg_to_rad = M_PI / 180.0;

  WrenchController() : rclcpp::Node("minitrone_wrench_controller")
  {
    // ===================== (MINITRONE) params =====================
    // 
    this->declare_parameter<double>("mass", 2.5);
    this->declare_parameter<double>("gravity", 9.81);
    mass_ = this->get_parameter("mass").as_double();
    grav_ = this->get_parameter("gravity").as_double();

    if (!std::isfinite(mass_) || mass_ <= 0.0 || !std::isfinite(grav_)) {
      throw std::invalid_argument("mass must be positive and gravity must be finite");
    }

    // Legacy force gains define the equivalent default cascade tuning.
    const double KP_POS[3] = {28.0, 28.0, 24.0};
    const double KI_POS[3] = {1.5, 1.5, 1.2};
    const double KD_POS[3] = {6.0, 6.0, 10.0};
    const double I_MIN_POS = -5.0, I_MAX_POS = 100.0, OUT_MIN_POS = -200.0, OUT_MAX_POS = 200.0;

    const double KP_ATT[3] = {6.00, 6.00, 6.00};
    const double KI_ATT[3] = {0.00, 0.00, 0.00};
    const double KD_ATT[3] = {0.80, 0.80, 0.80};
    const double I_MIN_ATT = -1.0, I_MAX_ATT = 1.0, OUT_MIN_ATT = -5.0, OUT_MAX_ATT = 5.0;

    auto init_pid =
      [](double kp, double ki, double kd,
         double i_min, double i_max,
         double out_min, double out_max)
      -> std::function<double(double,double,double,double,bool)>
    {
      double iacc = 0.0;
      return [=](double ref, double cur, double dcur, double dt, bool reset) mutable
      {
        // [ADMITTANCE 연동 최소 수정 1]
        // Admittance ON/OFF 전환 시 이전 위치 PID에 누적된 적분값이 남아 있으면,
        // OFF 순간 현재 pose를 reference로 받아도 잔류 force/moment가 발생할 수 있다.
        // 따라서 mode 전환 직후 한 제어 주기 동안 각 PID의 적분 상태만 0으로 만든다.
        if (reset) iacc = 0.0;

        if (dt <= 0.0) dt = 1e-3;
        const double e  = ref - cur;
        const double de = -dcur;
        iacc += ki * e * dt;
        iacc = std::clamp(iacc, i_min, i_max);
        double u = kp*e + iacc + kd*de;
        return std::clamp(u, out_min, out_max);
      };
    };

    const char * axes[] = {"x", "y", "z"};
    auto gain = [this](const std::string & name, double value) {
      const double result = declare_parameter<double>(name, value);
      if (!std::isfinite(result) || result < 0.0) {
        throw std::invalid_argument(name + " must be finite and nonnegative");
      }
      return result;
    };
    for (int axis = 0; axis < 3; ++axis) {
      const std::string suffix = std::string("_") + axes[axis];
      // v_ref = (Kp_old / Kd_old) e + integral(Ki_old / Kd_old e).
      // a_ref = (Kd_old / mass) (v_ref - v_world).
      // Thus mass * a_ref reproduces the old force PID with default gains.
      const double pos_kp = gain("position_kp" + suffix, KP_POS[axis] / KD_POS[axis]);
      const double pos_ki = gain("position_ki" + suffix, KI_POS[axis] / KD_POS[axis]);
      const double pos_kd = gain("position_kd" + suffix, 0.0);
      const double vel_kp = gain("velocity_kp" + suffix, KD_POS[axis] / mass_);
      const double vel_ki = gain("velocity_ki" + suffix, 0.0);
      const double vel_kd = gain("velocity_kd" + suffix, 0.0);
      // Zero disables the optional speed limit to retain the legacy force law.
      const double speed_limit = gain("velocity_limit" + suffix, 0.0);
      const double vmax = speed_limit > 0.0 ? speed_limit :
        std::numeric_limits<double>::infinity();
      pid_pos_[axis] = init_pid(pos_kp, pos_ki, pos_kd,
        I_MIN_POS / KD_POS[axis], I_MAX_POS / KD_POS[axis], -vmax, vmax);
      pid_vel_[axis] = init_pid(vel_kp, vel_ki, vel_kd,
        I_MIN_POS / mass_, I_MAX_POS / mass_, OUT_MIN_POS / mass_, OUT_MAX_POS / mass_);
    }

    pid_att_[0] = init_pid(KP_ATT[0], KI_ATT[0], KD_ATT[0], I_MIN_ATT, I_MAX_ATT, OUT_MIN_ATT, OUT_MAX_ATT);
    pid_att_[1] = init_pid(KP_ATT[1], KI_ATT[1], KD_ATT[1], I_MIN_ATT, I_MAX_ATT, OUT_MIN_ATT, OUT_MAX_ATT);
    pid_att_[2] = init_pid(KP_ATT[2], KI_ATT[2], KD_ATT[2], I_MIN_ATT, I_MAX_ATT, OUT_MIN_ATT, OUT_MAX_ATT);

    const std::string cmd_topic =
      declare_parameter<std::string>("cmd_topic", "/minitrone/cmd");
    const std::string att_cmd_topic =
      declare_parameter<std::string>("att_cmd_topic", "/minitrone/att_cmd");
    const std::string admittance_cmd_topic =
      declare_parameter<std::string>("admittance_cmd_topic", "/minitrone/cmd_admittance");
    const std::string admittance_att_cmd_topic =
      declare_parameter<std::string>("admittance_att_cmd_topic", "/minitrone/att_cmd_admittance");
    const std::string admittance_active_topic =
      declare_parameter<std::string>("admittance_active_topic", "/minitrone/admittance_active");

    // ===================== ROS I/O =====================
    sub_cmd_ = this->create_subscription<minitrone_interfaces::msg::Cmd>(
      cmd_topic, 10, std::bind(&WrenchController::onCmd, this, std::placeholders::_1));

    sub_admittance_cmd_ = this->create_subscription<minitrone_interfaces::msg::Cmd>(
      admittance_cmd_topic, 10,
      std::bind(&WrenchController::onAdmittanceCmd, this, std::placeholders::_1));

    sub_state_ = this->create_subscription<minitrone_interfaces::msg::MinitroneState>(
      "/minitrone/state", 10, std::bind(&WrenchController::onState, this, std::placeholders::_1));

    sub_att_cmd_ = this->create_subscription<minitrone_interfaces::msg::AttitudeCmd>(
      att_cmd_topic, 10, std::bind(&WrenchController::onAttCmd, this, std::placeholders::_1));

    sub_admittance_att_cmd_ =
      this->create_subscription<minitrone_interfaces::msg::AttitudeCmd>(
      admittance_att_cmd_topic, 10,
      std::bind(&WrenchController::onAdmittanceAttCmd, this, std::placeholders::_1));

    sub_admittance_active_ = this->create_subscription<std_msgs::msg::Bool>(
      admittance_active_topic, 10,
      std::bind(&WrenchController::onAdmittanceActive, this, std::placeholders::_1));

    // The passive-aligning filter consumes the conventional controller output.
    pub_wrench_ = this->create_publisher<minitrone_interfaces::msg::Wrench>(
      "/minitrone/wrench_cmd", 10);
    pub_att_ref_ = this->create_publisher<minitrone_interfaces::msg::AttitudeCmd>(
      "/minitrone/att_ref", 10);

    pos_cmd_.setZero();
    att_cmd_.setZero();
    last_time_ = this->now();
  }

private:
  void onCmd(const minitrone_interfaces::msg::Cmd::SharedPtr msg)
  {
    const Eigen::Vector3d incoming_cmd(
      static_cast<double>(msg->pos_cmd[0]),
      static_cast<double>(msg->pos_cmd[1]),
      static_cast<double>(msg->pos_cmd[2]));
    if (waiting_for_position_sync_) {
      if ((incoming_cmd - pos_cmd_).norm() > position_sync_tolerance_) {
        return;
      }
      waiting_for_position_sync_ = false;
      RCLCPP_INFO(get_logger(), "position teleop synchronized after admittance OFF");
    }
    pos_cmd_ = incoming_cmd;
    have_cmd_ = true;
  }

  void onAdmittanceCmd(const minitrone_interfaces::msg::Cmd::SharedPtr msg)
  {
    admittance_pos_cmd_ << static_cast<double>(msg->pos_cmd[0]),
                          static_cast<double>(msg->pos_cmd[1]),
                          static_cast<double>(msg->pos_cmd[2]);
    have_admittance_cmd_ = true;
  }

  void onAttCmd(const minitrone_interfaces::msg::AttitudeCmd::SharedPtr msg)
  {
    // minitrone_cmd publishes attitude commands in degrees.
    att_cmd_ << static_cast<double>(msg->roll_ref) * deg_to_rad,
                static_cast<double>(msg->pitch_ref) * deg_to_rad,
                static_cast<double>(msg->yaw_ref) * deg_to_rad;
    have_att_cmd_ = true;
  }

  void onAdmittanceAttCmd(const minitrone_interfaces::msg::AttitudeCmd::SharedPtr msg)
  {
    admittance_att_cmd_ << static_cast<double>(msg->roll_ref) * deg_to_rad,
                          static_cast<double>(msg->pitch_ref) * deg_to_rad,
                          static_cast<double>(msg->yaw_ref) * deg_to_rad;
    have_admittance_att_cmd_ = true;
  }

  void onAdmittanceActive(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (admittance_active_ != msg->data) {
      const bool was_active = admittance_active_;
      admittance_active_ = msg->data;
      reset_pid_pending_ = true;
      if (was_active && !admittance_active_ && have_state_) {
        // Hand the measured pose to the conventional controller at the exact
        // OFF transition. Ignore the teleop's old 100 Hz command until the
        // teleop synchronizes itself to this captured position.
        pos_cmd_ = pos_;
        att_cmd_ = rpy_;
        have_cmd_ = true;
        have_att_cmd_ = true;
        waiting_for_position_sync_ = true;
      }
      RCLCPP_INFO(
        get_logger(),
        "admittance state changed: %s",
        admittance_active_ ? "ACTIVE" : "OFF (conventional HOLD)");
    }
  }

  void onState(const minitrone_interfaces::msg::MinitroneState::SharedPtr msg)
  {
    pos_    << static_cast<double>(msg->pos[0]),   static_cast<double>(msg->pos[1]),   static_cast<double>(msg->pos[2]);
    vel_    << static_cast<double>(msg->vel[0]),   static_cast<double>(msg->vel[1]),   static_cast<double>(msg->vel[2]);
    rpy_    << static_cast<double>(msg->rpy[0]),   static_cast<double>(msg->rpy[1]),   static_cast<double>(msg->rpy[2]);
    w_body_ << static_cast<double>(msg->w_rpy[0]), static_cast<double>(msg->w_rpy[1]), static_cast<double>(msg->w_rpy[2]);

    have_state_ = true;
    tryPublish();
  }

  void tryPublish()
  {
    if (!have_state_) return;

    const rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;
    const bool valid_dt = dt > 0.0 && dt <= 0.2;
    if (!valid_dt) dt = 1.0 / 400.0;

    const bool use_admittance =
      admittance_active_ && have_admittance_cmd_ && have_admittance_att_cmd_;
    const Eigen::Vector3d pos_ref =
      use_admittance ? admittance_pos_cmd_ : (have_cmd_ ? pos_cmd_ : pos_);

    // Mode 전환 직후 모든 PID 적분기를 같은 제어 주기에 한 번만 초기화한다.
    const bool reset_pid = reset_pid_pending_;
    reset_pid_pending_ = false;

    // Position and velocity are world-frame quantities. Differentiate measured
    // world velocity for the optional inner D term: state.acc is body-frame IMU
    // specific force, not world-frame kinematic acceleration. Reset derivative
    // history on mode changes, first sample, or a timing discontinuity.
    const Eigen::Vector3d acceleration_world =
      have_previous_velocity_ && valid_dt && !reset_pid ?
      Eigen::Vector3d((vel_ - previous_velocity_) / dt) : Eigen::Vector3d::Zero();
    previous_velocity_ = vel_;
    have_previous_velocity_ = true;

    Eigen::Vector3d velocity_ref_world;
    Eigen::Vector3d acceleration_ref_world;
    for (int axis = 0; axis < 3; ++axis) {
      velocity_ref_world[axis] = pid_pos_[axis](
        pos_ref[axis], pos_[axis], vel_[axis], dt, reset_pid);
      acceleration_ref_world[axis] = pid_vel_[axis](
        velocity_ref_world[axis], vel_[axis], acceleration_world[axis], dt, reset_pid);
    }

    Eigen::Vector3d F_world = mass_ * acceleration_ref_world;
    F_world.z() += mass_ * grav_;

    // R_WB from rpy
    const double r = rpy_.x(), p = rpy_.y(), y = rpy_.z();
    const double sr = std::sin(r), cr = std::cos(r);
    const double sp = std::sin(p), cp = std::cos(p);
    const double sy = std::sin(y), cy = std::cos(y);

    Eigen::Matrix3d R_WB;
    R_WB <<  cy*cp,  cy*sp*sr - sy*cr,  cy*sp*cr + sy*sr,
             sy*cp,  sy*sp*sr + cy*cr,  sy*sp*cr - cy*sr,
               -sp,              cp*sr,              cp*cr;

    const Eigen::Vector3d F_body = R_WB.transpose() * F_world;

    // The admittance topic carries an angular offset from its entry pose.
    // Publish the reference actually used by the attitude PID for debugging.
    const Eigen::Vector3d att_ref =
      (have_att_cmd_ ? att_cmd_ : Eigen::Vector3d::Zero()) +
      (use_admittance ? admittance_att_cmd_ : Eigen::Vector3d::Zero());
    minitrone_interfaces::msg::AttitudeCmd att_ref_msg;
    att_ref_msg.roll_ref = static_cast<float>(att_ref.x() / deg_to_rad);
    att_ref_msg.pitch_ref = static_cast<float>(att_ref.y() / deg_to_rad);
    att_ref_msg.yaw_ref = static_cast<float>(att_ref.z() / deg_to_rad);
    pub_att_ref_->publish(att_ref_msg);
    const double r_ref_d = att_ref.x();
    const double p_ref_d = att_ref.y();
    const double y_ref_d = att_ref.z();

    // yaw wrap-around
    const double y_err = std::atan2(std::sin(y_ref_d - rpy_.z()),
                                    std::cos(y_ref_d - rpy_.z()));
    const double y_ref_equiv = rpy_.z() + y_err;

    // attitude PID -> body moments
    Eigen::Vector3d M_body;
    M_body.x() = pid_att_[0](r_ref_d,     rpy_.x(), w_body_.x(), dt, reset_pid);
    M_body.y() = pid_att_[1](p_ref_d,     rpy_.y(), w_body_.y(), dt, reset_pid);
    M_body.z() = pid_att_[2](y_ref_equiv, rpy_.z(), w_body_.z(), dt, reset_pid);

    minitrone_interfaces::msg::Wrench w;
    w.moment[0] = static_cast<float>(M_body(0));
    w.moment[1] = static_cast<float>(M_body(1));
    w.moment[2] = static_cast<float>(M_body(2));
    w.force[0]  = static_cast<float>(F_body(0));
    w.force[1]  = static_cast<float>(F_body(1));
    w.force[2]  = static_cast<float>(F_body(2));

    // Control calculation and publishing are driven only by fresh state data.
    pub_wrench_->publish(w);
  }

  // ROS
  rclcpp::Subscription<minitrone_interfaces::msg::Cmd>::SharedPtr            sub_cmd_;
  rclcpp::Subscription<minitrone_interfaces::msg::Cmd>::SharedPtr            sub_admittance_cmd_;
  rclcpp::Subscription<minitrone_interfaces::msg::MinitroneState>::SharedPtr sub_state_;
  rclcpp::Subscription<minitrone_interfaces::msg::AttitudeCmd>::SharedPtr     sub_att_cmd_;
  rclcpp::Subscription<minitrone_interfaces::msg::AttitudeCmd>::SharedPtr     sub_admittance_att_cmd_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                        sub_admittance_active_;
  rclcpp::Publisher<minitrone_interfaces::msg::Wrench>::SharedPtr            pub_wrench_;
  rclcpp::Publisher<minitrone_interfaces::msg::AttitudeCmd>::SharedPtr       pub_att_ref_;

  rclcpp::Time last_time_;

  // state/command
  Eigen::Vector3d pos_cmd_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d admittance_pos_cmd_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rpy_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d w_body_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d att_cmd_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d admittance_att_cmd_{Eigen::Vector3d::Zero()};

  std::function<double(double,double,double,double,bool)> pid_pos_[3];
  std::function<double(double,double,double,double,bool)> pid_vel_[3];
  std::function<double(double,double,double,double,bool)> pid_att_[3];
  Eigen::Vector3d previous_velocity_{Eigen::Vector3d::Zero()};
  bool have_previous_velocity_{false};

  double mass_{2.5};
  double grav_{9.81};
  bool have_state_{false}, have_cmd_{false}, have_att_cmd_{false};
  bool have_admittance_cmd_{false}, have_admittance_att_cmd_{false};
  bool admittance_active_{false};
  bool waiting_for_position_sync_{false};
  const double position_sync_tolerance_{0.05};

  // Admittance ACTIVE <-> HOLD 전환 직후 PID 적분기를 한 번 초기화하기 위한 flag.
  bool reset_pid_pending_{false};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WrenchController>());
  rclcpp::shutdown();
  return 0;
}
