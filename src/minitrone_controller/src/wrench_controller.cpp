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

    // ===================== gains (기존 유지) =====================
    const double KP_POS[3] = {28.0, 28.0, 24.0};
    const double KI_POS[3] = {1.5, 1.5, 1.2};
    const double KD_POS[3] = {6.0, 6.0, 10.0};
    const double I_MIN_POS = -5.0, I_MAX_POS = 100.0, OUT_MIN_POS = -200.0, OUT_MAX_POS = 200.0;

    const double KP_ATT[3] = {6.00, 6.00, 6.00};
    const double KI_ATT[3] = {0.06, 0.06, 0.06};
    const double KD_ATT[3] = {0.80, 0.80, 0.80};
    const double I_MIN_ATT = -1.0, I_MAX_ATT = 1.0, OUT_MIN_ATT = -5.0, OUT_MAX_ATT = 5.0;

    auto init_pid =
      [](double kp, double ki, double kd,
         double i_min, double i_max,
         double out_min, double out_max)
      -> std::function<double(double,double,double,double)>
    {
      double iacc = 0.0;
      return [=](double ref, double cur, double dcur, double dt) mutable
      {
        if (dt <= 0.0) dt = 1e-3;
        const double e  = ref - cur;
        const double de = -dcur;
        iacc += ki * e * dt;
        iacc = std::clamp(iacc, i_min, i_max);
        double u = kp*e + iacc + kd*de;
        return std::clamp(u, out_min, out_max);
      };
    };

    pid_pos_[0] = init_pid(KP_POS[0], KI_POS[0], KD_POS[0], I_MIN_POS, I_MAX_POS, OUT_MIN_POS, OUT_MAX_POS);
    pid_pos_[1] = init_pid(KP_POS[1], KI_POS[1], KD_POS[1], I_MIN_POS, I_MAX_POS, OUT_MIN_POS, OUT_MAX_POS);
    pid_pos_[2] = init_pid(KP_POS[2], KI_POS[2], KD_POS[2], I_MIN_POS, I_MAX_POS, OUT_MIN_POS, OUT_MAX_POS);

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

    // allocator가 /wrench 또는 /wrench_cmd 중 무엇을 보든 안전하게
    pub_wrench_     = this->create_publisher<minitrone_interfaces::msg::Wrench>("/minitrone/wrench", 10);
    pub_wrench_cmd_ = this->create_publisher<minitrone_interfaces::msg::Wrench>("/minitrone/wrench_cmd", 10);

    pos_cmd_.setZero();
    att_cmd_.setZero();
    last_time_ = this->now();
  }

private:
  void onCmd(const minitrone_interfaces::msg::Cmd::SharedPtr msg)
  {
    pos_cmd_ << static_cast<double>(msg->pos_cmd[0]),
                static_cast<double>(msg->pos_cmd[1]),
                static_cast<double>(msg->pos_cmd[2]);
    have_cmd_ = true;
    tryPublish();
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
    tryPublish();
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
    admittance_active_ = msg->data;
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
    if (!(dt > 0.0) || dt > 0.2) dt = 1.0 / 400.0;

    // ===== (중요) 명령 없으면 "현재 위치 유지" =====
    // pos_ref = 0 으로 두면 초기 위치가 0이 아닐 때 순간적으로 큰 힘이 나가서 튕김.
    const bool use_admittance_cmd = admittance_active_ && have_admittance_cmd_;
    const Eigen::Vector3d pos_ref =
      use_admittance_cmd ? admittance_pos_cmd_ : (have_cmd_ ? pos_cmd_ : pos_);

    Eigen::Vector3d position_pid;
    position_pid.x() = pid_pos_[0](pos_ref.x(), pos_.x(), vel_.x(), dt);
    position_pid.y() = pid_pos_[1](pos_ref.y(), pos_.y(), vel_.y(), dt);
    position_pid.z() = pid_pos_[2](pos_ref.z(), pos_.z(), vel_.z(), dt);

    Eigen::Vector3d F_world;
    F_world.x() = position_pid.x();
    F_world.y() = position_pid.y();
    F_world.z() = position_pid.z() + mass_ * grav_;

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

    // ===== attitude reference =====
    const bool use_admittance_att_cmd = admittance_active_ && have_admittance_att_cmd_;
    Eigen::Vector3d att_ref =
      use_admittance_att_cmd ?
      admittance_att_cmd_ :
      (have_att_cmd_ ? att_cmd_ : Eigen::Vector3d::Zero());
    const double r_ref_d = att_ref.x();
    const double p_ref_d = att_ref.y();
    const double y_ref_d = att_ref.z();

    // yaw wrap-around
    const double y_err = std::atan2(std::sin(y_ref_d - rpy_.z()),
                                    std::cos(y_ref_d - rpy_.z()));
    const double y_ref_equiv = rpy_.z() + y_err;

    // attitude PID -> body moments
    Eigen::Vector3d M_body;
    M_body.x() = pid_att_[0](r_ref_d,     rpy_.x(), w_body_.x(), dt);
    M_body.y() = pid_att_[1](p_ref_d,     rpy_.y(), w_body_.y(), dt);
    M_body.z() = pid_att_[2](y_ref_equiv, rpy_.z(), w_body_.z(), dt);

    minitrone_interfaces::msg::Wrench w;
    w.moment[0] = static_cast<float>(M_body(0));
    w.moment[1] = static_cast<float>(M_body(1));
    w.moment[2] = static_cast<float>(M_body(2));
    w.force[0]  = static_cast<float>(F_body(0));
    w.force[1]  = static_cast<float>(F_body(1));
    w.force[2]  = static_cast<float>(F_body(2));

    // 두 토픽 모두 publish (pipe mismatch 방지)
    pub_wrench_->publish(w);
    pub_wrench_cmd_->publish(w);
  }

  // ROS
  rclcpp::Subscription<minitrone_interfaces::msg::Cmd>::SharedPtr            sub_cmd_;
  rclcpp::Subscription<minitrone_interfaces::msg::Cmd>::SharedPtr            sub_admittance_cmd_;
  rclcpp::Subscription<minitrone_interfaces::msg::MinitroneState>::SharedPtr sub_state_;
  rclcpp::Subscription<minitrone_interfaces::msg::AttitudeCmd>::SharedPtr     sub_att_cmd_;
  rclcpp::Subscription<minitrone_interfaces::msg::AttitudeCmd>::SharedPtr     sub_admittance_att_cmd_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                        sub_admittance_active_;
  rclcpp::Publisher<minitrone_interfaces::msg::Wrench>::SharedPtr            pub_wrench_;
  rclcpp::Publisher<minitrone_interfaces::msg::Wrench>::SharedPtr            pub_wrench_cmd_;

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

  std::function<double(double,double,double,double)> pid_pos_[3];
  std::function<double(double,double,double,double)> pid_att_[3];

  double mass_{1.0};
  double grav_{9.81};
  bool have_state_{false}, have_cmd_{false}, have_att_cmd_{false};
  bool have_admittance_cmd_{false}, have_admittance_att_cmd_{false};
  bool admittance_active_{false};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WrenchController>());
  rclcpp::shutdown();
  return 0;
}
