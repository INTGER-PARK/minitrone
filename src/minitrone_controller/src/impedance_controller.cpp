#include <rclcpp/rclcpp.hpp>

#include <minitrone_interfaces/msg/attitude_cmd.hpp>
#include <minitrone_interfaces/msg/cmd.hpp>
#include <minitrone_interfaces/msg/minitrone_state.hpp>
#include <minitrone_interfaces/msg/wrench.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;

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

double moveToward(double current, double target, double max_step)
{
  const double step = std::max(0.0, max_step);
  if (current < target) {
    return std::min(current + step, target);
  }
  return std::max(current - step, target);
}
}  // namespace

class ImpedanceController : public rclcpp::Node
{
public:
  enum class Mode
  {
    NORMAL,
    IMPEDANCE
  };

  ImpedanceController()
  : rclcpp::Node("minitrone_impedance_controller")
  {
    // ========================================================================
    // 1. 접촉면 방향 설정
    // ========================================================================
    // n_face_body = [n_face_b_x, n_face_b_y, n_face_b_z]^T
    //
    // 드론 BODY 좌표계에서 "접촉면을 향하는 방향"을 정의하는 단위벡터이다.
    // admittance가 계산한 스칼라 변위 x_adm_delta_는 이 방향으로 위치 명령에
    // 더해진다. 또한 외력 벡터를 이 방향에 투영하여 normal force를 계산한다.
    //
    // 예시:
    //   드론 BODY +X 방향이 판을 향함 -> [1, 0, 0]
    //   드론 BODY -X 방향이 판을 향함 -> [-1, 0, 0]
    //   드론 BODY +Y 방향이 판을 향함 -> [0, 1, 0]
    //
    // 입력 벡터의 크기는 중요하지 않으며 아래에서 자동으로 normalize된다.
    // 모든 성분이 거의 0이면 안전하게 BODY +X 방향으로 대체한다.
    n_face_body_ << declare_parameter<double>("n_face_b_x", 1.0),
                    declare_parameter<double>("n_face_b_y", 0.0),
                    declare_parameter<double>("n_face_b_z", 0.0);
    if (n_face_body_.norm() < 1e-6) {
      n_face_body_ = Eigen::Vector3d::UnitX();
    }
    n_face_body_.normalize();

    // ========================================================================
    // 2. 목표 접촉력 관련 파라미터
    // ========================================================================

    // f_normal_des [N]
    // 최종적으로 유지하고 싶은 목표 normal force이다.
    // 예: 5.0이면 판을 약 5 N으로 계속 누르도록 위치 reference를 이동시킨다.
    // 값을 크게 하면 더 강하게 누르지만, 위치제어기 포화·기체 기울어짐·충격이
    // 커질 수 있으므로 낮은 값부터 올리는 것이 안전하다.
    f_normal_des_ = declare_parameter<double>("f_normal_des", 5.0);

    // f_normal_step [N/key]
    // 키보드 U/J를 한 번 누를 때 f_normal_des를 증가/감소시키는 크기이다.
    // U: +f_normal_step, J: -f_normal_step
    // 제어 응답 자체에는 직접 들어가지 않고 사용자가 목표 힘을 조절할 때만 쓴다.
    f_normal_step_ = declare_parameter<double>("f_normal_step", 0.1);

    // f_normal_min [N]
    // 사용자가 설정할 수 있는 목표 normal force의 하한이다.
    // 보통 음의 normal force를 사용하지 않으므로 0 N으로 둔다.
    f_normal_min_ = declare_parameter<double>("f_normal_min", 0.0);

    // f_normal_max [N]
    // 목표 normal force의 상한이다. U 키를 계속 눌러도 이 값을 넘지 않는다.
    // 하드웨어와 위치제어기의 안전 한계보다 충분히 낮게 설정해야 한다.
    f_normal_max_ = declare_parameter<double>("f_normal_max", 20.0);

    // f_ref_rate_max [N/s]
    // 실제 admittance에 넣는 활성 목표 힘 f_normal_ref_active가
    // f_normal_des까지 변하는 최대 속도이다.
    //
    // I를 눌러 mode를 켜거나 U/J로 목표 힘을 바꿔도 목표 힘이 계단처럼
    // 즉시 바뀌지 않고 이 속도로 ramp된다.
    //   작게 설정: 접촉력이 천천히 증가하여 부드럽지만 목표 도달이 느림
    //   크게 설정: 목표 도달은 빠르지만 접촉 충격과 위치 reference 변화가 커짐
    // 예: 1.0 N/s이면 0 N에서 5 N까지 약 5초가 걸린다.
    f_ref_rate_max_ = declare_parameter<double>("f_ref_rate_max", 1.0);  // [N/s]

    // ========================================================================
    // 3. 외력 추정값 및 force error 처리
    // ========================================================================

    // force_lpf_cutoff_hz [Hz]
    // 추정 normal force에 적용하는 1차 저역통과필터(LPF)의 차단주파수이다.
    //   작게 설정: 노이즈와 순간 충격을 강하게 제거하지만 힘 응답이 느려짐
    //   크게 설정: 실제 힘 변화를 빠르게 따라가지만 노이즈가 더 많이 통과함
    //   0 이하: 필터를 사용하지 않고 raw force를 그대로 사용
    // 너무 낮으면 접촉력이 늦게 반영되어 드론이 판 안쪽으로 더 밀고 들어갈 수 있다.
    force_lpf_cutoff_hz_ = declare_parameter<double>("force_lpf_cutoff_hz", 5.0);

    // force_error_deadband [N]
    // |목표 힘 - 추정 힘|이 이 값 이하이면 force error를 0으로 처리한다.
    // estimator 노이즈 때문에 위치 reference가 계속 미세하게 흔들리는 것을 막는다.
    //   크게 설정: 정지 상태는 안정적이지만 정상상태 힘 오차가 커질 수 있음
    //   작게 설정: 목표 힘을 정밀하게 추종하지만 채터링/미세 이동이 증가할 수 있음
    force_error_deadband_ = declare_parameter<double>("force_error_deadband", 0.10);

    // force_error_max [N]
    // admittance model에 입력되는 force error의 절댓값 상한이다.
    // 접촉이 순간적으로 사라져 추정 힘이 0이 되거나 estimator spike가 발생해도
    // 지나치게 큰 가상 가속도가 발생하지 않도록 한다.
    //   작게 설정: 접촉 복구가 부드럽지만 목표 힘 회복이 느림
    //   크게 설정: 회복은 빠르지만 급가속·재충돌 가능성이 커짐
    force_error_max_ = declare_parameter<double>("force_error_max", 1.50);

    // ========================================================================
    // 4. 선택적 force-error 적분항
    // ========================================================================

    // force_integral_gain
    // 적분된 force error를 추가적인 등가 힘으로 바꾸는 gain이다.
    // integral_force = force_integral_gain * integral(force_error dt)
    //
    // 판이 일정 속도로 움직이거나 모델 오차가 있을 때 남는 정상상태 force error를
    // 줄일 수 있지만, 값을 너무 크게 하면 overshoot와 접촉 진동이 생길 수 있다.
    // 기본 admittance 응답이 충분히 안정화되기 전에는 반드시 0.0을 권장한다.
    // 단위는 구현상 [1/s]에 해당한다.
    force_integral_gain_ = declare_parameter<double>("force_integral_gain", 0.0);

    // force_integral_limit [N*s]
    // force-error 적분 상태 force_error_integral_의 절댓값 상한이다.
    // 접촉이 장시간 약하거나 변위가 포화되었을 때 적분값이 무한히 쌓이는
    // integral windup을 제한한다.
    // 실제 추가 힘의 최대치는 대략
    //   force_integral_gain * force_integral_limit [N]
    // 이다.
    force_integral_limit_ = declare_parameter<double>("force_integral_limit", 1.0);

    // ========================================================================
    // 5. 가상 Mass-Spring-Damper admittance 파라미터
    // ========================================================================
    // 사용 식:
    //   M*x_ddot + D*x_dot + K*x
    //     = force_error + integral_force
    //
    // 여기서 x는 실제 드론 위치가 아니라 접촉 방향의 위치 reference 보정량
    // x_adm_delta_이다. M, D, K는 실제 기체 물성치가 아니라 사용자가 원하는
    // compliant motion을 만들기 위해 설정하는 "가상" 파라미터이다.

    // adm_mass [kg에 대응하는 가상 질량]
    // 같은 force error에 대해 reference 가속도가 얼마나 빠르게 변하는지 결정한다.
    //   크게 설정: 무겁고 둔하게 반응, 접촉 충격 감소, 힘 회복 느림
    //   작게 설정: 민감하고 빠르게 반응, 충격·진동 가능성 증가
    // 실제 계산에서는 0으로 나누는 것을 막기 위해 최소 1e-6으로 제한한다.
    adm_mass_ = declare_parameter<double>("adm_mass", 1.0);

    // adm_damping [N*s/m에 대응하는 가상 감쇠]
    // reference 속도 x_dot에 반대되는 힘 D*x_dot을 만든다.
    //   크게 설정: 움직임이 느리고 안정적이며 진동이 줄어듦
    //   작게 설정: 판의 움직임을 빠르게 따라가지만 overshoot/진동 가능성 증가
    // K=0인 현재 설정에서는 일정한 force error에 대한 정상상태 reference 속도가
    // 대략 x_dot = force_error / D가 된다.
    adm_damping_ = declare_parameter<double>("adm_damping", 20.0);

    // adm_stiffness [N/m에 대응하는 가상 강성]
    // mode 진입 시 기준 위치에서 멀어질수록 원래 위치로 돌아가려는 항 K*x를 만든다.
    //   0.0: 손으로 판을 움직이면 드론 reference가 계속 따라가고 새 위치에 남을 수 있음
    //   >0 : 스프링처럼 기준 위치로 돌아가려는 성질이 생김
    // 지속적인 접촉 추종이 목적이면 보통 0 또는 매우 작은 값부터 사용한다.
    adm_stiffness_ = declare_parameter<double>("adm_stiffness", 0.0);

    // ========================================================================
    // 6. 생성되는 위치 reference의 안전 제한
    // ========================================================================

    // x_ddot_max [m/s^2]
    // admittance 내부에서 생성되는 접촉 방향 reference 가속도의 절댓값 상한이다.
    // force error가 커도 위치 reference 속도가 갑자기 증가하지 못하게 한다.
    //   작게 설정: 접촉 복구가 매우 부드럽지만 느림
    //   크게 설정: force error 회복이 빠르지만 '팍 튀는' 현상이 증가할 수 있음
    x_ddot_max_ = declare_parameter<double>("x_ddot_max", 0.05);  // [m/s^2]

    // x_dot_max [m/s]
    // admittance 내부 접촉 방향 reference 속도의 절댓값 상한이다.
    // 접촉이 완전히 사라져 force error가 계속 양수여도 이 속도 이상으로
    // 판 방향을 향해 전진하지 않는다.
    // 0.015 m/s는 15 mm/s이다.
    x_dot_max_ = declare_parameter<double>("x_dot_max", 0.05);   // [m/s]

    // x_delta_max [m]
    // I를 눌러 impedance mode에 들어간 기준 위치로부터 접촉 방향으로 이동할 수 있는
    // 최대 위치 보정량의 절댓값이다. +방향과 -방향에 동일하게 적용된다.
    // 접촉 대상이 사라져도 드론이 무한히 이동하는 것을 막는 최종 안전 제한이다.
    // 예: 0.12 m이면 mode 진입 기준점에서 최대 ±12 cm 이동 가능하다.
    x_delta_max_ = declare_parameter<double>("x_delta_max", 5.0);  // [m]

    // pos_ref_rate_max [m/s]
    // 최종 publish되는 3차원 위치 reference 벡터 전체의 변화율 제한이다.
    // x_dot_max는 admittance 내부의 1차원 접촉방향 속도 제한이고,
    // pos_ref_rate_max는 그 결과를 publish하기 직전에 한 번 더 제한하는 보호층이다.
    //
    // 정상적으로는 pos_ref_rate_max >= x_dot_max로 두어 내부 admittance 동특성을
    // 지나치게 왜곡하지 않는 것이 좋다. 더 작게 두면 최종 명령은 부드러워지지만
    // 내부 x_adm_delta와 실제 publish reference 사이에 지연이 생긴다.
    pos_ref_rate_max_ = declare_parameter<double>("pos_ref_rate_max", 0.05);  // [m/s]

    if (f_normal_min_ > f_normal_max_) {
      std::swap(f_normal_min_, f_normal_max_);
    }

    f_normal_step_ = std::abs(f_normal_step_);
    f_normal_des_ = std::clamp(f_normal_des_, f_normal_min_, f_normal_max_);

    sub_cmd_ = create_subscription<minitrone_interfaces::msg::Cmd>(
      "/minitrone/cmd", 10,
      std::bind(&ImpedanceController::onCmd, this, std::placeholders::_1));

    sub_att_cmd_ = create_subscription<minitrone_interfaces::msg::AttitudeCmd>(
      "/minitrone/att_cmd", 10,
      std::bind(&ImpedanceController::onAttCmd, this, std::placeholders::_1));

    sub_state_ = create_subscription<minitrone_interfaces::msg::MinitroneState>(
      "/minitrone/state", 10,
      std::bind(&ImpedanceController::onState, this, std::placeholders::_1));

    sub_external_wrench_ = create_subscription<minitrone_interfaces::msg::Wrench>(
      "/minitrone/external_wrench_hat_second_order", 10,
      std::bind(&ImpedanceController::onExternalWrench, this, std::placeholders::_1));

    pub_cmd_ =
      create_publisher<minitrone_interfaces::msg::Cmd>("/minitrone/cmd_impedance", 10);
    pub_att_cmd_ = create_publisher<minitrone_interfaces::msg::AttitudeCmd>(
      "/minitrone/att_cmd_impedance", 10);
    pub_impedance_active_ =
      create_publisher<std_msgs::msg::Bool>("/minitrone/impedance_active", 10);
    pub_impedance_des_force_ =
      create_publisher<std_msgs::msg::Float64>("/minitrone/impedance_des_force", 10);

    setupKeyboard();
    keyboard_timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&ImpedanceController::pollKeyboard, this));

    last_time_ = now();
    publishImpedanceActive();
    publishImpedanceDesiredForce();

    RCLCPP_INFO(
      get_logger(),
      "Keyboard: I=toggle impedance, "
      "U=desired force +%.2f N, "
      "J=desired force -%.2f N "
      "(current %.2f N)",
      f_normal_step_,
      f_normal_step_,
      f_normal_des_);
  }

  ~ImpedanceController() override
  {
    restoreKeyboard();
  }

private:
  void onCmd(const minitrone_interfaces::msg::Cmd::SharedPtr msg)
  {
    pos_cmd_ << static_cast<double>(msg->pos_cmd[0]),
                static_cast<double>(msg->pos_cmd[1]),
                static_cast<double>(msg->pos_cmd[2]);
    have_cmd_ = true;
  }

  void onAttCmd(const minitrone_interfaces::msg::AttitudeCmd::SharedPtr msg)
  {
    att_cmd_msg_ = *msg;
    have_att_cmd_ = true;
  }

  void onExternalWrench(const minitrone_interfaces::msg::Wrench::SharedPtr msg)
  {
    external_force_body_ << static_cast<double>(msg->force[0]),
                            static_cast<double>(msg->force[1]),
                            static_cast<double>(msg->force[2]);
    have_external_wrench_ = true;
  }

  void resetAdmittanceState()
  {
    x_adm_delta_ = 0.0;
    x_adm_dot_ = 0.0;
    force_error_integral_ = 0.0;
    f_normal_ref_active_ = 0.0;
  }

  void enterImpedanceMode(const Eigen::Vector3d & n_face_world)
  {
    mode_ = Mode::IMPEDANCE;

    // Keep the contact direction fixed in the world frame during one impedance
    // episode. This is appropriate for contact with a fixed plate/wall and avoids
    // reference rotation caused by small attitude changes.
    n_contact_world_ = n_face_world;
    if (n_contact_world_.norm() < 1e-6) {
      n_contact_world_ = Eigen::Vector3d::UnitX();
    }
    n_contact_world_.normalize();

    // Bumpless mode entry: start at the last position reference that was actually
    // published instead of jumping to a separate approach/contact reference.
    p_impedance_base_world_ = pos_ref_initialized_ ? last_pos_ref_world_ : pos_;

    x_adm_delta_ = 0.0;
    x_adm_dot_ = 0.0;
    force_error_integral_ = 0.0;

    // Begin from the currently estimated force and ramp toward the desired force.
    f_normal_ref_active_ = std::clamp(
      f_normal_hat_,
      f_normal_min_,
      f_normal_des_);

    RCLCPP_INFO(
      get_logger(),
      "impedance mode ON: continuous force control, F_raw=%.3f N, F_lpf=%.3f N",
      f_normal_raw_,
      f_normal_hat_);
  }

  void setImpedanceArmed(bool armed)
  {
    if (impedance_armed_ == armed) {
      return;
    }

    impedance_armed_ = armed;

    if (impedance_armed_) {
      if (have_state_) {
        const Eigen::Matrix3d r_wb = rotationWorldFromBody(rpy_);
        enterImpedanceMode(r_wb * n_face_body_);
      } else {
        mode_ = Mode::NORMAL;
        resetAdmittanceState();
        RCLCPP_INFO(get_logger(), "impedance mode requested; waiting for state");
      }
    } else {
      mode_ = Mode::NORMAL;
      resetAdmittanceState();
      RCLCPP_INFO(get_logger(), "impedance mode OFF: NORMAL");
    }

    publishImpedanceActive();
  }

  void publishImpedanceActive()
  {
    std_msgs::msg::Bool msg;
    msg.data = impedance_armed_;
    pub_impedance_active_->publish(msg);
  }

  void publishImpedanceDesiredForce()
  {
    std_msgs::msg::Float64 msg;
    msg.data = f_normal_des_;
    pub_impedance_des_force_->publish(msg);
  }

  void toggleImpedance()
  {
    setImpedanceArmed(!impedance_armed_);
  }

  void adjustDesiredForce(double delta)
  {
    const double old_force = f_normal_des_;

    f_normal_des_ = std::clamp(
      f_normal_des_ + delta,
      f_normal_min_,
      f_normal_max_);

    if (std::abs(f_normal_des_ - old_force) > 1e-12) {
      RCLCPP_INFO(
        get_logger(),
        "desired normal force: %.2f -> %.2f N "
        "(filtered force: %.2f N, active reference: %.2f N)",
        old_force,
        f_normal_des_,
        f_normal_hat_,
        f_normal_ref_active_);
    } else {
      RCLCPP_WARN(
        get_logger(),
        "desired normal force remains %.2f N "
        "(limit [%.2f, %.2f] N)",
        f_normal_des_,
        f_normal_min_,
        f_normal_max_);
    }
    publishImpedanceDesiredForce();
  }

  void setupKeyboard()
  {
    keyboard_enabled_ = isatty(STDIN_FILENO);
    if (!keyboard_enabled_) {
      RCLCPP_WARN(
        get_logger(),
        "stdin is not a TTY; keyboard controls disabled. Run this node in a terminal.");
      return;
    }

    if (tcgetattr(STDIN_FILENO, &old_termios_) != 0) {
      keyboard_enabled_ = false;
      RCLCPP_WARN(get_logger(), "failed to read terminal settings; keyboard controls disabled");
      return;
    }

    termios raw = old_termios_;
    raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) {
      keyboard_enabled_ = false;
      RCLCPP_WARN(get_logger(), "failed to set terminal raw mode; keyboard controls disabled");
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
    timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    const int ready = select(STDIN_FILENO + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ready <= 0 || !FD_ISSET(STDIN_FILENO, &read_fds)) {
      return;
    }

    char key = '\0';
    if (read(STDIN_FILENO, &key, 1) != 1) {
      return;
    }

    if (key == 'I' || key == 'i') {
      toggleImpedance();
    } else if (key == 'U' || key == 'u') {
      adjustDesiredForce(+f_normal_step_);
    } else if (key == 'J' || key == 'j') {
      adjustDesiredForce(-f_normal_step_);
    }
  }

  void onState(const minitrone_interfaces::msg::MinitroneState::SharedPtr msg)
  {
    pos_ << static_cast<double>(msg->pos[0]),
            static_cast<double>(msg->pos[1]),
            static_cast<double>(msg->pos[2]);
    vel_ << static_cast<double>(msg->vel[0]),
            static_cast<double>(msg->vel[1]),
            static_cast<double>(msg->vel[2]);
    rpy_ << static_cast<double>(msg->rpy[0]),
            static_cast<double>(msg->rpy[1]),
            static_cast<double>(msg->rpy[2]);
    w_body_ << static_cast<double>(msg->w_rpy[0]),
               static_cast<double>(msg->w_rpy[1]),
               static_cast<double>(msg->w_rpy[2]);

    have_state_ = true;
    publishModifiedCommands();
  }

  Eigen::Vector3d limitPositionReference(
    const Eigen::Vector3d & target,
    double dt)
  {
    if (!pos_ref_initialized_) {
      last_pos_ref_world_ = target;
      pos_ref_initialized_ = true;
      return target;
    }

    // Do not alter the ordinary position command while impedance is disabled.
    if (!impedance_armed_) {
      last_pos_ref_world_ = target;
      return target;
    }

    const Eigen::Vector3d delta = target - last_pos_ref_world_;
    const double max_step = std::max(0.0, pos_ref_rate_max_) * dt;

    Eigen::Vector3d limited = target;
    if (max_step > 0.0 && delta.norm() > max_step) {
      limited = last_pos_ref_world_ + max_step * delta.normalized();
    }

    last_pos_ref_world_ = limited;
    return limited;
  }

  void publishModifiedCommands()
  {
    const rclcpp::Time t_now = now();
    double dt = (t_now - last_time_).seconds();
    last_time_ = t_now;
    if (!(dt > 0.0) || dt > 0.2) {
      dt = 1.0 / 400.0;
    }

    const Eigen::Matrix3d r_wb = rotationWorldFromBody(rpy_);
    const Eigen::Vector3d n_face_world = r_wb * n_face_body_;

    // The force estimator is updated on every state callback. While impedance
    // mode is active, use the world-fixed contact direction selected at mode entry.
    const Eigen::Vector3d & force_normal_world =
      (impedance_armed_ && mode_ == Mode::IMPEDANCE) ? n_contact_world_ : n_face_world;
    updateNormalForce(dt, r_wb, force_normal_world);

    const Eigen::Vector3d pos_ref_target = computePositionReference(dt, n_face_world);
    const Eigen::Vector3d pos_ref = limitPositionReference(pos_ref_target, dt);

    minitrone_interfaces::msg::Cmd cmd_msg;
    cmd_msg.pos_cmd[0] = static_cast<float>(pos_ref.x());
    cmd_msg.pos_cmd[1] = static_cast<float>(pos_ref.y());
    cmd_msg.pos_cmd[2] = static_cast<float>(pos_ref.z());

    pub_cmd_->publish(cmd_msg);
    pub_att_cmd_->publish(
      have_att_cmd_ ? att_cmd_msg_ : minitrone_interfaces::msg::AttitudeCmd());
    publishImpedanceActive();
    publishImpedanceDesiredForce();
  }

  double filteredAndLimitedForceError() const
  {
    // 부호 규약:
    //   force_error > 0 : 현재 힘이 목표보다 작음
    //                     -> +n_contact_world 방향으로 더 전진해야 함
    //   force_error < 0 : 현재 힘이 목표보다 큼
    //                     -> -n_contact_world 방향으로 물러나야 함
    double force_error = f_normal_ref_active_ - f_normal_hat_;

    if (std::abs(force_error) <= std::abs(force_error_deadband_)) {
      force_error = 0.0;
    }

    return std::clamp(
      force_error,
      -std::abs(force_error_max_),
      std::abs(force_error_max_));
  }

  void integrateAdmittance(double dt)
  {
    const double adm_mass = std::max(std::abs(adm_mass_), 1e-6);
    const double force_error = filteredAndLimitedForceError();
    const double delta_limit = std::abs(x_delta_max_);

    // There is intentionally no contact-present/contact-released condition here.
    // As long as I has enabled impedance mode, force error is processed every cycle,
    // even when the estimated normal force is zero.
    const bool pushing_beyond_positive_limit =
      x_adm_delta_ >= delta_limit && force_error > 0.0;
    const bool pushing_beyond_negative_limit =
      x_adm_delta_ <= -delta_limit && force_error < 0.0;

    // Simple integral anti-windup at the displacement limits.
    if (!pushing_beyond_positive_limit && !pushing_beyond_negative_limit) {
      force_error_integral_ += force_error * dt;
    }
    force_error_integral_ = std::clamp(
      force_error_integral_,
      -std::abs(force_integral_limit_),
      std::abs(force_integral_limit_));

    const double integral_force = force_integral_gain_ * force_error_integral_;

    // 가상 Mass-Spring-Damper 식을 x_ddot에 대해 정리한 형태:
    //   x_ddot = (e_F + F_I - D*x_dot - K*x) / M
    //
    // e_F가 양수이면 판 방향으로 reference를 가속하고,
    // D*x_dot은 움직임을 감쇠시키며, K*x는 기준점 복귀 성분을 만든다.
    double x_adm_ddot =
      (force_error + integral_force -
      adm_damping_ * x_adm_dot_ - adm_stiffness_ * x_adm_delta_) /
      adm_mass;

    x_adm_ddot = std::clamp(
      x_adm_ddot,
      -std::abs(x_ddot_max_),
      std::abs(x_ddot_max_));

    x_adm_dot_ += x_adm_ddot * dt;
    x_adm_dot_ = std::clamp(
      x_adm_dot_,
      -std::abs(x_dot_max_),
      std::abs(x_dot_max_));

    x_adm_delta_ += x_adm_dot_ * dt;

    // Position saturation with velocity anti-windup.
    if (x_adm_delta_ >= delta_limit) {
      x_adm_delta_ = delta_limit;
      if (x_adm_dot_ > 0.0) {
        x_adm_dot_ = 0.0;
      }
    } else if (x_adm_delta_ <= -delta_limit) {
      x_adm_delta_ = -delta_limit;
      if (x_adm_dot_ < 0.0) {
        x_adm_dot_ = 0.0;
      }
    }
  }

  Eigen::Vector3d computePositionReference(
    double dt,
    const Eigen::Vector3d & n_face_world)
  {
    if (!impedance_armed_) {
      mode_ = Mode::NORMAL;
      return have_cmd_ ? pos_cmd_ : pos_;
    }

    // This is reached when I was pressed before the first state message arrived.
    if (mode_ != Mode::IMPEDANCE) {
      enterImpedanceMode(n_face_world);
    }

    // Smoothly change the force target. No contact threshold is involved.
    f_normal_ref_active_ = moveToward(
      f_normal_ref_active_,
      f_normal_des_,
      std::abs(f_ref_rate_max_) * dt);

    // Always run force-error admittance while I-mode is ON.
    integrateAdmittance(dt);

    return p_impedance_base_world_ + x_adm_delta_ * n_contact_world_;
  }

  void updateNormalForce(
    double dt,
    const Eigen::Matrix3d & r_wb,
    const Eigen::Vector3d & force_normal_world)
  {
    if (!have_external_wrench_) {
      f_normal_raw_ = 0.0;
    } else {
      const Eigen::Vector3d external_force_world = r_wb * external_force_body_;
      f_normal_raw_ = std::max(
        0.0,
        -force_normal_world.dot(external_force_world));
    }

    if (!force_filter_initialized_) {
      f_normal_hat_ = f_normal_raw_;
      force_filter_initialized_ = true;
      return;
    }

    const double cutoff_hz = std::max(0.0, force_lpf_cutoff_hz_);
    if (cutoff_hz <= 0.0) {
      f_normal_hat_ = f_normal_raw_;
      return;
    }

    // 연속시간 1차 LPF를 샘플주기 dt에 맞게 이산화한 계수이다.
    // cutoff_hz가 커질수록 alpha가 커져 raw force를 더 빠르게 따라간다.
    const double alpha = 1.0 - std::exp(-2.0 * kPi * cutoff_hz * dt);
    f_normal_hat_ +=
      std::clamp(alpha, 0.0, 1.0) * (f_normal_raw_ - f_normal_hat_);
  }

  rclcpp::Subscription<minitrone_interfaces::msg::Cmd>::SharedPtr sub_cmd_;
  rclcpp::Subscription<minitrone_interfaces::msg::AttitudeCmd>::SharedPtr sub_att_cmd_;
  rclcpp::Subscription<minitrone_interfaces::msg::MinitroneState>::SharedPtr sub_state_;
  rclcpp::Subscription<minitrone_interfaces::msg::Wrench>::SharedPtr sub_external_wrench_;
  rclcpp::Publisher<minitrone_interfaces::msg::Cmd>::SharedPtr pub_cmd_;
  rclcpp::Publisher<minitrone_interfaces::msg::AttitudeCmd>::SharedPtr pub_att_cmd_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_impedance_active_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_impedance_des_force_;
  rclcpp::TimerBase::SharedPtr keyboard_timer_;

  rclcpp::Time last_time_;

  Eigen::Vector3d pos_cmd_{Eigen::Vector3d::Zero()};
  minitrone_interfaces::msg::AttitudeCmd att_cmd_msg_;
  Eigen::Vector3d pos_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d vel_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rpy_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d w_body_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d external_force_body_{Eigen::Vector3d::Zero()};

  // BODY frame에서 사용자가 지정한 접촉면 방향 단위벡터.
  Eigen::Vector3d n_face_body_{Eigen::Vector3d::UnitX()};

  // I를 눌러 mode에 들어간 순간의 접촉 방향을 WORLD frame에 고정한 단위벡터.
  // 작은 자세 변화로 force 투영 방향과 위치 이동 방향이 흔들리는 것을 막는다.
  Eigen::Vector3d n_contact_world_{Eigen::Vector3d::UnitX()};

  // Impedance mode 진입 시 기준 위치 [m]. 최종 명령은
  // p_impedance_base_world_ + x_adm_delta_ * n_contact_world_ 로 계산된다.
  Eigen::Vector3d p_impedance_base_world_{Eigen::Vector3d::Zero()};

  // 직전 제어 주기에 실제 publish한 위치 reference [m].
  // pos_ref_rate_max에 의한 slew-rate limit과 bumpless mode entry에 사용한다.
  Eigen::Vector3d last_pos_ref_world_{Eigen::Vector3d::Zero()};

  // 아래 값들은 생성자에서 declare_parameter() 반환값으로 덮어써진다.
  // 따라서 실제 기본값은 위 ROS2 parameter 선언부의 값을 기준으로 보면 된다.
  double f_normal_des_{5.0};             // 최종 목표 normal force [N]
  double f_normal_step_{0.1};            // U/J 1회 입력당 목표 힘 변화량 [N]
  double f_normal_min_{0.0};             // 목표 힘 하한 [N]
  double f_normal_max_{20.0};            // 목표 힘 상한 [N]
  double f_ref_rate_max_{1.0};           // 활성 목표 힘의 최대 변화율 [N/s]

  double force_lpf_cutoff_hz_{8.0};      // normal force LPF 차단주파수 [Hz]
  double force_error_deadband_{0.10};    // force error 무시 구간 [N]
  double force_error_max_{1.50};         // admittance 입력 force error 제한 [N]
  double force_integral_gain_{0.0};      // force error 적분 gain [1/s]
  double force_integral_limit_{1.0};     // force error 적분 상태 제한 [N*s]

  double adm_mass_{1.0};               // 가상 질량 M
  double adm_damping_{20.0};             // 가상 감쇠 D [N*s/m]
  double adm_stiffness_{0.0};            // 가상 강성 K [N/m]
  double x_ddot_max_{0.05};              // admittance reference 가속도 제한 [m/s^2]
  double x_dot_max_{0.05};               // admittance reference 속도 제한 [m/s]
  double x_delta_max_{5.00};               // mode 기준점 대비 최대 변위 [m]
  double pos_ref_rate_max_{0.05};         // 최종 3D position reference 변화율 [m/s]

  double x_adm_delta_{0.0};               // 접촉 방향 위치 보정량 x [m]
  double x_adm_dot_{0.0};                 // 접촉 방향 위치 reference 속도 [m/s]

  double f_normal_raw_{0.0};              // 투영 직후, 필터 전 normal force [N]
  double f_normal_hat_{0.0};              // LPF가 적용된 제어용 normal force [N]
  double f_normal_ref_active_{0.0};        // ramp가 적용된 현재 활성 목표 힘 [N]
  double force_error_integral_{0.0};       // force error 시간 적분값 [N*s]

  Mode mode_{Mode::NORMAL};
  bool impedance_armed_{false};
  bool have_cmd_{false};
  bool have_att_cmd_{false};
  bool have_state_{false};
  bool have_external_wrench_{false};
  bool force_filter_initialized_{false};
  bool pos_ref_initialized_{false};
  bool keyboard_enabled_{false};
  bool termios_configured_{false};
  termios old_termios_{};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImpedanceController>());
  rclcpp::shutdown();
  return 0;
}
