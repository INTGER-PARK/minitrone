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

/*
 * Passive Aligning Controller
 * ===========================
 *
 * 목적
 * ----
 * 접촉면에 수직인 방향(normal direction)에서는 원하는 접촉력을 유지하고,
 * 접촉면의 기울기 오차를 만드는 회전축에서는 자세 제어 모멘트를 약화시켜
 * 드론의 접촉면이 대상 표면에 수동적으로 정렬되도록 한다.
 *
 * 예: 접촉면 법선이 body +x 방향인 경우
 *   - 병진력 Fx : 목표 접촉력 추종
 *   - 모멘트 Mx : 법선축 회전(roll에 해당)은 기존 제어를 유지
 *   - 모멘트 My, Mz : pitch/yaw 제어를 약화하거나 제거하여 passive alignment 허용
 *
 * 축 분리
 * -------
 * 단위 법선벡터를 n이라 하면,
 *
 *   Pn = n n^T      : 법선축 투영행렬
 *   Pt = I - Pn     : 접선 평면 투영행렬
 *
 * 벡터 v는 다음과 같이 분해된다.
 *
 *   v_normal     = Pn v
 *   v_tangential = Pt v
 *
 * 최종 힘 명령
 * ------------
 *   F_hybrid = Pt F_input + F_normal_cmd n
 *
 * 즉, 기존 wrench controller의 접선 방향 힘은 그대로 통과시키고,
 * 법선 방향 힘만 별도의 PI 접촉력 제어기로 교체한다.
 *
 * 최종 모멘트 명령
 * ----------------
 *   M_hybrid = Pn M_input
 *            + passive_scale Pt M_input
 *            - D Pt omega
 *
 * passive_scale = 0이면 정렬을 방해하는 접선축 자세 제어 모멘트를 제거한다.
 * 작은 damping은 회전 자유도를 완전히 무감쇠로 두지 않고 진동만 억제한다.
 *
 * 주의
 * ----
 * 이 코드는 실제 기계식 passive joint가 아니라, 특정 회전축의 능동 자세 제어를
 * 약화시켜 외력에 순응하도록 만드는 제어 기반의 passive-like alignment이다.
 */

namespace
{
constexpr double kPi = 3.14159265358979323846;

/**
 * @brief current 값을 한 번에 max_step 이하만큼 target 방향으로 이동시킨다.
 *
 * enable/disable 전환이나 목표 접촉력 변경 시 명령이 불연속적으로 튀는 것을
 * 방지하기 위한 slew-rate limiter로 사용된다.
 */
double moveToward(double current, double target, double max_step)
{
  // 음수 step이 들어와도 실제 이동량은 0 이상이 되도록 제한한다.
  const double step = std::max(0.0, max_step);

  // target을 초과하지 않는 범위에서 current를 증가 또는 감소시킨다.
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
    // -----------------------------------------------------------------------
    // 1. 입출력 wrench 토픽 설정
    // -----------------------------------------------------------------------
    // 기존 wrench controller가 계산한 명령을 입력받는다.
    const auto input_topic = declare_parameter<std::string>(
      "input_wrench_topic", "/minitrone/wrench_cmd");

    // passive-align 처리가 끝난 최종 wrench를 출력.
    const auto output_topic = declare_parameter<std::string>(
      "output_wrench_topic", "/minitrone/wrench_passive_align");

    // -----------------------------------------------------------------------
    // 2. Passive-align 활성화 여부 및 접촉면 법선 설정
    // -----------------------------------------------------------------------
    enabled_ = declare_parameter<bool>("enabled", false);

    // 접촉면의 body-frame 단위 법선벡터 n.
    // 기본값 [1, 0, 0]^T는 접촉판이 body +x 방향을 향한다는 의미이다.
    normal_ << declare_parameter<double>("n_face_b_x", 1.0),
      declare_parameter<double>("n_face_b_y", 0.0),
      declare_parameter<double>("n_face_b_z", 0.0);

    // 잘못된 영벡터가 들어오면 안전하게 body +x를 사용한다.
    if (normal_.norm() < 1e-6) {
      normal_ = Eigen::Vector3d::UnitX();
    }

    // Pn = n n^T가 올바른 투영행렬이 되도록 반드시 단위벡터로 정규화한다.
    normal_.normalize();

    // -----------------------------------------------------------------------
    // 3. 법선 방향 접촉력 제어 파라미터
    // -----------------------------------------------------------------------
    mass_ = std::max(0.0, declare_parameter<double>("mass", 2.5));
    gravity_ = std::max(0.0, declare_parameter<double>("gravity", 9.81));

    // 사용자가 요구하는 정상상태 법선 접촉력 [N].
    f_des_ = declare_parameter<double>("f_normal_des", 5.0);

    // 키보드 U/J 입력 한 번당 목표 접촉력 증감량 [N].
    f_step_ = std::abs(declare_parameter<double>("f_normal_step", 0.1));

    // 사용자가 설정할 수 있는 목표 접촉력의 범위 [N].
    f_min_ = declare_parameter<double>("f_normal_min", 0.0);
    f_max_ = declare_parameter<double>("f_normal_max", 20.0);
    if (f_min_ > f_max_) std::swap(f_min_, f_max_);
    f_des_ = std::clamp(f_des_, f_min_, f_max_);

    // 실제 내부 목표 f_ref가 f_des를 따라가는 최대 변화율 [N/s].
    // 접촉력 명령이 갑자기 증가하여 충격이 발생하는 것을 줄인다.
    f_rate_ = std::abs(declare_parameter<double>("f_ref_rate_max", 1.0));

    // 법선 접촉력 PI 제어기 이득.
    force_kp_ = std::max(0.0, declare_parameter<double>("force_kp", 0.5));
    force_ki_ = std::max(0.0, declare_parameter<double>("force_ki", 0.0));

    // 적분 wind-up을 방지하는 적분 상태 제한값.
    integral_limit_ = std::abs(
      declare_parameter<double>("force_integral_limit", 2.0));

    // 법선 방향으로 생성할 최종 force command의 최소/최대값 [N].
    force_cmd_min_ = declare_parameter<double>("force_cmd_min", 0.0);
    force_cmd_max_ = declare_parameter<double>("force_cmd_max", 20.0);
    if (force_cmd_min_ > force_cmd_max_) {
      std::swap(force_cmd_min_, force_cmd_max_);
    }

    // 외력 추정값에 적용할 1차 저역통과필터 차단주파수 [Hz].
    cutoff_hz_ = std::max(
      0.0, declare_parameter<double>("force_lpf_cutoff_hz", 20.0));

    // 외력 추정 메시지가 이 시간보다 오래되면 stale로 판단한다 [s].
    // 0 이하이면 시간 제한을 사용하지 않는다.
    timeout_sec_ = std::max(
      0.0, declare_parameter<double>("wrench_timeout_sec", 0.1));

    // -----------------------------------------------------------------------
    // 4. Passive 회전축 제어 파라미터
    // -----------------------------------------------------------------------
    // 접선 회전축에 기존 자세제어 모멘트를 얼마나 남길지 결정한다.
    //   0.0 : 기존 접선축 모멘트를 완전히 제거 → 가장 compliant
    //   1.0 : 기존 모멘트를 그대로 유지       → passive alignment 없음
    passive_scale_ = std::clamp(
      declare_parameter<double>("passive_axis_control_scale", 0.0), 0.0, 1.0);

    // body roll/pitch/yaw 각속도에 대한 감쇠 계수.
    // 기본 법선이 +x이면 roll은 법선축이므로 유지되고,
    // pitch/yaw 방향 감쇠가 접촉면 정렬 중 진동을 억제한다.
    const double damping_roll =
      std::max(0.0, declare_parameter<double>("passive_damping_roll", 0.0));
    const double damping_pitch =
      std::max(0.0, declare_parameter<double>("passive_damping_pitch", 0.2));
    const double damping_yaw =
      std::max(0.0, declare_parameter<double>("passive_damping_yaw", 0.2));
    damping_ << damping_roll,
      std::max(0.0, declare_parameter<double>("passive_damping_y", damping_pitch)),
      std::max(0.0, declare_parameter<double>("passive_damping_z", damping_yaw));

    // damping으로 만들어지는 각 축 모멘트의 절댓값 제한 [N·m].
    moment_limit_ = std::abs(
      declare_parameter<double>("passive_moment_limit", 1.0));

    // passive-align ON/OFF 시 원래 wrench와 수정 wrench 사이를
    // 선형적으로 전환하는 데 걸리는 시간 [s].
    const double legacy_transition_sec = std::max(
      0.0, declare_parameter<double>("transition_time_sec", 0.5));
    transition_sec_ = std::max(
      0.0, declare_parameter<double>(
        "handoff_blend_time", legacy_transition_sec));

    // -----------------------------------------------------------------------
    // 5. ROS 2 subscriber
    // -----------------------------------------------------------------------
    // 기존 controller가 생성한 nominal wrench command.
    // 이 메시지가 들어올 때마다 onInput()에서 passive-align 처리를 수행한다.
    input_sub_ = create_subscription<minitrone_interfaces::msg::Wrench>(
      input_topic, 10,
      std::bind(
        &PassiveAligningController::onInput, this, std::placeholders::_1));

    // 드론의 body roll/pitch/yaw 각속도 [rad/s]를 받아 감쇠 모멘트 계산에 사용한다.
    state_sub_ = create_subscription<minitrone_interfaces::msg::MinitroneState>(
      "/minitrone/state", 10,
      [this](const minitrone_interfaces::msg::MinitroneState::SharedPtr msg) {
        rpy_ << msg->rpy[0], msg->rpy[1], msg->rpy[2];
        omega_ << msg->w_rpy[0], msg->w_rpy[1], msg->w_rpy[2];
      });

    // 외력 관측기에서 추정한 external wrench를 입력받는다.
    // 본 노드에서는 force 성분만 사용하여 실제 법선 접촉력을 계산한다.
    external_sub_ = create_subscription<minitrone_interfaces::msg::Wrench>(
      "/minitrone/external_wrench_hat_second_order", 10,
      [this](const minitrone_interfaces::msg::Wrench::SharedPtr msg) {
        external_force_ << msg->force[0], msg->force[1], msg->force[2];
        have_external_ = true;
        last_external_time_ = now();
      });

    // 외부 ROS 토픽을 이용한 passive-align ON/OFF 명령.
    enable_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/minitrone/passive_align_enable", 10,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        setEnabled(msg->data);
      });

    // -----------------------------------------------------------------------
    // 6. ROS 2 publisher
    // -----------------------------------------------------------------------
    // passive-align 처리를 거친 최종 wrench.
    output_pub_ = create_publisher<minitrone_interfaces::msg::Wrench>(
      output_topic, 10);

    // 현재 passive-align 활성화 상태.
    active_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/minitrone/passive_align_active", 10);

    // 사용자가 설정한 최종 목표 접촉력 f_des.
    desired_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/minitrone/passive_align_des_force", 10);

    // 저역통과필터를 거친 측정 법선 접촉력.
    measured_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/minitrone/passive_align_measured_force", 10);
    force_measured_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/minitrone/passive_align/force_measured", 10);

    // Debug signals: final force target, current ramped force reference,
    // passive blend, and pitch command before/after the selection matrix.
    force_target_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/minitrone/passive_align/force_target", 10);
    force_ref_ramped_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/minitrone/passive_align/force_ref_ramped", 10);
    blend_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/minitrone/passive_align/blend", 10);
    normal_force_command_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/minitrone/passive_align/normal_force_command", 10);
    normal_force_unsaturated_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/minitrone/passive_align/normal_force_command_unsaturated", 10);
    gravity_normal_compensation_pub_ =
      create_publisher<std_msgs::msg::Float64>(
      "/minitrone/passive_align/gravity_normal_compensation", 10);
    force_command_saturated_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/minitrone/passive_align/force_command_saturated", 10);
    my_before_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/contact_method1/debug/my_cmd_before_selection", 10);
    my_after_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/contact_method1/debug/my_cmd_after_selection", 10);
    my_damping_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/contact_method1/debug/my_cmd_damping", 10);

    // 실행 중 ros2 param set으로 일부 파라미터를 변경할 수 있도록 콜백 등록.
    parameter_handle_ = add_on_set_parameters_callback(
      std::bind(
        &PassiveAligningController::onParameters,
        this,
        std::placeholders::_1));

    // 터미널 키를 Enter 없이 즉시 읽을 수 있도록 설정한다.
    setupKeyboard();

    // 20 ms마다 키보드 입력을 확인한다. 즉, 50 Hz polling이다.
    keyboard_timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&PassiveAligningController::pollKeyboard, this));

    // 첫 wrench callback의 dt 계산을 위한 초기 시각.
    last_output_time_ = now();

    publishStatus();
    RCLCPP_INFO(
      get_logger(), "Passive aligning ready. O=ON/OFF, U/J=force +/-");
  }

  // 노드 종료 시 터미널 설정을 원래 상태로 복원한다.
  ~PassiveAligningController() override
  {
    restoreKeyboard();
  }

private:
  /**
   * @brief nominal wrench에 passive-align 처리를 적용한다.
   *
   * 처리 순서
   * 1) 샘플링 시간 dt 계산
   * 2) enable 전환 blend 및 목표 접촉력 f_ref ramp 적용
   * 3) 힘/모멘트를 법선축과 접선축으로 분해
   * 4) 법선 힘 PI 제어
   * 5) 접선 회전축 자세 모멘트 약화 + 각속도 감쇠
   * 6) 원래 명령과 수정 명령을 blend하여 publish
   */
  void onInput(const minitrone_interfaces::msg::Wrench::SharedPtr msg)
  {
    // -----------------------------------------------------------------------
    // 1. 현재 callback의 샘플링 시간 계산
    // -----------------------------------------------------------------------
    const auto t = now();
    double dt = (t - last_output_time_).seconds();
    last_output_time_ = t;

    // 첫 실행, 시간 역행 또는 긴 callback 지연이 발생하면
    // nominal control rate 400 Hz에 해당하는 dt를 사용한다.
    if (!(dt > 0.0) || dt > 0.2) {
      dt = 1.0 / 400.0;
    }

    // -----------------------------------------------------------------------
    // 2. Passive-align 명령의 부드러운 ON/OFF 전환
    // -----------------------------------------------------------------------
    const double blend_target = enabled_ ? 1.0 : 0.0;

    // blend = 0 : 원래 nominal wrench만 사용
    // blend = 1 : passive-align wrench를 완전히 사용
    // transition_sec 동안 0↔1로 서서히 변화시켜 wrench 불연속을 줄인다.
    blend_ = transition_sec_ <= 1e-6 ? blend_target :
      moveToward(blend_, blend_target, dt / transition_sec_);

    // 실제 force-controller가 사용하는 내부 목표 f_ref도 f_rate [N/s]로 제한한다.
    // OFF 상태에서는 0 N을 향하고, ON 상태에서는 f_des를 향한다.
    f_ref_ = moveToward(
      f_ref_, enabled_ ? f_des_ : 0.0, f_rate_ * dt);

    // 입력 nominal wrench를 Eigen 벡터로 변환한다.
    Eigen::Vector3d force(
      msg->force[0], msg->force[1], msg->force[2]);
    Eigen::Vector3d moment(
      msg->moment[0], msg->moment[1], msg->moment[2]);

    // -----------------------------------------------------------------------
    // 3. 법선축/접선축 투영행렬 구성
    // -----------------------------------------------------------------------
    // Pn = n n^T
    // 임의 벡터 v에 Pn을 곱하면 접촉면 법선 방향 성분만 남는다.
    const Eigen::Matrix3d pn = normal_ * normal_.transpose();

    // Pt = I - Pn
    // 임의 벡터 v에 Pt를 곱하면 접촉면에 평행한 성분만 남는다.
    const Eigen::Matrix3d pt = Eigen::Matrix3d::Identity() - pn;

    // 외력 정보가 없을 때는 원래 force를 그대로 사용하기 위해 초기화한다.
    Eigen::Vector3d hybrid_force = force;
    double normal_cmd = normal_.dot(force);
    double unsaturated_cmd = normal_cmd;
    bool force_command_saturated = false;
    const double roll = rpy_.x();
    const double pitch = rpy_.y();
    const Eigen::Vector3d world_up_in_body(
      -std::sin(pitch),
      std::cos(pitch) * std::sin(roll),
      std::cos(pitch) * std::cos(roll));
    const double gravity_normal_compensation =
      mass_ * gravity_ * normal_.dot(world_up_in_body);

    // -----------------------------------------------------------------------
    // 4. 법선 방향 접촉력 PI 제어
    // -----------------------------------------------------------------------
    // 외력 데이터가 한 번 이상 수신되었고 최신 데이터인지 확인한다.
    const bool external_fresh = have_external_ &&
      (timeout_sec_ <= 0.0 ||
      (t - last_external_time_).seconds() <= timeout_sec_);

    if (external_fresh) {
      /*
       * 외력 부호 규약
       * -----------
       * 접촉 중 환경이 드론을 -n 방향으로 미는 외력을 준다고 가정한다.
       * 따라서 실제 압축 접촉력 크기는
       *
       *   F_meas = max(0, -n^T F_ext)
       *
       * 로 계산한다.
       *
       * 외력 관측기의 부호 정의가 반대라면 여기의 '-' 부호도 바꿔야 한다.
       */
      const double measured = std::max(
        0.0, -normal_.dot(external_force_));

      // 필터 첫 샘플에서는 초기 과도응답을 피하기 위해 측정값으로 바로 초기화한다.
      if (!filter_initialized_) {
        filtered_force_ = measured;
        filter_initialized_ = true;
      } else {
        /*
         * 연속시간 1차 LPF를 샘플링 시간 dt에 맞춰 이산화한 계수:
         *
         *   alpha = 1 - exp(-2 pi fc dt)
         *   F_f[k] = F_f[k-1] + alpha(F_meas[k] - F_f[k-1])
         *
         * cutoff_hz <= 0이면 alpha=1이므로 필터 없이 측정값을 바로 사용한다.
         */
        const double alpha = cutoff_hz_ <= 0.0 ? 1.0 : std::clamp(
          1.0 - std::exp(-2.0 * kPi * cutoff_hz_ * dt), 0.0, 1.0);

        filtered_force_ += alpha * (measured - filtered_force_);
      }

      // 접촉력 오차: 양수이면 현재 힘이 부족하므로 더 밀어야 한다.
      const double error = f_ref_ - filtered_force_;

      // PI 제어기의 적분항. 지정 범위로 제한하여 wind-up을 방지한다.
      const double integral_candidate = std::clamp(
        force_integral_ + error * dt,
        -integral_limit_,
        integral_limit_);

      /*
       * 법선 방향 force command:
       *
       *   F_normal_cmd = n^T F_gravity_comp
       *                + f_ref + Kp(f_ref - F_meas)
       *                        + Ki integral(f_ref - F_meas)
       *
       * The normal position-control term is deliberately removed, but gravity
       * compensation must remain. At nonzero pitch, replacing the entire
       * nominal normal force with f_ref otherwise adds the body-normal gravity
       * projection to the requested contact load (the observed 7.64 N issue).
       */
      unsaturated_cmd =
        gravity_normal_compensation +
        f_ref_ + force_kp_ * error + force_ki_ * integral_candidate;
      normal_cmd = std::clamp(
        unsaturated_cmd,
        force_cmd_min_,
        force_cmd_max_);
      force_command_saturated =
        std::abs(normal_cmd - unsaturated_cmd) > 1e-9;
      // Conditional integration: accept the candidate unless saturation and
      // the current error would drive the command farther into saturation.
      if (unsaturated_cmd == normal_cmd ||
        (unsaturated_cmd > force_cmd_max_ && error < 0.0) ||
        (unsaturated_cmd < force_cmd_min_ && error > 0.0))
      {
        force_integral_ = integral_candidate;
      }

      /*
       * Hybrid force 구성:
       *
       *   F_hybrid = Pt F_nominal + F_normal_cmd n
       *
       * - 접선 방향 힘: 기존 위치/운동 controller의 명령 유지
       * - 법선 방향 힘: 접촉력 controller의 명령으로 교체
       */
      hybrid_force = pt * force + normal_cmd * normal_;
    } else if (enabled_) {
      // 외력 데이터가 오래되면 잘못된 force feedback을 사용하지 않는다.
      // 이때 hybrid_force는 초기값인 nominal force 그대로 유지된다.
      force_integral_ = 0.0;

      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "external wrench stale: upstream normal force is passed through");
    }

    // -----------------------------------------------------------------------
    // 5. Passive alignment를 위한 모멘트 수정
    // -----------------------------------------------------------------------
    /*
     * 접선 회전축 각속도만 추출한 뒤 점성 감쇠 모멘트를 계산한다.
     *
     *   M_damping = -D Pt omega
     *
     * 기본 n=[1,0,0]이면 Pt omega = [0, wy, wz]^T이므로
     * pitch/yaw 각속도에 반대되는 작은 모멘트만 생성된다.
     */
    Eigen::Vector3d damping =
      -(damping_.asDiagonal() * (pt * omega_));

    // D가 body축 대각행렬이므로 계산 후 다시 Pt로 투영하여
    // 법선축 방향 감쇠 모멘트가 섞이지 않도록 한다.
    damping = pt * damping;

    // 비정상적으로 큰 각속도에서도 damping moment가 제한값을 넘지 않게 한다.
    for (int i = 0; i < 3; ++i) {
      damping[i] = std::clamp(
        damping[i], -moment_limit_, moment_limit_);
    }

    /*
     * Hybrid moment 구성:
     *
     *   M_hybrid = Pn M_nominal
     *            + passive_scale Pt M_nominal
     *            + M_damping
     *
     * 1) Pn M_nominal
     *    접촉면 법선 주위 회전축의 기존 자세 제어는 유지한다.
     *    n=+x이면 Mx가 유지된다.
     *
     * 2) passive_scale Pt M_nominal
     *    접촉면 정렬에 필요한 회전축의 자세 제어 강성을 줄인다.
     *    n=+x이면 My와 Mz가 대상이다.
     *
     * 3) damping
     *    자유롭게 둔 회전축에서 과도한 진동이나 계속되는 회전을 억제한다.
     */
    const Eigen::Vector3d hybrid_moment =
      pn * moment + passive_scale_ * pt * moment + damping;

    // -----------------------------------------------------------------------
    // 6. 원래 wrench와 passive-align wrench를 blending
    // -----------------------------------------------------------------------
    // OFF→ON, ON→OFF 전환 중 wrench가 순간적으로 변하지 않도록 선형 보간한다.
    const Eigen::Vector3d force_out =
      (1.0 - blend_) * force + blend_ * hybrid_force;

    const Eigen::Vector3d moment_out =
      (1.0 - blend_) * moment + blend_ * hybrid_moment;

    // Eigen double 벡터를 사용자 정의 Wrench 메시지의 float 배열로 변환한다.
    minitrone_interfaces::msg::Wrench out;
    for (int i = 0; i < 3; ++i) {
      out.force[i] = static_cast<float>(force_out[i]);
      out.moment[i] = static_cast<float>(moment_out[i]);
    }

    output_pub_->publish(out);

    std_msgs::msg::Float64 scalar;
    scalar.data = moment.y();
    my_before_pub_->publish(scalar);
    scalar.data = moment_out.y();
    my_after_pub_->publish(scalar);
    scalar.data = damping.y();
    my_damping_pub_->publish(scalar);
    scalar.data = normal_.dot(force_out);
    normal_force_command_pub_->publish(scalar);
    scalar.data = unsaturated_cmd;
    normal_force_unsaturated_pub_->publish(scalar);
    scalar.data = gravity_normal_compensation;
    gravity_normal_compensation_pub_->publish(scalar);
    std_msgs::msg::Bool saturated_msg;
    saturated_msg.data = force_command_saturated;
    force_command_saturated_pub_->publish(saturated_msg);

    // 활성 상태, 목표 힘, 측정 힘을 모니터링용 토픽으로 출력한다.
    publishStatus();
  }

  /**
   * @brief Passive-align 활성 상태를 변경한다.
   */
  void setEnabled(bool value)
  {
    // 동일한 상태를 다시 요청하면 아무 작업도 하지 않는다.
    if (enabled_ == value) return;

    enabled_ = value;

    // 모드 전환 전의 적분값이 새 동작에 영향을 주지 않도록 초기화한다.
    force_integral_ = 0.0;

    /*
     * 이미 유효한 접촉력 측정값이 있는 상태에서 ON하면,
     * f_ref를 0에서 시작하지 않고 현재 측정 힘 근처에서 시작한다.
     * 따라서 모드 활성화 순간의 접촉력 명령 점프를 줄일 수 있다.
     *
     * 단, f_ref는 [f_min, f_des] 범위로 제한된다.
     */
    if (enabled_ && filter_initialized_) {
      f_ref_ = std::clamp(filtered_force_, f_min_, f_des_);
    }

    RCLCPP_INFO(
      get_logger(), "PASSIVE ALIGN %s", enabled_ ? "ON" : "OFF");

    publishStatus();
  }

  /**
   * @brief 실행 중 변경 가능한 ROS 2 parameter를 처리한다.
   *
   * 여기에 명시된 파라미터만 내부 변수에 즉시 반영된다.
   * 생성자에서 선언된 모든 파라미터가 동적 변경 가능한 것은 아니다.
   */
  rcl_interfaces::msg::SetParametersResult onParameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & p : params) {
      const auto & name = p.get_name();

      if (name == "enabled") {
        setEnabled(p.as_bool());
      } else if (name == "f_normal_des") {
        f_des_ = std::clamp(p.as_double(), f_min_, f_max_);
      } else if (name == "force_kp") {
        force_kp_ = std::max(0.0, p.as_double());
      } else if (name == "force_ki") {
        force_ki_ = std::max(0.0, p.as_double());
      } else if (name == "passive_axis_control_scale") {
        passive_scale_ = std::clamp(p.as_double(), 0.0, 1.0);
      } else if (name == "passive_damping_pitch") {
        damping_.y() = std::max(0.0, p.as_double());
      } else if (name == "passive_damping_yaw") {
        damping_.z() = std::max(0.0, p.as_double());
      } else if (name == "passive_damping_y") {
        damping_.y() = std::max(0.0, p.as_double());
      } else if (name == "passive_damping_z") {
        damping_.z() = std::max(0.0, p.as_double());
      }
    }

    return result;
  }

  /**
   * @brief 모니터링용 상태 토픽을 publish한다.
   */
  void publishStatus()
  {
    // 실제 blend 값이 아니라 사용자가 요청한 enable 상태를 나타낸다.
    std_msgs::msg::Bool active;
    active.data = enabled_;
    active_pub_->publish(active);

    // 최종 목표값 f_des. 내부 ramp 값 f_ref와는 다르다.
    std_msgs::msg::Float64 desired;
    desired.data = f_des_;
    desired_pub_->publish(desired);

    // 외력 측정값의 LPF 결과.
    std_msgs::msg::Float64 measured;
    measured.data = filtered_force_;
    measured_pub_->publish(measured);
    force_measured_pub_->publish(measured);

    std_msgs::msg::Float64 scalar;
    scalar.data = f_des_;
    force_target_pub_->publish(scalar);
    scalar.data = f_ref_;
    force_ref_ramped_pub_->publish(scalar);
    scalar.data = blend_;
    blend_pub_->publish(scalar);
  }

  /**
   * @brief 터미널 입력을 canonical mode에서 non-canonical mode로 변경한다.
   *
   * 일반 터미널은 Enter를 눌러야 입력이 전달되지만,
   * ICANON과 ECHO를 해제하면 L/U/J 키를 즉시 읽을 수 있다.
   */
  void setupKeyboard()
  {
    // stdin이 실제 터미널이 아닐 경우 키보드 기능을 비활성화한다.
    keyboard_enabled_ = isatty(STDIN_FILENO);

    // 기존 터미널 설정을 저장하지 못하면 변경하지 않는다.
    if (!keyboard_enabled_ ||
      tcgetattr(STDIN_FILENO, &old_termios_) != 0)
    {
      return;
    }

    termios raw = old_termios_;

    // ICANON: 줄 단위 입력, ECHO: 입력 문자를 화면에 표시.
    // 두 기능을 제거하여 키 하나를 즉시, 화면 출력 없이 읽는다.
    raw.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));

    // read()가 입력을 기다리며 block되지 않도록 설정한다.
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
      termios_configured_ = true;
    }
  }

  /**
   * @brief 노드 시작 전의 터미널 설정으로 복구한다.
   */
  void restoreKeyboard()
  {
    if (termios_configured_) {
      tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
    }
  }

  /**
   * @brief non-blocking 방식으로 키보드를 확인한다.
   *
   * L : passive-align ON/OFF
   * U : 목표 법선력 +f_step
   * J : 목표 법선력 -f_step
   */
  void pollKeyboard()
  {
    if (!keyboard_enabled_) return;

    // select()로 stdin에 읽을 데이터가 있는지만 먼저 확인한다.
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);

    // timeout 0은 기다리지 않고 즉시 반환하는 non-blocking 검사이다.
    timeval timeout{0, 0};
    if (select(
        STDIN_FILENO + 1, &fds, nullptr, nullptr, &timeout) <= 0)
    {
      return;
    }

    char key = '\0';
    if (read(STDIN_FILENO, &key, 1) != 1) return;

    if (key == 'o' || key == 'O') {
      setEnabled(!enabled_);
    } else if (key == 'u' || key == 'U') {
      f_des_ = std::clamp(f_des_ + f_step_, f_min_, f_max_);
    } else if (key == 'j' || key == 'J') {
      f_des_ = std::clamp(f_des_ - f_step_, f_min_, f_max_);
    }
  }

  // -------------------------------------------------------------------------
  // ROS 2 통신 객체
  // -------------------------------------------------------------------------
  rclcpp::Subscription<minitrone_interfaces::msg::Wrench>::SharedPtr
    input_sub_, external_sub_;
  rclcpp::Subscription<minitrone_interfaces::msg::MinitroneState>::SharedPtr
    state_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;

  rclcpp::Publisher<minitrone_interfaces::msg::Wrench>::SharedPtr output_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
    force_command_saturated_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    desired_pub_, measured_pub_, force_target_pub_, force_ref_ramped_pub_,
    force_measured_pub_, blend_pub_, normal_force_command_pub_,
    normal_force_unsaturated_pub_, gravity_normal_compensation_pub_,
    my_before_pub_, my_after_pub_, my_damping_pub_;

  rclcpp::TimerBase::SharedPtr keyboard_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_handle_;

  // -------------------------------------------------------------------------
  // 기하 및 상태 변수
  // -------------------------------------------------------------------------
  // body frame에서 표현한 접촉면 단위 법선벡터 n.
  Eigen::Vector3d normal_{Eigen::Vector3d::UnitX()};

  // body roll/pitch/yaw 각속도 감쇠 계수.
  Eigen::Vector3d damping_{Eigen::Vector3d::Zero()};

  // body roll/pitch/yaw 각속도 [rad/s].
  Eigen::Vector3d omega_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d rpy_{Eigen::Vector3d::Zero()};

  // 외력 관측기가 추정한 force 벡터 [N].
  Eigen::Vector3d external_force_{Eigen::Vector3d::Zero()};

  // -------------------------------------------------------------------------
  // 법선 접촉력 제어 변수
  // -------------------------------------------------------------------------
  double f_des_{5.0};       // 사용자가 지정한 목표 법선력 [N]
  double f_step_{0.1};      // 키 입력당 목표 법선력 증감량 [N]
  double f_min_{0.0};       // 목표 법선력 하한 [N]
  double f_max_{20.0};      // 목표 법선력 상한 [N]
  double f_rate_{1.0};      // 내부 목표 f_ref 변화율 제한 [N/s]

  double force_kp_{0.5};    // 접촉력 비례 이득
  double force_ki_{0.2};    // 접촉력 적분 이득
  double integral_limit_{2.0};
  double force_integral_{0.0};

  double force_cmd_min_{0.0};
  double force_cmd_max_{20.0};
  double cutoff_hz_{10.0};
  double mass_{2.5};
  double gravity_{9.81};

  // -------------------------------------------------------------------------
  // Passive alignment 및 전환 변수
  // -------------------------------------------------------------------------
  double timeout_sec_{0.1};      // 외력 메시지 유효 시간 [s]
  double passive_scale_{0.0};    // 접선축 nominal moment 잔존 비율
  double moment_limit_{0.5};     // 축별 damping moment 제한 [N·m]
  double transition_sec_{0.3};   // ON/OFF blending 시간 [s]

  double blend_{0.0};            // 0: nominal, 1: passive-align
  double f_ref_{0.0};            // slew-rate가 적용된 내부 목표 법선력 [N]
  double filtered_force_{0.0};   // LPF를 거친 측정 법선력 [N]

  // -------------------------------------------------------------------------
  // 시간 및 상태 플래그
  // -------------------------------------------------------------------------
  rclcpp::Time last_output_time_;
  rclcpp::Time last_external_time_;

  bool enabled_{false};
  bool have_external_{false};
  bool filter_initialized_{false};

  // -------------------------------------------------------------------------
  // 터미널 키보드 처리 변수
  // -------------------------------------------------------------------------
  bool keyboard_enabled_{false};
  bool termios_configured_{false};
  termios old_termios_{};
};

int main(int argc, char ** argv)
{
  // ROS 2 통신 초기화.
  rclcpp::init(argc, argv);

  // PassiveAligningController 노드를 생성하고 callback을 계속 실행한다.
  rclcpp::spin(std::make_shared<PassiveAligningController>());

  // spin 종료 후 ROS 2 자원을 정리한다.
  rclcpp::shutdown();
  return 0;
}
