#include <chrono>
#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <minitrone_interfaces/msg/cmd.hpp>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

// static constexpr double X_CMD = 0.0;
// static constexpr double Y_CMD = 0.0;
// static constexpr double Z_CMD = 1.0;
static constexpr int    RATE_HZ = 200;

namespace
{
double moveToward(double current, double target, double max_step)
{
  const double step = std::max(0.0, max_step);
  if (current < target) {
    return std::min(current + step, target);
  }
  return std::max(current - step, target);
}
}  // namespace

class PositionCmd : public rclcpp::Node {
public:
  PositionCmd() : rclcpp::Node("minitrone_position_cmd"),
                  X_CMD_(0.0),
                  Y_CMD_(0.0),
                  Z_CMD_(1.0)
  {
    using minitrone_interfaces::msg::Cmd;
    


    this->declare_parameter<double>("X_CMD", 0.0);
    this->declare_parameter<double>("Y_CMD", 0.0);
    this->declare_parameter<double>("Z_CMD", 1.0);
    this->declare_parameter<double>("max_speed_x", 0.10);
    this->declare_parameter<double>("max_speed_y", 0.10);
    this->declare_parameter<double>("max_speed_z", 0.30);
    this->declare_parameter<double>("approach_start_sec", 5.0);
    this->declare_parameter<double>("y_start_sec", 10.0);
    enabled_ = this->declare_parameter<bool>("enabled", false);
    //present parameter reading
    this->get_parameter("X_CMD", X_CMD_);
    this->get_parameter("Y_CMD", Y_CMD_); 
    this->get_parameter("Z_CMD", Z_CMD_);
    max_speed_x_ = std::abs(this->get_parameter("max_speed_x").as_double());
    max_speed_y_ = std::abs(this->get_parameter("max_speed_y").as_double());
    max_speed_z_ = std::abs(this->get_parameter("max_speed_z").as_double());
    approach_start_sec_ = std::max(
      0.0, this->get_parameter("approach_start_sec").as_double());
    y_start_sec_ = std::max(
      approach_start_sec_, this->get_parameter("y_start_sec").as_double());
    // Preserve the existing takeoff command. The new rate limiter primarily
    // removes the abrupt wall-approach step on x; later Z_CMD changes are
    // still rate-limited from this initial reference.
    ref_z_ = Z_CMD_;

    RCLCPP_INFO(this->get_logger(),
                "Position ramp target=[%.3f %.3f %.3f], max speed=[%.3f %.3f %.3f] m/s",
                X_CMD_, Y_CMD_, Z_CMD_, max_speed_x_, max_speed_y_, max_speed_z_);

    // 3) 파라미터가 실행 중에 바뀔 때 콜백 (ros2 param set 용)
    param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&PositionCmd::onParamChange, this, std::placeholders::_1)
    );

    // 4) 퍼블리셔 생성
    pub_cmd_ = this->create_publisher<Cmd>("/minitrone/cmd", 10);

    // 5) 주기 타이머 설정 (200 Hz → 5 ms)
    auto period_ms = std::chrono::milliseconds(1000 / (RATE_HZ > 0 ? RATE_HZ : 1));
    t0_ = std::chrono::steady_clock::now();
    last_tick_ = t0_;
    timer_ = this->create_wall_timer(
      period_ms,
      std::bind(&PositionCmd::onTick, this)
    );
    setupKeyboard();
    RCLCPP_INFO(this->get_logger(), "Position command ready. O=ON/OFF");
  }

  ~PositionCmd() override
  {
    restoreKeyboard();
  }

  private:
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
      fd_set read_fds;
      FD_ZERO(&read_fds);
      FD_SET(STDIN_FILENO, &read_fds);
      timeval timeout{0, 0};
      if (select(STDIN_FILENO + 1, &read_fds, nullptr, nullptr, &timeout) <= 0) return;
      char key = '\0';
      if (read(STDIN_FILENO, &key, 1) == 1 && (key == 'o' || key == 'O')) {
        enabled_ = !enabled_;
        if (enabled_) {
          t0_ = std::chrono::steady_clock::now();
          last_tick_ = t0_;
        }
        RCLCPP_INFO(
          this->get_logger(), "Position command %s", enabled_ ? "ON" : "OFF");
      }
    }

    // 파라미터 변경 콜백 (ros2 param set으로 값 바꿀 때 반영)
    rcl_interfaces::msg::SetParametersResult
    onParamChange(const std::vector<rclcpp::Parameter> &params)
    {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      result.reason = "success";

      for (const auto &p : params) {
        if (p.get_name() == "X_CMD") {
          X_CMD_ = p.as_double();
          RCLCPP_INFO(this->get_logger(), "X_CMD updated: %.3f", X_CMD_);
        } else if (p.get_name() == "Y_CMD") {
          Y_CMD_ = p.as_double();
          RCLCPP_INFO(this->get_logger(), "Y_CMD updated: %.3f", Y_CMD_);
        } else if (p.get_name() == "Z_CMD") {
          Z_CMD_ = p.as_double();
          RCLCPP_INFO(this->get_logger(), "Z_CMD updated: %.3f", Z_CMD_);
        } else if (p.get_name() == "max_speed_x") {
          max_speed_x_ = std::abs(p.as_double());
        } else if (p.get_name() == "max_speed_y") {
          max_speed_y_ = std::abs(p.as_double());
        } else if (p.get_name() == "max_speed_z") {
          max_speed_z_ = std::abs(p.as_double());
        } else if (p.get_name() == "approach_start_sec") {
          approach_start_sec_ = std::max(0.0, p.as_double());
        } else if (p.get_name() == "y_start_sec") {
          y_start_sec_ = std::max(approach_start_sec_, p.as_double());
        } else if (p.get_name() == "enabled") {
          enabled_ = p.as_bool();
        }
      }

      return result;
    }

  
  void onTick(){
    using minitrone_interfaces::msg::Cmd;
    pollKeyboard();
    if (!enabled_) return;

    // 경과 시간 계산
    auto now = std::chrono::steady_clock::now();
    const double t = std::chrono::duration<double>(now - t0_).count();
    double dt = std::chrono::duration<double>(now - last_tick_).count();
    last_tick_ = now;
    if (!(dt > 0.0) || dt > 0.1) {
      dt = 1.0 / static_cast<double>(RATE_HZ);
    }

    // Preserve the original sequence (takeoff, x approach, then y), but move
    // the published reference toward each target with a bounded velocity.
    const double target_x = t >= approach_start_sec_ ? X_CMD_ : 0.0;
    const double target_y = t >= y_start_sec_ ? Y_CMD_ : 0.0;
    const double target_z = Z_CMD_;
    ref_x_ = moveToward(ref_x_, target_x, max_speed_x_ * dt);
    ref_y_ = moveToward(ref_y_, target_y, max_speed_y_ * dt);
    ref_z_ = moveToward(ref_z_, target_z, max_speed_z_ * dt);

    Cmd msg;
    msg.pos_cmd[0] = static_cast<float>(ref_x_);
    msg.pos_cmd[1] = static_cast<float>(ref_y_);
    msg.pos_cmd[2] = static_cast<float>(ref_z_);
    
    pub_cmd_->publish(msg);
  }

  rclcpp::Publisher<minitrone_interfaces::msg::Cmd>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point t0_;
  std::chrono::steady_clock::time_point last_tick_;
    
  double X_CMD_;
  double Y_CMD_;
  double Z_CMD_;
  double max_speed_x_{0.10};
  double max_speed_y_{0.10};
  double max_speed_z_{0.30};
  double approach_start_sec_{5.0};
  double y_start_sec_{10.0};
  double ref_x_{0.0};
  double ref_y_{0.0};
  double ref_z_{0.0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
  bool enabled_{false};
  bool keyboard_enabled_{false};
  bool termios_configured_{false};
  termios old_termios_{};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionCmd>());
  rclcpp::shutdown();
  return 0;
}
