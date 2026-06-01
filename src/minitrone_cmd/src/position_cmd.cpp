#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <minitrone_interfaces/msg/cmd.hpp>

// static constexpr double X_CMD = 0.0;
// static constexpr double Y_CMD = 0.0;
// static constexpr double Z_CMD = 1.0;
static constexpr int    RATE_HZ = 200;

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
    //present parameter reading
    this->get_parameter("X_CMD", X_CMD_);
    this->get_parameter("Y_CMD", Y_CMD_); 
    this->get_parameter("Z_CMD", Z_CMD_);

    RCLCPP_INFO(this->get_logger(),
                "Initial params - X_CMD: %.3f, Y_CMD: %.3f, Z_CMD: %.3f",
                X_CMD_, Y_CMD_, Z_CMD_);

    // 3) 파라미터가 실행 중에 바뀔 때 콜백 (ros2 param set 용)
    param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&PositionCmd::onParamChange, this, std::placeholders::_1)
    );

    // 4) 퍼블리셔 생성
    pub_cmd_ = this->create_publisher<Cmd>("/minitrone/cmd", 10);

    // 5) 주기 타이머 설정 (200 Hz → 5 ms)
    auto period_ms = std::chrono::milliseconds(1000 / (RATE_HZ > 0 ? RATE_HZ : 1));
    t0_ = std::chrono::steady_clock::now();
    timer_ = this->create_wall_timer(
      period_ms,
      std::bind(&PositionCmd::onTick, this)
    );
  }

  private:
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
        }
      }

      return result;
    }

  
  void onTick(){
    using minitrone_interfaces::msg::Cmd;

    // 경과 시간 계산
    auto now = std::chrono::steady_clock::now();
    double t = std::chrono::duration<double>(now - t0_).count();  // [sec]

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

     // 시간에 따라 목표 위치를 단계적으로 변경
    if (t < 5.0) {
      // 처음 5초: z만 명령 (수직 상승/호버)
      x = 0.0;
      y = 0.0;
      z = Z_CMD_;
    } else if (t < 10.0) {
      // 다음 5초: x까지 추가 (x 방향으로 이동)
      x = X_CMD_;
      y = 0.0;
      z = Z_CMD_;
    } else {
      // 5초 이후: y까지 추가 (x, y 둘 다 명령)
      x = X_CMD_;
      y = Y_CMD_;
      z = Z_CMD_;
    }

    Cmd msg;
    msg.pos_cmd[0] = static_cast<float>(x);
    msg.pos_cmd[1] = static_cast<float>(y)                       ;
    msg.pos_cmd[2] = static_cast<float>(z);
    
    pub_cmd_->publish(msg);
  }

  rclcpp::Publisher<minitrone_interfaces::msg::Cmd>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::chrono::steady_clock::time_point t0_;
    
  double X_CMD_;
  double Y_CMD_;
  double Z_CMD_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionCmd>());
  rclcpp::shutdown();
  return 0;
}
