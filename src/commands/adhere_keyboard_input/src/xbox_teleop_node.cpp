// xbox_teleop_node: Mode-2 (美国手) Xbox gamepad teleop bridge (CLAUDE.md Phase 3.2 Stage 2.3).
// Subscribes sensor_msgs/msg/Joy on /joy (published by the official ROS 2 joy package's
// joy_node) and republishes TeleopCommand on /rov/teleop_cmd synchronously in the /joy callback
// -- no separate timer needed, since joy_node's autorepeat_rate parameter already drives the
// cadence (default 50Hz, see config/xbox_teleop.yaml / launch/teleop_gcs.launch.py). Also
// edge-debounces 4 gamepad buttons into instant FsmCommand state switches on /rov/fsm_command.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

#include "adhere_control_msgs/msg/fsm_command.hpp"
#include "adhere_control_msgs/msg/teleop_command.hpp"

namespace
{

// Deadzone + optional cubic sensitivity curve (CLAUDE.md Phase 3.2 Stage 2.3): rescales the
// post-deadzone magnitude into [0,1] before applying the curve, so the full [-1,1] command
// range is still reachable right at the stick's physical limit.
double applyDeadzoneCurve(double raw, double deadzone, bool cubic)
{
  const double mag = std::abs(raw);
  if (mag <= deadzone) {
    return 0.0;
  }
  double scaled = (mag - deadzone) / (1.0 - deadzone);
  scaled = std::min(scaled, 1.0);
  if (cubic) {
    scaled = scaled * scaled * scaled;
  }
  return std::copysign(scaled, raw);
}

// Phase 3.5: LT/RT trigger -> [0.0, 1.0] press magnitude with a deadzone, no sign (triggers are
// single-sided, unlike the full-range sticks above). Most Linux xpad configs report triggers
// resting-unpressed at +1.0 and fully-pressed at -1.0 (the same convention joy_node forwards
// verbatim), hence the (1 - raw) / 2 remap before the deadzone.
double applyTriggerDeadzone(double raw, double deadzone)
{
  const double pressed = std::clamp((1.0 - raw) / 2.0, 0.0, 1.0);
  if (pressed <= deadzone) {
    return 0.0;
  }
  return std::min((pressed - deadzone) / (1.0 - deadzone), 1.0);
}

}  // namespace

class XboxTeleopNode : public rclcpp::Node
{
public:
  XboxTeleopNode()
  : Node("xbox_teleop_node")
  {
    deadzone_ = declare_parameter<double>("deadzone", 0.08);
    cubic_curve_ = declare_parameter<bool>("cubic_curve", true);

    axis_heave_ = declare_parameter<int>("axis_heave", 1);
    axis_yaw_ = declare_parameter<int>("axis_yaw", 0);
    axis_surge_ = declare_parameter<int>("axis_surge", 4);
    axis_sway_ = declare_parameter<int>("axis_sway", 3);
    axis_pitch_trigger_ = declare_parameter<int>("axis_pitch_trigger", 2);  // LT
    axis_roll_trigger_ = declare_parameter<int>("axis_roll_trigger", 5);    // RT

    button_passive_ = declare_parameter<int>("button_passive", 0);     // A
    button_pid_hold_ = declare_parameter<int>("button_pid_hold", 1);   // B
    button_depth_hold_ = declare_parameter<int>("button_depth_hold", 2);  // X
    button_open_loop_ = declare_parameter<int>("button_open_loop", 3);    // Y
    button_pitch_reverse_ = declare_parameter<int>("button_pitch_reverse", 4);  // LB
    button_roll_reverse_ = declare_parameter<int>("button_roll_reverse", 5);    // RB

    teleop_pub_ =
      create_publisher<adhere_control_msgs::msg::TeleopCommand>("/rov/teleop_cmd", 10);
    fsm_command_pub_ =
      create_publisher<adhere_control_msgs::msg::FsmCommand>("/rov/fsm_command", 10);

    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&XboxTeleopNode::joyCallback, this, std::placeholders::_1));
  }

private:
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    const auto axisValue = [&msg](int index) -> double {
      return (index >= 0 && static_cast<size_t>(index) < msg->axes.size()) ? msg->axes[index]
                                                                            : 0.0;
    };
    const auto buttonPressed = [&msg](int index) -> bool {
      return index >= 0 && static_cast<size_t>(index) < msg->buttons.size() &&
        msg->buttons[index] != 0;
    };

    // Mode-2 (美国手) stick mapping (CLAUDE.md Phase 3.2 Stage 2.3): raw axis sign convention
    // matches the Linux xpad driver's default (up/left = positive) that joy_node forwards
    // verbatim; applyDeadzoneCurve()'s sign-preserving copysign keeps that convention intact.
    const double surge_x = applyDeadzoneCurve(axisValue(axis_surge_), deadzone_, cubic_curve_);
    const double sway_y = applyDeadzoneCurve(axisValue(axis_sway_), deadzone_, cubic_curve_);
    const double heave_z = applyDeadzoneCurve(axisValue(axis_heave_), deadzone_, cubic_curve_);
    const double yaw_rate = applyDeadzoneCurve(axisValue(axis_yaw_), deadzone_, cubic_curve_);

    // Phase 3.5: LT (+LB reverses) -> pitch_rate, RT (+RB reverses) -> roll_rate.
    const double lt_val = applyTriggerDeadzone(axisValue(axis_pitch_trigger_), deadzone_);
    const double rt_val = applyTriggerDeadzone(axisValue(axis_roll_trigger_), deadzone_);
    const double pitch_rate = buttonPressed(button_pitch_reverse_) ? -lt_val : lt_val;
    const double roll_rate = buttonPressed(button_roll_reverse_) ? -rt_val : rt_val;

    adhere_control_msgs::msg::TeleopCommand teleop_msg;
    teleop_msg.header.stamp = now();
    teleop_msg.linear.x = surge_x;
    teleop_msg.linear.y = sway_y;
    teleop_msg.linear.z = heave_z;
    teleop_msg.angular.x = roll_rate;
    teleop_msg.angular.y = pitch_rate;
    teleop_msg.angular.z = yaw_rate;
    teleop_msg.active = (surge_x != 0.0) || (sway_y != 0.0) || (heave_z != 0.0) ||
      (yaw_rate != 0.0) || (pitch_rate != 0.0) || (roll_rate != 0.0);
    teleop_pub_->publish(teleop_msg);

    publishFsmCommandOnRisingEdge(
      msg->buttons, button_passive_, adhere_control_msgs::msg::FsmCommand::PASSIVE);
    publishFsmCommandOnRisingEdge(
      msg->buttons, button_depth_hold_, adhere_control_msgs::msg::FsmCommand::DEPTH_HOLD);
    publishFsmCommandOnRisingEdge(
      msg->buttons, button_open_loop_, adhere_control_msgs::msg::FsmCommand::OPEN_LOOP);
    publishFsmCommandOnRisingEdge(
      msg->buttons, button_pid_hold_, adhere_control_msgs::msg::FsmCommand::PID_HOLD);
  }

  // Rising-edge debounce (CLAUDE.md Phase 3.2 Stage 2.3): fires FsmCommand exactly once per
  // button press, not once per /joy message while the button stays held.
  void publishFsmCommandOnRisingEdge(const std::vector<int32_t> & buttons, int index, int32_t command)
  {
    if (index < 0 || static_cast<size_t>(index) >= buttons.size()) {
      return;
    }
    if (prev_button_states_.size() != buttons.size()) {
      prev_button_states_.assign(buttons.size(), 0);
    }
    const bool pressed = buttons[index] != 0;
    const bool rising_edge = pressed && prev_button_states_[index] == 0;
    prev_button_states_[index] = pressed ? 1 : 0;
    if (!rising_edge) {
      return;
    }
    adhere_control_msgs::msg::FsmCommand cmd;
    cmd.header.stamp = now();
    cmd.command = command;
    fsm_command_pub_->publish(cmd);
  }

  double deadzone_ = 0.08;
  bool cubic_curve_ = true;
  int axis_heave_ = 1;
  int axis_yaw_ = 0;
  int axis_surge_ = 4;
  int axis_sway_ = 3;
  int axis_pitch_trigger_ = 2;
  int axis_roll_trigger_ = 5;
  int button_passive_ = 0;
  int button_pid_hold_ = 1;
  int button_depth_hold_ = 2;
  int button_open_loop_ = 3;
  int button_pitch_reverse_ = 4;
  int button_roll_reverse_ = 5;

  std::vector<int> prev_button_states_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<adhere_control_msgs::msg::TeleopCommand>::SharedPtr teleop_pub_;
  rclcpp::Publisher<adhere_control_msgs::msg::FsmCommand>::SharedPtr fsm_command_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<XboxTeleopNode>());
  rclcpp::shutdown();
  return 0;
}
