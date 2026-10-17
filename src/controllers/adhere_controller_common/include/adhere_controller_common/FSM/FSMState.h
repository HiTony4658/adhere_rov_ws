#ifndef ADHERE_CONTROLLER_COMMON__FSM__FSMSTATE_H_
#define ADHERE_CONTROLLER_COMMON__FSM__FSMSTATE_H_

#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "adhere_controller_common/common/enumClass.h"
#include "adhere_controller_common/CtrlInterfaces.h"

namespace adhere_controller_common
{

class FSMState
{
public:
  virtual ~FSMState() = default;

  FSMState(FSMStateName state_name, std::string state_name_string, CtrlInterfaces & ctrl_interfaces)
  : state_name(state_name),
    state_name_string(std::move(state_name_string)),
    ctrl_interfaces_(ctrl_interfaces)
  {
  }

  virtual void enter() = 0;

  virtual void run(const rclcpp::Time & time, const rclcpp::Duration & period) = 0;

  virtual void exit() = 0;

  virtual FSMStateName checkChange() { return FSMStateName::INVALID; }

  FSMStateName state_name;
  std::string state_name_string;

protected:
  CtrlInterfaces & ctrl_interfaces_;
};

}  // namespace adhere_controller_common

#endif  // ADHERE_CONTROLLER_COMMON__FSM__FSMSTATE_H_
