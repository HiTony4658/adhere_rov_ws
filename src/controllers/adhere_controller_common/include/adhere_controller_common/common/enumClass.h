#ifndef ADHERE_CONTROLLER_COMMON__COMMON__ENUMCLASS_H_
#define ADHERE_CONTROLLER_COMMON__COMMON__ENUMCLASS_H_

namespace adhere_controller_common
{

enum class FSMStateName
{
  INVALID,
  PASSIVE,
  DEPTH_HOLD,
  OPEN_LOOP,
  PID_HOLD
};

enum class FSMMode
{
  NORMAL,
  CHANGE
};

}  // namespace adhere_controller_common

#endif  // ADHERE_CONTROLLER_COMMON__COMMON__ENUMCLASS_H_
