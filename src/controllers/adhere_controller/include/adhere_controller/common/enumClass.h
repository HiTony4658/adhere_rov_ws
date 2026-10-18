#ifndef ADHERE_CONTROLLER__COMMON__ENUMCLASS_H_
#define ADHERE_CONTROLLER__COMMON__ENUMCLASS_H_

namespace adhere_controller
{

enum class FSMStateName
{
  INVALID,
  PASSIVE,
  DEPTH_HOLD,
  OPEN_LOOP,
  PID_HOLD,
  ADHERE_PID,
  ADHERE_SMC
};

enum class FSMMode
{
  NORMAL,
  CHANGE
};

}  // namespace adhere_controller

#endif  // ADHERE_CONTROLLER__COMMON__ENUMCLASS_H_
