#include "adhere_mujoco_hardware/flow_environment_plugin.hpp"

namespace adhere_mujoco_hardware
{

std::unique_ptr<FlowEnvironmentPlugin> createFlowEnvironment(const std::string & profile_name)
{
  if (profile_name == "station_keeping_gust") {
    return std::make_unique<StationKeepingGustPlugin>();
  }
  // "calm" and any unrecognized profile name both fall back to calm water.
  return std::make_unique<CalmWaterPlugin>();
}

}  // namespace adhere_mujoco_hardware
