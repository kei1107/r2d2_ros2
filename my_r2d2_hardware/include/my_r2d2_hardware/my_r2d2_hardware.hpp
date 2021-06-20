#ifndef MY_R2D2_HARDWARE__SYSTEM_HPP_
#define MY_R2D2_HARDWARE__SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"
#include "my_r2d2_hardware/visibility_control.h"

namespace my_r2d2_hardware{
class MyR2D2Hardware : public hardware_interface::BaseInterface<hardware_interface::SystemInterface>{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MyR2D2Hardware)

  MY_R2D2_HARDWARE_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  MY_R2D2_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  MY_R2D2_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  MY_R2D2_HARDWARE_PUBLIC
  hardware_interface::return_type start() override;

  MY_R2D2_HARDWARE_PUBLIC
  hardware_interface::return_type stop() override;

  MY_R2D2_HARDWARE_PUBLIC
  hardware_interface::return_type read() override;

  MY_R2D2_HARDWARE_PUBLIC
  hardware_interface::return_type write() override;

private:

// Parameters for the DiffBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  // Store the wheeled robot position
  double base_x_, base_y_, base_theta_;

  // wheel information
  // radius of the wheels, distance between the wheels
  double wheel_radius,wheel_separation;

  // update rate [s]
  // Control period
  double dt;
};
} // namespace my_r2d2_hardware


#endif // MY_R2D2_HARDWARE__SYSTEM_HPP_