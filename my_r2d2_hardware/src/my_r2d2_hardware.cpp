#include "my_r2d2_hardware/my_r2d2_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace my_r2d2_hardware{

hardware_interface::return_type MyR2D2Hardware::configure(const hardware_interface::HardwareInfo & info){
  base_x_ = 0.0;
  base_y_ = 0.0;
  base_theta_ = 0.0;

  if (configure_default(info) != hardware_interface::return_type::OK){
    return hardware_interface::return_type::ERROR;
  }

  hw_start_sec_ = stod(info_.hardware_parameters["hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["hw_stop_duration_sec"]);
  wheel_radius = stod(info_.hardware_parameters["wheel_radius"]);
  wheel_separation = stod(info_.hardware_parameters["wheel_separation"]);
  dt = stod(info_.hardware_parameters["dt"]);

  for(auto& joint: info_.joints){
    RCLCPP_INFO(rclcpp::get_logger("MyR2D2Hardware"), "Get joints : '%s'",joint.name.c_str());
  }
  RCLCPP_INFO(rclcpp::get_logger("MyR2D2Hardware"), "PARAMETERS | [hw_start_duration_sec : %f] [hw_stop_duration_sec : %f] [wheel_radius : %f] [wheel_separation : %f] [dt : %f]",hw_start_sec_,hw_stop_sec_,wheel_radius,wheel_separation,dt);

  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints){
    // MyR2D2System has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1){
      RCLCPP_FATAL(
        rclcpp::get_logger("MyR2D2System"),
        "Joint '%s' has %d command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY){
      RCLCPP_FATAL(
        rclcpp::get_logger("MyR2D2System"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 2){
      RCLCPP_FATAL(
        rclcpp::get_logger("MyR2D2Hardware"),
        "Joint '%s' has %d state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION){
      RCLCPP_FATAL(
        rclcpp::get_logger("MyR2D2System"),
        "Joint '%s' have '%s' as first state interface. '%s' and '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY){
      RCLCPP_FATAL(
        rclcpp::get_logger("MyR2D2System"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }
  }

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> MyR2D2Hardware::export_state_interfaces(){
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++){
    RCLCPP_INFO(rclcpp::get_logger("MyR2D2Hardware"), "Joint '%s' : Export state interfaces",info_.joints[i].name.c_str());
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MyR2D2Hardware::export_command_interfaces(){
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++){
    RCLCPP_INFO(rclcpp::get_logger("MyR2D2Hardware"), "Joint '%s' : Export command interfaces",info_.joints[i].name.c_str());
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type MyR2D2Hardware::start(){
  RCLCPP_INFO(rclcpp::get_logger("MyR2D2Hardware"), "Starting ...please wait...");
  for (auto i = 0; i <= hw_start_sec_; i++){
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(rclcpp::get_logger("MyR2D2Hardware"), "%.1f seconds left...", hw_start_sec_ - i);
  }

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++){
    if (std::isnan(hw_positions_[i])){
      RCLCPP_INFO(rclcpp::get_logger("MyR2D2Hardware"), "Joint '%s' : Set some default values",info_.joints[i].name.c_str());
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(rclcpp::get_logger("MyR2D2Hardware"), "System Successfully started!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyR2D2Hardware::stop(){
  RCLCPP_INFO(rclcpp::get_logger("MyR2D2Hardware"), "Stopping ...please wait...");

  for (auto i = 0; i <= hw_stop_sec_; i++){
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(rclcpp::get_logger("MyR2D2Hardware"), "%.1f seconds left...", hw_stop_sec_ - i);
  }

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(rclcpp::get_logger("MyR2D2Hardware"), "System successfully stopped!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyR2D2Hardware::read(){
  RCLCPP_INFO(rclcpp::get_logger("MyR2D2Hardware"), "Reading... %d commands",hw_commands_.size());

  for (auto i = 0u; i < hw_commands_.size(); i++){
    // Simply integrates
    
    // command -> angular velocity [rad/s] -> velocity ( v = r * w)
    hw_positions_[i] = hw_positions_[i] + dt * hw_commands_[i];
    hw_velocities_[i] = hw_commands_[i];

    RCLCPP_INFO(
      rclcpp::get_logger("MyR2D2Hardware"),
      "Got position state %.5f [rad] ( %.5f [m]) and velocity state %.5f for '%s'!", hw_positions_[i], hw_positions_[i] * wheel_radius,
      hw_velocities_[i], info_.joints[i].name.c_str());
  }

  // simple diffeerentiable kinematics : https://tajimarobotics.com/wheel_robot/  
  double base_dx = 0.5 * wheel_radius * (hw_commands_[0] + hw_commands_[1]) * cos(base_theta_);
  double base_dy = 0.5 * wheel_radius * (hw_commands_[0] + hw_commands_[1]) * sin(base_theta_);
  double base_dtheta = wheel_radius * (hw_commands_[0] - hw_commands_[1]) / wheel_separation;
  base_x_ += base_dx * dt;
  base_y_ += base_dy * dt;
  base_theta_ += base_dtheta * dt;

  RCLCPP_INFO(rclcpp::get_logger("MyR2D2Hardware"), "Joints successfully read! (%.5f,%.5f,%.5f)",base_x_, base_y_, base_theta_);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyR2D2Hardware::MyR2D2Hardware::write(){
  RCLCPP_INFO(rclcpp::get_logger("MyR2D2Hardware"), "Writing...");

  for (auto i = 0u; i < hw_commands_.size(); i++){
    // Simulate sending commands to the hardware
    RCLCPP_INFO(rclcpp::get_logger("MyR2D2Hardware"), "Got command %.5f for '%s'!", hw_commands_[i],info_.joints[i].name.c_str());
  }

  RCLCPP_INFO(rclcpp::get_logger("MyR2D2Hardware"), "Joints successfully written!");
  return hardware_interface::return_type::OK;
}

}  // namespace my_r2d2_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  my_r2d2_hardware::MyR2D2Hardware, hardware_interface::SystemInterface)