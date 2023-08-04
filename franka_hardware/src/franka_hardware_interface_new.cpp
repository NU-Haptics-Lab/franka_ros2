// Copyright (c) 2021 Franka Emika GmbH
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <franka_hardware/franka_hardware_interface_new.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <chrono>

#include <franka/exception.h>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>

namespace franka_hardware {

using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

std::vector<StateInterface> FrankaHardwareInterfaceNew::export_state_interfaces() {
  std::vector<StateInterface> state_interfaces;
  for (auto i = 0U; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_.at(0)));
    state_interfaces.emplace_back(StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_.at(i)));
        //for my controller: effort state interface is Ftip
    state_interfaces.emplace_back(
        StateInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_.at(i)));
  }
  return state_interfaces;
}

std::vector<CommandInterface> FrankaHardwareInterfaceNew::export_command_interfaces() {
  std::vector<CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());
  for (auto i = 0U; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_.at(i)));
  }
  return command_interfaces;
}

FrankaHardwareInterfaceNew::CallbackReturn FrankaHardwareInterfaceNew::on_activate(const rclcpp_lifecycle::State &) {
  robot_->initializeContinuousReading();
  hw_commands_.fill(0);
  auto initial_state = robot_->read();
  prev_time = initial_state.time;
  prev_jacobian_array_ = model_->zeroJacobian(franka::Frame::kEndEffector, initial_state);
  int k{0};
  for (int i{0}; i < 6; ++i){
      for (int j{0}; j < 7; ++j){
          prev_jacobian(i,j) = prev_jacobian_array_[k];
          ++k;
      }
  }
  // Note: read does not use Time in the version of the api
  read(rclcpp::Time(), rclcpp::Time()-rclcpp::Time());  // makes sure that the robot state is properly initialized.
  MyFile.open("~/output.txt");
 
  RCLCPP_INFO(getLogger(), "Started");
  
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

FrankaHardwareInterfaceNew::CallbackReturn   FrankaHardwareInterfaceNew::on_deactivate(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(getLogger(), "trying to Stop...");
  robot_->stopRobot();
  RCLCPP_INFO(getLogger(), "Stopped");
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FrankaHardwareInterfaceNew::read(const rclcpp::Time &, const rclcpp::Duration & ) {
  auto kState = robot_->read();
  dt = kState.time - prev_time;
  prev_time = kState.time;

  F_tip_ = kState.O_F_ext_hat_K; //should be a negative number for force applied to robot
  for (int i{0}; i < 6; ++i){
    hw_efforts_[i] = F_tip_[i];
  }

  calculate_twist(kState.dq, model_->zeroJacobian(franka::Frame::kEndEffector, kState));
  //hw_velocities set in calculate_twist function
  d_jacobian = (jacobian - prev_jacobian)/dt.toSec();

  // coriolis vector
  hw_coriolis_array = model_->coriolis(kState);
  for (int i{0}; i < 7; ++i){
        hw_coriolis_(i) = hw_coriolis_array[i];
  }

  // hw_coriolis_ = model_->coriolis(kState);
  // hw_efforts_ = kState.tau_J;
  // gravity_array_ = model_->gravity(kState); // this is to check for tau_j -> tau_ext calc

  mass_matrix_array = model_->mass(kState);
  int f{0};
  for (int i{0}; i < 7; ++i){
    for (int j{0}; j < 7; ++j){
        mass_matrix(i,j) = mass_matrix_array[f];
        ++f;
    }
  }
  MyFile << "mass matrix array: " << mass_matrix << std::endl;
  MyFile << "jacobian array: " << jacobian << std::endl;
  MyFile << "derivative of jacobian: " << d_jacobian << std::endl;

// NimbRo Implementation
  // if (j > 0){
  //   if (j == 1){
  //     tau_bias = kState.tau_ext_hat_filtered;
  //   //   std::cout << "tau_bias: [";
  //   //   for(int i{0}; i < (int) sizeof(tau_bias.data()); i++){
  //   //     std::cout << tau_bias.data()[i] << ' ';
  //   //   }
  //   //   std::cout<<std::endl;
  //   }
  //   hw_efforts_ = kState.tau_ext_hat_filtered;
  //   for(int i{0}; i < (int) sizeof(hw_efforts_.data()); i++){
  //     hw_efforts_.data()[i] -= tau_bias.data()[i] - hw_coriolis_.data()[i];
  //   }
    
    // Simple output function
    // std::cout << "feed forward torque: [";
    // for(int i{0}; i < (int) sizeof(hw_efforts_.data()); i++){
    //   std::cout << hw_efforts_.data()[i] << ' ';
    // }
    // std::cout << "]\n";
  // }
  // j++;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaHardwareInterfaceNew::write(const rclcpp::Time &, const rclcpp::Duration & ) {
  if (std::any_of(hw_commands_.begin(), hw_commands_.end(),
                  [](double c) { return not std::isfinite(c); })) {
    return hardware_interface::return_type::ERROR;
  }
  
  for (int i{0}; i < 6; ++i){
    d_twist(i) = hw_commands_.data()[i];
  }

  // joint acceleration calculation
  accelerations_ = jacobian.completeOrthogonalDecomposition().pseudoInverse() * (d_twist - d_jacobian*vel);
  prev_jacobian = jacobian;

  // joint torque calculation
  tau_cmd = mass_matrix * accelerations_ + hw_coriolis_;
  for(int i{0}; i < 7; i++){
    if(abs(tau_cmd(i)) <= 1.0){
      tau_cmd_array[i] = tau_cmd(i);
    } else if (tau_cmd(i) > 1.0){
      tau_cmd_array[i] = 1.0;
    } else if (tau_cmd(i) < -1.0){
      tau_cmd_array[i] = -1.0;
    }
  }
  robot_->write(tau_cmd_array);


  // std::cout << "commanded torque: [";
  // for(int i{0}; i < (int) sizeof(hw_commands_.data()); i++){
  //   std::cout << hw_commands_.data()[i] << ' ';
  // }
  // std::cout << "]\n";

  return hardware_interface::return_type::OK;
}

FrankaHardwareInterfaceNew::CallbackReturn FrankaHardwareInterfaceNew::on_init(const hardware_interface::HardwareInfo& info) {
    if(hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }
  if (info_.joints.size() != kNumberOfJoints) {
    RCLCPP_FATAL(getLogger(), "Got %zu joints. Expected %zu.", info_.joints.size(), kNumberOfJoints);
    return CallbackReturn::ERROR;
  }

  for (const auto& joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected command interface '%s'. Expected '%s'",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_EFFORT);
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has %zu state interfaces found. 3 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_VELOCITY);
    }
    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(getLogger(), "Joint '%s' has unexpected state interface '%s'. Expected '%s'",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_EFFORT);
    }
  }
  std::string robot_ip;
  try {
    robot_ip = info_.hardware_parameters.at("robot_ip");
  } catch (const std::out_of_range& ex) {
    RCLCPP_FATAL(getLogger(), "Parameter 'robot_ip' not set");
    return CallbackReturn::ERROR;
  }
  try {
    RCLCPP_INFO(getLogger(), "Connecting to robot at \"%s\" ...", robot_ip.c_str());
    robot_ = std::make_unique<FrankaRobot>(robot_ip, getLogger());
    
    // my addition to get franka::robot model //
    model_ = std::make_unique<franka::Model>(robot_->robot_->loadModel());
    // const auto initial_state = robot_->read();
    // gravity_array = model.gravity(initial_state);
    //             my addition                 //


  } catch (const franka::Exception& e) {
    RCLCPP_FATAL(getLogger(), "Could not connect to robot");
    RCLCPP_FATAL(getLogger(), e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(getLogger(), "Successfully connected to robot");
  return CallbackReturn::SUCCESS;
}

rclcpp::Logger FrankaHardwareInterfaceNew::getLogger() {
  return rclcpp::get_logger("FrankaHardwareInterfaceNew");
}

hardware_interface::return_type FrankaHardwareInterfaceNew::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/) {
  if (not effort_interface_running_ and effort_interface_claimed_) {
    robot_->stopRobot();
    robot_->initializeTorqueControl();
    effort_interface_running_ = true;
  } else if (effort_interface_running_ and not effort_interface_claimed_) {
    robot_->stopRobot();
    robot_->initializeContinuousReading();
    effort_interface_running_ = false;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FrankaHardwareInterfaceNew::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) {
  auto is_effort_interface = [](const std::string& interface) {
    return interface.find(hardware_interface::HW_IF_EFFORT) != std::string::npos;
  };

  int64_t num_stop_effort_interfaces =
      std::count_if(stop_interfaces.begin(), stop_interfaces.end(), is_effort_interface);
  if (num_stop_effort_interfaces == kNumberOfJoints) {
    effort_interface_claimed_ = false;
  } else if (num_stop_effort_interfaces != 0) {
    RCLCPP_FATAL(this->getLogger(), "Expected %zu effort interfaces to stop, but got %zu instead.",
                 kNumberOfJoints, num_stop_effort_interfaces);
    std::string error_string = "Invalid number of effort interfaces to stop. Expected ";
    error_string += std::to_string(kNumberOfJoints);
    throw std::invalid_argument(error_string);
  }

  int64_t num_start_effort_interfaces =
      std::count_if(start_interfaces.begin(), start_interfaces.end(), is_effort_interface);
  if (num_start_effort_interfaces == kNumberOfJoints) {
    effort_interface_claimed_ = true;
  } else if (num_start_effort_interfaces != 0) {
    RCLCPP_FATAL(this->getLogger(), "Expected %zu effort interfaces to start, but got %zu instead.",
                 kNumberOfJoints, num_start_effort_interfaces);
    std::string error_string = "Invalid number of effort interfaces to start. Expected ";
    error_string += std::to_string(kNumberOfJoints);
    throw std::invalid_argument(error_string);
  }
  return hardware_interface::return_type::OK;
}


void FrankaHardwareInterfaceNew::calculate_twist(const std::array<double,7>& velocities,
                                                  const std::array<double,42>& jac_array){
    int k{0};
    for (int i{0}; i < 6; ++i){
      vel(i) = velocities[i];
        for (int j{0}; j < 7; ++j){
            jacobian(i,j) = jac_array[k];
            ++k;
        }
    }
    const auto ans = jacobian*vel;
    for (int i{0}; i < 6; ++i){
      hw_velocities_[i] = ans(i);
    }
}


void FrankaHardwareInterfaceNew::calculate_torque(const std::array<double, 7>& twist_dot){
  for (int i{0}; i < 6; ++i){
    d_twist(i) = twist_dot.data()[i];
  }
}


}  // namespace franka_hardware



#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_hardware::FrankaHardwareInterfaceNew,
                       hardware_interface::SystemInterface)
