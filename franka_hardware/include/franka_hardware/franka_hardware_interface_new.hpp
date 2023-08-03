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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <franka/model.h>

#include <hardware_interface/visibility_control.h>
#include <franka_hardware/robot_new.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"
#include <Eigen/Eigen>

namespace franka_hardware {

class FrankaHardwareInterfaceNew
    : public hardware_interface::SystemInterface {
 public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  hardware_interface::return_type prepare_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string>& start_interfaces,
      const std::vector<std::string>& stop_interfaces) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn  on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  static const size_t kNumberOfJoints = 7;
  // int j{0};
  std::unique_ptr<FrankaRobot> robot_;

  

 private:
  
  std::array<double, kNumberOfJoints> tau_bias{0, 0, 0, 0, 0, 0, 0};
  std::unique_ptr<franka::Model> model_;
  std::array<double, kNumberOfJoints*9> hw_positions_{0};
  std::array<double, kNumberOfJoints> hw_command_0_{0, 0, 0, 0, 0, 0, 0};
  std::array<double, kNumberOfJoints> hw_commands_{0, 0, 0, 0, 0, 0, 0};
  // std::array<double, kNumberOfJoints> hw_positions_{0, 0, 0, 0, 0, 0, 0};
 
  std::array<double, kNumberOfJoints> hw_efforts_{0, 0, 0, 0, 0, 0, 0};


  
  
  std::array<double, kNumberOfJoints> hw_coriolis_{0, 0, 0, 0, 0, 0, 0};
  std::array<double, kNumberOfJoints> gravity_array_{0, 0, 0, 0, 0, 0, 0};
  std::array<double, 49> mass_matrix_{};

  // Twist calculation variables
  std::array<double, kNumberOfJoints*6> jacobian_array_{0, 0, 0, 0, 0, 0, 0};
  Eigen::Matrix<double, 6,7> jacobian = Eigen::Matrix<double, 6, 7>::Zero();
  Eigen::Vector<double, 7> vel{0,0,0,0,0,0,0};
  std::array<double, kNumberOfJoints> hw_velocities_{0, 0, 0, 0, 0, 0, 0};
std::array<double, 6> F_tip_{0, 0, 0, 0, 0, 0};

  //acceleration calculation variables
  Eigen::Vector<double,6> d_twist{0,0,0,0,0,0,0};

  bool effort_interface_claimed_ = false;
  bool effort_interface_running_ = false;
  static rclcpp::Logger getLogger();

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gravity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr mass_matrix_pub_;

  void calculate_twist(const std::array<double,7>& velocities, const std::array<double,42>& jac_array);
  void calculate_torque(const std::array<double, 7>& twist_dot);
};

}  // namespace franka_hardware
