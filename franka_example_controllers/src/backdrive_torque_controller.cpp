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

#include <franka_example_controllers/backdrive_torque_controller.hpp>

#include <exception>
#include <string>

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
BackdriveTorqueController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_joints; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
    
  }
  return config;
}

controller_interface::InterfaceConfiguration
BackdriveTorqueController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (int i{1}; i <= num_joints; ++i) {
        config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
        config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
        config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
    }
  return config;
}

controller_interface::return_type BackdriveTorqueController::update(const rclcpp::Time &, const rclcpp::Duration &) {
  updateJointStates();
  for(int i{0}; i < 6; ++i){
    ftip(i) = F_tip(i);
    mtwist(i) = twist(i);
  }
  d_twist = inv_lambda * (ftip - damping*mtwist);
  int i{0};
  for (auto& command_interface : command_interfaces_) {
    if(i < 6){
      command_interface.set_value(d_twist(i));
      ++i;
    } else{
      command_interface.set_value(0);
    }
  }
  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BackdriveTorqueController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
    arm_id_ = get_node()->get_parameter("arm_id").as_string();
    // auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
    // for (int i{0}; i < num_joints; ++i) {
    //     k_gains_(i) = k_gains.at(i);
    // }
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
BackdriveTorqueController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
  // updateJointStates();
  // initial_q_ = q_;
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type BackdriveTorqueController::init(
    const std::string& controller_name, const std::string &, const rclcpp::NodeOptions &) {
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }

  try {
    auto_declare<std::string>("arm_id", "panda");

    // auto_declare<std::vector<double>>("k_gains", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

void BackdriveTorqueController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {

    const auto& position_interface = state_interfaces_.at(3 * i);
    const auto& velocity_interface = state_interfaces_.at(3 * i + 1);
    const auto& effort_interface = state_interfaces_.at(3 * i + 2);

    assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");
    assert(effort_interface.get_interface_name() == "effort");

    F_tip(i) = effort_interface.get_value();
    twist(i) = velocity_interface.get_value();
  }

  
  
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::BackdriveTorqueController,
                       controller_interface::ControllerInterface)
