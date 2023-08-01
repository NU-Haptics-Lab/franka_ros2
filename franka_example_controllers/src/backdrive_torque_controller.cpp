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
        config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
        config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
    }
    for (int i{1}; i <= num_joints; ++i) {
        config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
    }
  return config;
}

controller_interface::return_type BackdriveTorqueController::update(const rclcpp::Time &, const rclcpp::Duration &) {
  updateJointStates();
  // q_ *= 0.5; // alpha = 0.1
  // trajectory_msgs::msg::JointTrajectoryPoint msg;
  // std_msgs::msg::Float64MultiArray publisher_msg;
  std_msgs::msg::Float64MultiArray tau_ext_msg;
  // std_msgs::msg::MultiArrayLayout ms
  tau_ext_msg.layout.dim[0].label = "tau_ext";
  tau_ext_msg.layout.dim[0].size = 7;
  tau_ext_msg.layout.dim[0].stride =0;

  std_msgs::msg::Float64MultiArray coriolis_msg;
  coriolis_msg.layout.dim[1].label = "coriolis";
  coriolis_msg.layout.dim[1].size = 7;
  coriolis_msg.layout.dim[1].stride = 0;

  std_msgs::msg::Float64MultiArray mass_matrix_msg;
  mass_matrix_msg.layout.dim[1].label = "mass_matrix";
  mass_matrix_msg.layout.dim[1].size = 49;
  mass_matrix_msg.layout.dim[1].stride = 0;

  std_msgs::msg::Float64MultiArray gravity_msg;
  gravity_msg.layout.dim[1].label = "gravity";
  gravity_msg.layout.dim[1].size = 7;
  gravity_msg.layout.dim[1].stride = 0;

  std_msgs::msg::Float64MultiArray tau_j_msg;
  tau_j_msg.layout.dim[1].label = "tau_j";
  tau_j_msg.layout.dim[1].size = 7;
  tau_j_msg.layout.dim[1].stride = 0;

  // test
  for (int i{0}; i < 7; ++i){
    tau_ext_msg.data[i] = tau_ext_(i);
    tau_j_msg.data[i] = tau_j_(i);
    coriolis_msg.data[i] = coriolis_gravity_mass(i);
    gravity_msg.data[i] = coriolis_gravity_mass(7*i);
  }
  int j{0};
  for (int i{14}; i < 63; ++i){
    mass_matrix_msg.data[j] = coriolis_gravity_mass(i);
    ++j;
  }

  tau_ext_pub_->publish(tau_ext_msg);
  mass_matrix_pub_->publish(mass_matrix_msg);
  coriolis_pub_->publish(coriolis_msg);
  gravity_pub_->publish(gravity_msg);
  tau_J_pub_->publish(tau_j_msg);


  // Vector7d tau_cmd = q_;
  // int j{0};
  // for (auto& command_interface : command_interfaces_) {
  //     tau_cmd(j) = -q_(j);
  //     //minimum joint torques for motion
  //     //joint 7 -> 0.6
  //     //joint 6 -> 0.5
  //     //joint 5 -> 0.9
  //     //joint 4 -> 0.5
  //     //joint 3 ->
  //     //joint 2 ->
  //     //joint 1 ->

  //   if (std::fabs(tau_cmd(j)) < max_tau_cmd(j)) {
  //     command_interface.set_value(tau_cmd(j));
  //   }
  //   else if(tau_cmd(j) > max_tau_cmd(j)) {
  //     command_interface.set_value(max_tau_cmd(j));
  //   }
  //   else if (tau_cmd(j) < -max_tau_cmd(j)) {
  //     command_interface.set_value(-max_tau_cmd(j));
  //   }
  //   msg.effort.push_back(command_interface.get_value());
  //   j++;
  // }
  // cmds_->publish(msg);
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
    // whole_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/everything", 10);

    tau_ext_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/tau_ext_f", 10);
    mass_matrix_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/mass_matrix", 10);
    coriolis_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/coriolis", 10);
    gravity_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/gravity", 10);
    tau_J_pub_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/tau_j", 10);

    // auto_declare<std::vector<double>>("k_gains", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }
  return controller_interface::return_type::OK;
}

void BackdriveTorqueController::updateJointStates() {
  for (auto i = 0; i < num_joints; ++i) {

    const auto& velocity_interface = state_interfaces_.at(3 * i+1);
    const auto& effort_interface = state_interfaces_.at(3 * i + 2);

    // assert(position_interface.get_interface_name() == "position");
    assert(velocity_interface.get_interface_name() == "velocity");
    assert(effort_interface.get_interface_name() == "effort");

    tau_ext_(i) = effort_interface.get_value();
    tau_j_(i) = velocity_interface.get_value();
  }

  for (auto i{0}; i < 63; ++i){
    const auto& position_interface = state_interfaces_.at(3*i);
    assert(position_interface.get_interface_name() == "position");
    
    coriolis_gravity_mass(i) = position_interface.get_value();
  }
}

}  // namespace franka_example_controllers
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::BackdriveTorqueController,
                       controller_interface::ControllerInterface)
