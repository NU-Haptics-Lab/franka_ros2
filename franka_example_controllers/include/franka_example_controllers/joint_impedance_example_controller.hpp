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

#include <string>
#include <trajectory_msgs/msg/joint_trajectory_point.h>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

namespace franka_example_controllers {

/**
 * The joint impedance example controller moves joint 4 and 5 in a very compliant periodic movement.
 */
class JointImpedanceExampleController : public controller_interface::ControllerInterface {
 public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;
    controller_interface::return_type init(const std::string & controller_name,
                                           const std::string & namespace_="",
                                           const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions()) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    /// added to make this compile, don't know if it is correct
   CallbackReturn on_init() override { return CallbackReturn::SUCCESS; };
 private:
  std::string arm_id_;
  const int num_joints = 7;
  Vector7d q_;
  Vector7d initial_q_;
  Vector7d desired_iq_{1.69469, -1.7544, -0.811505, -1.50818, -1.65261, 2.22612, -2.25689};
  // Vector7d steps;
  Vector7d dq_;
  Vector7d dq_filtered_;
  Vector7d k_gains_;
  Vector7d d_gains_;
  rclcpp::Time start_time_;
  int i{0};
  void updateJointStates();
};

}  // namespace franka_example_controllers
