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

#include <controller_interface/controller_interface.hpp>
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <Eigen/Core>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"

namespace franka_example_controllers {

/**
 * backdrivability controller (BackdriveTorque controller)
 */
class BackdriveTorqueController : public controller_interface::ControllerInterface {
 public:
    using Vector7d = Eigen::Vector<double, 7>;
    using Vector6d = Eigen::Vector<double, 6>;
    
    CallbackReturn on_init() override {return CallbackReturn::SUCCESS;};
    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::return_type init(const std::string& controller_name,
                                           const std::string & namespace_ = "",
                                           const rclcpp::NodeOptions & options = rclcpp::NodeOptions()
        ) override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & dur) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

 private:

  std::string arm_id_;
  const int num_joints = 7;
  double ext_torque_;
  Vector7d max_tau_cmd{1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 2.0};

  double lambda{100.0};
  double damping{0.0};
  Vector7d F_tip{0,0,0,0,0,0,0};
  Vector6d ftip{0,0,0,0,0,0};
  Vector7d twist{0,0,0,0,0,0,0};
  Vector6d mtwist{0,0,0,0,0,0};
  Vector6d d_twist{0,0,0,0,0,0};

  Eigen::Matrix<double, 6,6> inv_lambda = 1.0/lambda * Eigen::Matrix<double,6,6>::Identity();
  


//   Vector7d initial_q_;
  Vector7d k_gains_;
  void updateJointStates();
  
};
}  // namespace franka_example_controllers
