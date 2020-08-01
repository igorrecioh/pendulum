// Copyright 2020 Igor Recio
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

#include <gtest/gtest.h>
#include "rclcpp/rclcpp.hpp"
#include "pendulum_driver/pendulum_driver_node.hpp"
#include "pendulum_controller_node/pendulum_controller_node.hpp"
#include "pendulum_controller_node/pendulum_controller.hpp"
#include "pendulum_controllers/full_state_feedback_controller.hpp"
#include "pendulum_simulation/pendulum_simulation.hpp"

static const size_t DEFAULT_DEADLINE_PERIOD_US = 2000;
static const size_t DEFAULT_CONTROLLER_UPDATE_PERIOD_US = 1000;
static const size_t DEFAULT_PHYSICS_UPDATE_PERIOD_US = 1000;

TEST(DummyTest, not_equal) {
  std::chrono::microseconds deadline_duration(DEFAULT_DEADLINE_PERIOD_US);

  // controller options
  std::chrono::microseconds controller_update_period(DEFAULT_CONTROLLER_UPDATE_PERIOD_US);
  std::vector<double> feedback_matrix = {-10.0000, -51.5393, 356.8637, 154.4146};

  // driver options
  std::chrono::microseconds physics_update_period(DEFAULT_PHYSICS_UPDATE_PERIOD_US);

  // set QoS deadline period
  rclcpp::QoS qos_deadline_profile(10);
  qos_deadline_profile.deadline(deadline_duration);

  // Create a controller
  std::unique_ptr<pendulum::PendulumController> controller = std::make_unique<
    pendulum::FullStateFeedbackController>(feedback_matrix);

  // Create pendulum simulation
  std::unique_ptr<pendulum::PendulumDriverInterface> sim =
    std::make_unique<pendulum::PendulumSimulation>(physics_update_period);

  // Create pendulum controller node
  pendulum::PendulumControllerOptions controller_options;
  controller_options.node_name = "pendulum_controller";
  controller_options.command_publish_period = controller_update_period;
  controller_options.status_qos_profile = qos_deadline_profile;
  controller_options.command_qos_profile = qos_deadline_profile;
  controller_options.setpoint_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

  // Create pendulum driver node
  pendulum::PendulumDriverOptions driver_options;
  driver_options.node_name = "pendulum_driver";
  // set sensor publishing period equal to simulation physics update period
  driver_options.status_publish_period = physics_update_period;
  driver_options.status_qos_profile = qos_deadline_profile;

  auto pendulum_driver = std::make_shared<pendulum::PendulumDriverNode>(
    std::move(sim),
    driver_options,
    rclcpp::NodeOptions().use_intra_process_comms(true));

  EXPECT_EQ(1, 1);
}
