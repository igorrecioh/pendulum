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
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "pendulum_controller_node/pendulum_controller_node.hpp"

TEST(FirstTest, verify_namespace) {
  rclcpp::init(0, nullptr);

  auto pendulum_driver = std::make_shared<
    pendulum::PendulumControllerNode>(rclcpp::NodeOptions());

  EXPECT_STREQ("/", pendulum_driver->get_namespace());
}
