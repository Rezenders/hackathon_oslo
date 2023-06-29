// Copyright 2023 Intelligent Robotics Lab
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

#include <string>
#include <iostream>
#include <algorithm>

#include "scenario_maps/IsBombNear.hpp"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "bombs_msgs/msg/bomb_detection.hpp"

#include "rclcpp/rclcpp.hpp"

namespace scenario_maps
{

using namespace std::chrono_literals;
using namespace std::placeholders;

IsBombNear::IsBombNear(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf)
{
  config().blackboard->get("node", node_);

  // FIX40 Create necessary Publishers and Subscripcions
  detection_sub_  = node_->create_subscription<bombs_msgs::msg::BombDetection>(
      "/bomb_detector", 10, std::bind(&IsBombNear::bomb_detector_callback, this, _1));
}

// FIX41 Update last_detections_
void
IsBombNear::bomb_detector_callback(const bombs_msgs::msg::BombDetection &msg)
{
  last_detections_[msg.bomb_id] = msg;
}

BT::NodeStatus
IsBombNear::tick()
{
  std::string bomb_id;
  getInput("bomb_id", bomb_id);

  // FIX42 get distance to the bomb
  double distance = last_detections_[bomb_id].distance;

  if (distance < 3.0) {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "IsBombNear SUCCESS");
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_DEBUG_STREAM(node_->get_logger(), "IsBombNear FAILURE");
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace scenario_maps

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<scenario_maps::IsBombNear>("IsBombNear");
}
