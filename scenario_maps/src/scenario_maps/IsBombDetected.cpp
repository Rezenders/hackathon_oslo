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

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "rclcpp/rclcpp.hpp"

#include "scenario_maps/IsBombDetected.hpp"

namespace scenario_maps
{

using namespace std::chrono_literals;
using namespace std::placeholders;

IsBombDetected::IsBombDetected(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(xml_tag_name, conf),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  config().blackboard->get("node", node_);

  // FIX34 Create necessary Publishers and Subscripcions
  detection_sub_  = node_->create_subscription<bombs_msgs::msg::BombDetection>(
      "/bomb_detector", 10, std::bind(&IsBombDetected::bomb_detector_callback, this, _1));

  // FIX35 Read bombs and codes parameters
  // ...
  // don't forget global_frame, in case we are not in map frame
  node_->declare_parameter("bomb_poses_x", bomb_poses_x_);
  node_->declare_parameter("bomb_poses_y", bomb_poses_y_);
  node_->get_parameter("bomb_poses_x", bomb_poses_x_);
  node_->get_parameter("bomb_poses_y", bomb_poses_y_);

  node_->declare_parameter("bombs", bombs_);
  node_->declare_parameter("codes", codes_);
  node_->get_parameter("bombs", bombs_);
  node_->get_parameter("codes", codes_);

  node_->declare_parameter("global_frame", global_frame_);
  node_->get_parameter("global_frame", global_frame_);
}

// FIX36 Update last_detections_
void
IsBombDetected::bomb_detector_callback(const bombs_msgs::msg::BombDetection & msg)
{
  last_detections_[msg.bomb_id] = msg;
}


BT::NodeStatus
IsBombDetected::tick()
{

  if (last_detections_.empty()) {
    return BT::NodeStatus::FAILURE;
  }

  bombs_msgs::msg::BombDetection bomb_to_disable;
  float min_dist = 9999.0;
  for(auto bomb: last_detections_){
    if (bomb.second.distance < min_dist && bomb.second.status == bombs_msgs::msg::BombDetection::ENABLED){
      bomb_to_disable = bomb.second;
      min_dist = bomb.second.distance;
    }
  }
  geometry_msgs::msg::PoseStamped wp;
  wp.header.frame_id = global_frame_;
  wp.pose.orientation.w = 1.0;

  std::string _code;

  for (size_t i = 0; i < bombs_.size(); i++) {
    if (bombs_[i] == bomb_to_disable.bomb_id) {
      _code = bombs_[i] + "_" + codes_[i];
      wp.pose.position.x = bomb_poses_x_[i];
      wp.pose.position.y = bomb_poses_y_[i];
    }
  }
  setOutput("code", _code);
  setOutput("bomb_pose", wp);
  setOutput("bomb_id", bomb_to_disable.bomb_id);

  RCLCPP_DEBUG_STREAM(node_->get_logger(), "IsBombDetected SUCCESS");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace scenario_maps

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<scenario_maps::IsBombDetected>("IsBombDetected");
}
