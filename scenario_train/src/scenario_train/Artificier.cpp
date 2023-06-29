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


#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/transform_datatypes.h"
#include "tf2/LinearMath/Quaternion.h"

#include "visualization_msgs/msg/marker_array.hpp"
#include "bombs_msgs/msg/operate_bomb.hpp"
#include "bombs_msgs/msg/bomb_detection.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

#include "scenario_train/Artificier.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

namespace scenario_train
{

Artificier::Artificier(const rclcpp::NodeOptions & options)
: Node("artificier", options),
  tf_buffer_(),
  tf_listener_(tf_buffer_)
{
  // FIX07 Create Publishers and Subscripcions
  detection_sub_  = this->create_subscription<bombs_msgs::msg::BombDetection>(
      "/bomb_detector", 10, std::bind(&Artificier::bomb_detector_callback, this, _1));

  bomb_operation_pub_ = this->create_publisher<bombs_msgs::msg::OperateBomb>("/bombs_operation", 10);
  vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/key_vel", 10);

  // FIX08 Create Timers
  timer_ =  this->create_wall_timer(
      500ms, std::bind(&Artificier::control_cycle, this));

  // FIX09 Read bombs and codes parameters
  declare_parameter("bombs", bombs_);
  declare_parameter("codes", codes_);
  get_parameter("bombs", bombs_);
  get_parameter("codes", codes_);
}

// FIX10 Update last_detections_
void
Artificier::bomb_detector_callback(const bombs_msgs::msg::BombDetection &msg)
{
  last_detections_[msg.bomb_id] = msg;
  // last_detections_.push_back(msg);
}

void
Artificier::control_cycle()
{
  if (last_detections_.empty()) {
    return;
  }

  // FIX11 Select bomb to disable
  bombs_msgs::msg::BombDetection bomb_to_disable;
  float min_dist = 9999.0;
  for(auto bomb: last_detections_){
    if (bomb.second.distance < min_dist && bomb.second.status == bombs_msgs::msg::BombDetection::ENABLED){
      bomb_to_disable = bomb.second;
      min_dist = bomb.second.distance;
    }
  }

  std::cerr << "Bomb selected: " << bomb_to_disable.bomb_id << std::endl;

  if (bomb_to_disable.distance < 3.0) {
    disable_bomb(bomb_to_disable);
  }

  if (is_visible(bomb_to_disable)) {
    approach_bomb(bomb_to_disable);
  } else {
    reactive_navigate_bomb(bomb_to_disable);
  }
}

void
Artificier::disable_bomb(const bombs_msgs::msg::BombDetection & bomb)
{
  // FIX12 Send a message to disable a bomb
  if (codes_.empty()) {
    RCLCPP_WARN_STREAM(get_logger(), "No codes found for disabling");
    return;
  }

  bombs_msgs::msg::OperateBomb msg;
  msg.operation = bombs_msgs::msg::OperateBomb::DEACTIVATE;
  msg.bomb_id = bomb.bomb_id;

  for (size_t i = 0; i < bombs_.size(); i++) {
    if (bombs_[i] == bomb.bomb_id) {
      msg.code = bombs_[i] + "_" + codes_[i];
    }
  }

  bomb_operation_pub_->publish(msg);
}

bool
Artificier::is_visible(const bombs_msgs::msg::BombDetection & bomb)
{
  // FIX13 Check if bomb is visible (there is a TF to it)
  geometry_msgs::msg::TransformStamped robot2bomb_msg;
  try {
    robot2bomb_msg = tf_buffer_.lookupTransform(
      "base_footprint", bomb.bomb_id, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    return false;
  }

  return (now() - rclcpp::Time(robot2bomb_msg.header.stamp)) < 2s;
}

void
Artificier::approach_bomb(const bombs_msgs::msg::BombDetection & bomb)
{
  geometry_msgs::msg::TransformStamped robot2bomb_msg;
  try {
    robot2bomb_msg = tf_buffer_.lookupTransform(
      "base_footprint", bomb.bomb_id, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    return;
  }

  const double & x = robot2bomb_msg.transform.translation.x;
  const double & y = robot2bomb_msg.transform.translation.y;

  geometry_msgs::msg::Twist msg;
  msg.linear.x = 1.0;
  msg.angular.z = atan2(y, x);

  vel_pub_->publish(msg);
}

void
Artificier::reactive_navigate_bomb(const bombs_msgs::msg::BombDetection & bomb)
{
  // FIX15 send speed commands to the robot to find the bomb, only based on distances
  double dx = bomb.distance - last_distance_;

  if ( abs(dx) < 0.00001) {
    vel_pub_->publish(last_vel_);
    return;
  }

  geometry_msgs::msg::Twist msg;
  msg.linear.x = 1.0;

  double dv = last_dx_ - dx;
  double da = last_dv_ - dv;

  if (dx > 0.0) {
    msg.angular.z = 1.0;
  } else {
    if (dv > 0.0) {
      msg.angular.z = -last_vel_.angular.z;
    } else {
      if (da > 0.0) {
        msg.angular.z = -last_vel_.angular.z * fabs(dv);
      } else {
        msg.angular.z = last_vel_.angular.z * fabs(dv);
      }
    }
  }

  last_dv_ = dv;
  last_dx_ = dx;
  last_distance_ = bomb.distance;

  msg.angular.z = std::clamp(msg.angular.z, -1.0, 1.0);
  if (fabs(msg.angular.z) < 0.01) msg.angular.z = (std::signbit(msg.angular.z)? 1.0: -1.0) * 0.01;

  RCLCPP_DEBUG_STREAM(
    get_logger(),
    "[" << bomb.distance << "] [" << dx << "] [" << dv << "] [" << da << "] -> " << msg.angular.z);

  last_vel_ = msg;
  vel_pub_->publish(msg);
}

}  // namespace scenario_train

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(scenario_train::Artificier)
