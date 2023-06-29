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


#ifndef SCENARIO_TRAIN__ARTIFICIER_HPP_
#define SCENARIO_TRAIN__ARTIFICIER_HPP_

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
#include "rclcpp/macros.hpp"

namespace scenario_train
{

class Artificier : public rclcpp::Node
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(Artificier)

  Artificier(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  void control_cycle();

  // FIX05 Add callbacks Subscripcions
  void bomb_detector_callback(const bombs_msgs::msg::BombDetection &msg);

  void disable_bomb(const bombs_msgs::msg::BombDetection & bomb);
  bool is_visible(const bombs_msgs::msg::BombDetection & bomb);
  void approach_bomb(const bombs_msgs::msg::BombDetection & bomb);
  void reactive_navigate_bomb(const bombs_msgs::msg::BombDetection & bomb);

private:
  // FIX04 Add Publishers and Subscripcions
  rclcpp::Subscription<bombs_msgs::msg::BombDetection>::SharedPtr detection_sub_;
  rclcpp::Publisher<bombs_msgs::msg::OperateBomb>::SharedPtr bomb_operation_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  // FIX06 Add Timer fo the control cycle
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  tf2::BufferCore tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::vector<std::string> bombs_;
  std::vector<std::string> codes_;

  // std::vector<bombs_msgs::msg::BombDetection> last_detections_;
  std::map<std::string, bombs_msgs::msg::BombDetection> last_detections_;
  // std::vector<bombs_msgs::msg::BombDetection> last_detections_;

  double last_distance_ {100.0}, last_dx_ {0.0}, last_dv_{0.0};
  geometry_msgs::msg::Twist last_vel_;
};

}  // namespace scenario_train

#endif  // SCENARIO_TRAIN__ARTIFICIER_HPP_
