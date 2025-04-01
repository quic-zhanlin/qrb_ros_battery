/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_ROS_BATTERY__BATTERY_NODE_HPP_
#define QRB_ROS_BATTERY__BATTERY_NODE_HPP_

#include <map>
#include <memory>
#include <sensor_msgs/msg/battery_state.hpp>
#include <string>

#include "qrb_battery_client/battery_client.hpp"
#include "rclcpp/rclcpp.hpp"

namespace qrb_ros
{
namespace battery
{
class BatteryStatsReader : public rclcpp::Node
{
public:
  BatteryStatsReader();

private:
  void timer_callback();
  void pack_battery_state_msg(std::map<std::string, std::string> & data,
      std::unique_ptr<sensor_msgs::msg::BatteryState> & msg);
  std::map<std::string, std::string> string_to_map(std::unique_ptr<std::string> str);
  float safestof(std::string & str);

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr publisher_;
  qrb::battery_client::BatteryClient bc_;
};
}  // namespace battery
}  // namespace qrb_ros
#endif  // BATTERYNODE_HPP
