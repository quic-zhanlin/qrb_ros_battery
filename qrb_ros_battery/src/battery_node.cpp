/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "qrb_ros_battery/battery_node.hpp"

#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

using namespace std::chrono_literals;
using namespace qrb_ros::battery;

// This example creates a subclass of Node and uses std::bind() to register a
// member function as a callback from the timer. */

BatteryStatsReader::BatteryStatsReader() : Node("battery_stats_publisher")
{
  RCLCPP_INFO(this->get_logger(), "Started Battery Node");
  publisher_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_stats", 10);
  bool ret = bc_.InitConnection();
  if (!ret) {
    RCLCPP_ERROR(this->get_logger(), "Battery client connect failed");
  } else {
    timer_ = this->create_wall_timer(2000ms, std::bind(&BatteryStatsReader::timer_callback, this));
  }
}

float BatteryStatsReader::safestof(std::string & str)
{
  if (str == "EMPTY")
    return -1;
  return std::stof(str);
}

void BatteryStatsReader::pack_battery_state_msg(std::map<std::string, std::string> & data,
    std::unique_ptr<sensor_msgs::msg::BatteryState> & msg)
{
  msg->header.stamp = this->now();
  msg->header.frame_id = "battery_stats";

  msg->voltage = safestof(data["voltage_now"]);                 // 0
  msg->temperature = safestof(data["temp"]);                    // 0
  msg->current = safestof(data["current_now"]);                 // 0
  msg->charge = safestof(data["charge_counter"]);               // 0
  msg->capacity = safestof(data["capacity"]);                   // 50
  msg->design_capacity = safestof(data["charge_full_design"]);  // 0
  msg->percentage = safestof(data["capacity"]);                 // 50

  // setting Power Supply Status constants
  if (data["status"] == "Charging")
    msg->power_supply_status = 1;
  else if (data["status"] == "Discharging")
    msg->power_supply_status = 2;
  else if (data["status"] == "Not charging")
    msg->power_supply_status = 3;
  else if (data["status"] == "Full")
    msg->power_supply_status = 4;
  else
    msg->power_supply_status = 0;

  // setting Power Supply Health constants
  if (data["health"] == "Unknown")
    msg->power_supply_health = 0;
  else if (data["health"] == "Good")
    msg->power_supply_health = 1;
  else if (data["health"] == "Overheat")
    msg->power_supply_health = 2;
  else if (data["health"] == "Dead")
    msg->power_supply_health = 3;
  else if (data["health"] == "Over voltage")
    msg->power_supply_health = 4;
  else if (data["health"] == "Unspecified failure")
    msg->power_supply_health = 5;
  else if (data["health"] == "Cold")
    msg->power_supply_health = 6;
  else if (data["health"] == "Watchdog timer expire")
    msg->power_supply_health = 7;
  else if (data["health"] == "Safety timer expire")
    msg->power_supply_health = 8;

  // setting Power Supply Technology constants
  if (data["technology"] == "Unknown")
    msg->power_supply_technology = 0;
  else if (data["technology"] == "NiMH")
    msg->power_supply_technology = 1;
  else if (data["technology"] == "Li-ion")
    msg->power_supply_technology = 2;
  else if (data["technology"] == "Li-poly")
    msg->power_supply_technology = 3;
  else if (data["technology"] == "LiFe")
    msg->power_supply_technology = 4;
  else if (data["technology"] == "NiCd")
    msg->power_supply_technology = 5;
  else if (data["technology"] == "LiMn")
    msg->power_supply_technology = 6;

  msg->present = std::stoi(data["present"]);  // 1

  msg->location = "battery_stats";
  msg->serial_number = "battery_stats";
  msg->cell_voltage.push_back(safestof(data["voltage_now"]));  // 0
  msg->cell_temperature.push_back(safestof(data["temp"]));     // 0
}

std::map<std::string, std::string> BatteryStatsReader::string_to_map(std::string & str)
{
  std::map<std::string, std::string> result;
  std::istringstream iss(str);

  std::string line;
  while (std::getline(iss, line)) {
    size_t delimiterPos = line.find(':');
    if (delimiterPos != std::string::npos) {
      std::string key = line.substr(0, delimiterPos);
      std::string value = line.substr(delimiterPos + 1);
      result[key] = value;
    }
  }

  return result;
}

void BatteryStatsReader::timer_callback()
{
  RCLCPP_DEBUG(this->get_logger(), "begin timer callback");
  std::unique_ptr<std::string> data;
  auto msg = std::make_unique<sensor_msgs::msg::BatteryState>();
  auto ret = bc_.GetBatteryStats(data);
  RCLCPP_DEBUG(this->get_logger(), "get battery state data: %s", (*data).c_str());

  auto bstatmap = string_to_map(*data);

  pack_battery_state_msg(bstatmap, msg);
  publisher_->publish(std::move(msg));
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryStatsReader>());
  rclcpp::shutdown();
  return 0;
}
