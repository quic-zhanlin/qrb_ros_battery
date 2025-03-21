/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "qrb_battery_client/battery_client.hpp"

#include <iostream>


int main(int argc, char ** argv)
{
  qrb::battery_client::BatteryClient bc;
  bc.InitConnection();
  while (1) {
    char input;
    std::cin >> input;
    if (input != 's') {
      break;
    }
    std::unique_ptr<std::string> data;
    auto ret = bc.GetBatteryStats(data);
    if (!ret) {
      std::cout << "Get Battery Stats Failed" << std::endl;
      std::cout << std::endl;
      break;
    }
    std::cout << "Battery Stats:: " << *data << std::endl;
    std::cout << std::endl;
    std::cout << "Type 's' to get battery stats:: " << std::endl;
  }
  bc.CloseConnection();
  return 0;
}