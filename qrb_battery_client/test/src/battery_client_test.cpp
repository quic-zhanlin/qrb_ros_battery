/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "qrb_battery_client/battery_client.hpp"

#include <iostream>

int main(int argc, char ** argv)
{
  qrb::battery_client::BatteryClient bc;
  bc.init_connection();
  while (1) {
    char input;
    std::cin >> input;
    if (input != 's') {
      break;
    }
    std::unique_ptr<std::string> data;
    auto ret = bc.get_battery_stats(data);
    if (!ret) {
      std::cout << "Get Battery Stats Failed" << std::endl;
      std::cout << std::endl;
      break;
    }
    std::cout << "Battery Stats:: " << *data << std::endl;
    std::cout << std::endl;
    std::cout << "Type 's' to get battery stats:: " << std::endl;
  }
  bc.close_connection();
  return 0;
}