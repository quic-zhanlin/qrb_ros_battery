/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_BATTERY_CLIEN__BATTERY__CLIENT_HPP
#define QRB_BATTERY_CLIEN__BATTERY__CLIENT_HPP

#include <dbus/dbus.h>
#include <string.h>

#include <memory>

namespace qrb
{
namespace battery_client
{
class BatteryClient
{
public:
  bool init_connection();
  void close_connection();
  bool get_battery_stats(std::unique_ptr<std::string> & msg);
  ~BatteryClient();

private:
  bool get_server_msg(std::unique_ptr<std::string> & msg);
  void print_dus_error(DBusError & dbus_error);
  DBusConnection * _conn = NULL;
};
}  // namespace battery_client

}  // namespace qrb

#endif
