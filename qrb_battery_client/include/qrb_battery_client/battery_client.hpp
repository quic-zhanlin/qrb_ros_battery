/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#ifndef QRB_BATTERY_CLIEN__BATTERY__CLIENT_HPP
#define QRB_BATTERY_CLIEN__BATTERY__CLIENT_HPP

#include <dbus/dbus.h>
#include <memory>
#include <string.h>

namespace qrb
{
namespace battery_client
{
class BatteryClient
{
public:
  bool InitConnection();
  void CloseConnection();
  bool GetBatteryStats(std::unique_ptr<std::string> & msg);
  ~BatteryClient();

private:
  bool GetServerMsg(std::unique_ptr<std::string> & msg);
  void PrintDusError(DBusError & dbus_error);
  DBusConnection * _conn = NULL;
};
}  // namespace battery_client

}  // namespace qrb

#endif
