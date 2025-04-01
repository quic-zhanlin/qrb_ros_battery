/*
 * Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 */

#include "qrb_battery_client/battery_client.hpp"

#include <fstream>
#include <iostream>

const char * const SERVER_BUS_NAME = "in.battery_server";
const char * const INTERFACE_NAME = "in.battery_server.system";
const char * const SERVER_OBJECT_PATH_NAME = "/in/battery_system/server";
const char * const METHOD_NAME = "get_battery_stats";
const char * const MATCH_RULES = "type='method_call',interface='in.battery_server.system'";

namespace qrb
{
namespace battery_client
{
bool BatteryClient::InitConnection()
{
  DBusError dbus_error;
  dbus_error_init(&dbus_error);
  _conn = dbus_bus_get(DBUS_BUS_SYSTEM, &dbus_error);

  if (dbus_error_is_set(&dbus_error))
    PrintDusError(dbus_error);

  return _conn == NULL ? false : true;
}

BatteryClient::~BatteryClient()
{
  CloseConnection();
}

void BatteryClient::CloseConnection()
{
  if (_conn != NULL) {
    dbus_connection_unref(_conn);
  }
}

bool BatteryClient::GetServerMsg(std::unique_ptr<std::string> & msg)
{
  DBusMessage * request;
  DBusError error;
  DBusPendingCall * pending;
  dbus_error_init(&error);

  // Constructs a new message to invoke a method on a remote object
  if ((request = dbus_message_new_method_call(
           SERVER_BUS_NAME, SERVER_OBJECT_PATH_NAME, INTERFACE_NAME, METHOD_NAME)) == NULL) {
    fprintf(stderr, "Error in dbus_message_new_method_call\n");
    return false;
  }

  // Queues a message to send, as with dbus_connection_send()
  if (!dbus_connection_send_with_reply(_conn, request, &pending, -1)) {
    fprintf(stderr, "Error in dbus_connection_send_with_reply\n");
    return false;
  }
  if (pending == NULL) {
    fprintf(stderr, "pending return is NULL");
    return false;
  }

  dbus_connection_flush(_conn);

  // Release message
  dbus_message_unref(request);

  // Wait for reply
  dbus_pending_call_block(pending);

  // Get the reply
  DBusMessage * reply;
  if ((reply = dbus_pending_call_steal_reply(pending)) == NULL) {
    fprintf(stderr, "Error in dbus_pending_call_steal_reply");
    return false;
  }

  // Release pending call
  dbus_pending_call_unref(pending);

  char * raw_cstr;
  if (!dbus_message_get_args(reply, &error, DBUS_TYPE_STRING, &raw_cstr, DBUS_TYPE_INVALID)) {
    fprintf(stderr, "Did not get arguments in reply\n");
    return false;
  }

  msg = std::make_unique<std::string>(raw_cstr);

  dbus_message_unref(reply);

  return true;
}

bool BatteryClient::GetBatteryStats(std::unique_ptr<std::string> & msg)
{
  DBusError match_error;
  dbus_error_init(&match_error);
  dbus_bus_add_match(_conn, MATCH_RULES, &match_error);
  if (dbus_error_is_set(&match_error)) {
    PrintDusError(match_error);
    return false;
  }

  auto ret = GetServerMsg(msg);

  DBusError release_error;
  dbus_error_init(&release_error);
  if (dbus_bus_release_name(_conn, SERVER_BUS_NAME, &release_error) == -1) {
    if (dbus_error_is_set(&release_error)) {
      PrintDusError(release_error);
    }
  }

  return ret;
}

void BatteryClient::PrintDusError(DBusError & dbus_error)
{
  fprintf(stderr, "Get Dbus error: %s\n", dbus_error.message);
  dbus_error_free(&dbus_error);
}

}  // namespace battery_client
}  // namespace qrb