#pragma once
#include "px4_comm_types.hpp"
#include "px4_interface/msg/battery_status.hpp"
#include "px4_interface/msg/position_ned.hpp"
#include "px4_interface/msg/vehicle_status.hpp"

namespace MsgConverters {

// 将缓存的 VehicleStatus 转换为 ROS 消息
inline px4_interface::msg::VehicleStatus convert(
    const px4Status::VehicleStatus &status) {
  px4_interface::msg::VehicleStatus msg;
  msg.valid = status.valid;
  msg.latest_timestamp = status.latest_timestamp;
  msg.arming_state = status.arming_state;
  msg.nav_state = status.nav_state;
  msg.failsafe = status.failsafe;
  msg.pre_flight_checks_pass = status.pre_flight_checks_pass;
  return msg;
}
// 将缓存的 PositionNED 转换为 ROS 消息
inline px4_interface::msg::PositionNED convert(
    const px4Position::BasicPosition<px4Position::FrameNED> &position) {
  px4_interface::msg::PositionNED msg;
  msg.valid = position.valid;
  msg.translation = {position.translation.x(), position.translation.y(),
                     position.translation.z()};
  msg.orientation = {position.orientation.w(), position.orientation.x(),
                     position.orientation.y(), position.orientation.z()};
  msg.timestamp = position.timestamp;
  return msg;
}
// 将缓存的 BatteryStatus 转换为 ROS 消息
inline px4_interface::msg::BatteryStatus convert(
    const px4Status::BatteryStatus &battery) {
  px4_interface::msg::BatteryStatus msg;
  msg.valid = battery.valid;
  msg.timestamp = battery.timestamp;
  msg.voltage_v = battery.voltage_v;
  msg.current_a = battery.current_a;
  msg.remaining = battery.remaining;
  msg.warning = battery.warning;
  return msg;
}
}  // namespace MsgConverters