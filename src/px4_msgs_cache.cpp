#include "px4_interface/px4_msgs_cache.hpp"

void Px4MsgsCache::updatePositionNED(
    const px4Position::PositionNED& new_position) {
  position_ned_cache_.update(new_position);
}

void Px4MsgsCache::updateBatteryStatus(
    const px4Status::BatteryStatus& new_status) {
  battery_status_cache_.update(new_status);
}

void Px4MsgsCache::updateVehicleStatus(
    const px4Status::VehicleStatus& new_status) {
  vehicle_status_cache_.update(new_status);
}

px4Position::PositionNED Px4MsgsCache::getPositionNED() const {
  return position_ned_cache_.get();
}

px4Status::BatteryStatus Px4MsgsCache::getBatteryStatus() const {
  return battery_status_cache_.get();
}

px4Status::VehicleStatus Px4MsgsCache::getVehicleStatus() const {
  return vehicle_status_cache_.get();
}
