#include "px4_interface/px4_gateway.hpp"
#include "px4_interface/msg_converters.hpp"
#include "px4_interface/px4_msgs_cache.hpp"
#include <exception>
#include <memory>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

PX4Gateway::PX4Gateway(const rclcpp::NodeOptions &options,
                       std::shared_ptr<Px4MsgsCache> cache)
    : Node("PX4_Gateway", options), px4_msgs_cache_(std::move(cache)) {
  if (px4_msgs_cache_ == nullptr) {
    throw std::invalid_argument("Px4MsgsCache pointer cannot be null");
  }
  if (!rclcpp::ok()) {
    throw std::runtime_error("ROS2 is not initialized");
  }
  init();
  RCLCPP_INFO(this->get_logger(), "PX4Gateway node initialized");
}

bool PX4Gateway::checkPx4Publishers() const {
  auto publishers =
      this->get_publishers_info_by_topic("/fmu/out/vehicle_status");
  if (publishers.empty()) {
    RCLCPP_WARN(this->get_logger(),
                "No publishers found for /fmu/out/vehicle_status.Check PX4 "
                "connection.");
    return false;
  }
  return true;
}

template <typename Data, typename PublisherPtr>
void publish(const Data &data, PublisherPtr publisher) {
  auto msg = MsgConverters::convert(data);
  publisher->publish(msg);
}

void PX4Gateway::publishCache() const {
  publish(px4_msgs_cache_->getVehicleStatus(), vehicle_status_publisher_);
  publish(px4_msgs_cache_->getPositionNED(), vehicle_local_position_publisher_);
  publish(px4_msgs_cache_->getBatteryStatus(), battery_status_publisher_);
}

void PX4Gateway::chooseControlMethod(bool position_control,
                                     bool velocity_control) {
  offboard_position_control_ = position_control;
  offboard_velocity_control_ = velocity_control;
}

void PX4Gateway::setTarget(const px4GatewayTypes::setpoint &target) {
  px4_msgs::msg::TrajectorySetpoint
      traj_setpoint_msg; // Todo: 优化，这里的转化过于机械
  traj_setpoint_msg.position[0] = target.position[0];
  traj_setpoint_msg.position[1] = target.position[1];
  traj_setpoint_msg.position[2] = target.position[2];
  traj_setpoint_msg.velocity[0] = target.velocity[0];
  traj_setpoint_msg.velocity[1] = target.velocity[1];
  traj_setpoint_msg.velocity[2] = target.velocity[2];
  traj_setpoint_msg.yaw = target.yaw;
  traj_setpoint_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  trajectory_setpoint_publisher_->publish(traj_setpoint_msg);
}

void PX4Gateway::setTarget(const px4_msgs::msg::TrajectorySetpoint &target) {
  trajectory_setpoint_publisher_->publish(target);
}

void PX4Gateway::publishVehicleCommand(const px4Enum::VehicleCommand command,
                                       const float param1, const float param2) {
  px4_msgs::msg::VehicleCommand vehicle_command_msg;
  vehicle_command_msg.command = static_cast<uint32_t>(command);
  vehicle_command_msg.param1 = param1;
  vehicle_command_msg.param2 = param2;
  vehicle_command_msg.target_system = 1;    // 通常为1
  vehicle_command_msg.target_component = 1; // 通常为1
  vehicle_command_msg.source_system = 1;    // 通常为1
  vehicle_command_msg.source_component = 1; // 通常为1
  vehicle_command_msg.from_external = true; // 来自外部控制器
  vehicle_command_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  vehicle_command_publisher_->publish(vehicle_command_msg);
}

void PX4Gateway::setArmMode() {
  publishVehicleCommand(
      px4Enum::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f, 0.0f);
  RCLCPP_INFO(this->get_logger(), "Sent ARM command to PX4");
}

void PX4Gateway::setDisarmMode() {
  publishVehicleCommand(
      px4Enum::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f, 0.0f);
  RCLCPP_INFO(this->get_logger(), "Sent DISARM command to PX4");
}

void PX4Gateway::setOffboardMode() {
  publishVehicleCommand(px4Enum::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f,
                        6.0f);
  RCLCPP_INFO(this->get_logger(), "Sent OFFBOARD mode command to PX4");
}

void PX4Gateway::setLandMode() {
  publishVehicleCommand(px4Enum::VehicleCommand::VEHICLE_CMD_NAV_LAND);
  RCLCPP_INFO(this->get_logger(), "Sent LAND command to PX4");
}

void PX4Gateway::triggerEmergencyStop() {
  publishVehicleCommand(
      px4Enum::VehicleCommand::VEHICLE_CMD_DO_FLIGHTTERMINATION, 1.0f);
  RCLCPP_WARN(this->get_logger(), "Sent EMERGENCY STOP command to PX4!");
  RCLCPP_WARN(this->get_logger(), "Motors will stop immediately!");
}

void PX4Gateway::init() {
  // 提供给PX4的发布者
  offboard_control_mode_publisher_ =
      this->create_publisher<px4_msgs::msg::OffboardControlMode>(
          "/fmu/in/offboard_control_mode", 10);
  trajectory_setpoint_publisher_ =
      this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
          "/fmu/in/trajectory_setpoint", 10);
  vehicle_command_publisher_ =
      this->create_publisher<px4_msgs::msg::VehicleCommand>(
          "/fmu/in/vehicle_command", 10); // 发布车辆控制命令

  // 提供给外部的发布者
  vehicle_status_publisher_ =
      this->create_publisher<px4_interface::msg::VehicleStatus>(
          "/cache/vehicle_status", 10);
  vehicle_local_position_publisher_ =
      this->create_publisher<px4_interface::msg::PositionNED>(
          "/cache/vehicle_local_position", 10);
  battery_status_publisher_ =
      this->create_publisher<px4_interface::msg::BatteryStatus>(
          "/cache/battery_status", 10);

  // 创建订阅者
  auto qos_profile = rclcpp::SystemDefaultsQoS();
  vehicle_status_subscriber_ =
      this->create_subscription<px4_msgs::msg::VehicleStatus>(
          "/fmu/out/vehicle_status", qos_profile,
          [this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
            px4Status::VehicleStatus status;
            status.valid = true;
            status.latest_timestamp = this->get_clock()->now();
            status.arming_state = msg->arming_state;
            status.nav_state = msg->nav_state;
            status.failsafe = msg->failsafe;
            status.pre_flight_checks_pass = msg->pre_flight_checks_pass;
            px4_msgs_cache_->updateVehicleStatus(status);
          });
  vehicle_local_position_subscriber_ =
      this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
          "/fmu/out/vehicle_local_position", qos_profile,
          [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
            px4Position::BasicPosition<px4Position::FrameNED> position;
            position.valid = msg->xy_valid && msg->z_valid;
            position.translation = Eigen::Vector3d(msg->x, msg->y, msg->z);
            position.orientation =
                Eigen::Quaterniond::Identity(); // PX4不提供姿态
            position.timestamp = this->get_clock()->now();
            px4_msgs_cache_->updatePositionNED(position);
          });
  battery_status_subscriber_ =
      this->create_subscription<px4_msgs::msg::BatteryStatus>(
          "/fmu/out/battery_status", qos_profile,
          [this](const px4_msgs::msg::BatteryStatus::SharedPtr msg) {
            px4Status::BatteryStatus battery;
            battery.valid = true;
            battery.timestamp = this->get_clock()->now();
            battery.voltage_v = msg->voltage_v;
            battery.current_a = msg->current_a;
            battery.remaining = msg->remaining;
            battery.warning = msg->warning;
            px4_msgs_cache_->updateBatteryStatus(battery);
          });
  // 创建定时器，定期发布OffboardControlMode(50Hz)
  offboard_control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / offboard_control_rate_hz_), [this]() {
        publishOffboardControlMode(offboard_position_control_,
                                   offboard_velocity_control_);
      });
  // 创建定时器，定期发布缓存数据(10Hz)
  cache_publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                                 [this]() { publishCache(); });
}

void PX4Gateway::publishOffboardControlMode(const bool position_control,
                                            const bool velocity_control) {
  px4_msgs::msg::OffboardControlMode offboard_msg;
  offboard_msg.position = position_control; // 位置控制
  offboard_msg.velocity = velocity_control; // 速度控制
  offboard_msg.acceleration = false;
  offboard_msg.attitude = false;
  offboard_msg.body_rate = false;
  offboard_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  offboard_control_mode_publisher_->publish(offboard_msg);
}
