#include "px4_interface/px4_gateway.hpp"
#include "px4_interface/px4_msgs_cache.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <gtest/gtest.h>
#include <memory>
#include <mutex>
#include <optional>
#include <px4_interface/msg/battery_status.hpp>
#include <px4_interface/msg/position_ned.hpp>
#include <px4_interface/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>

using namespace std::chrono_literals;

namespace {

class CacheSubscriberNode : public rclcpp::Node {
public:
  CacheSubscriberNode() : rclcpp::Node("px4_gateway_test_listener") {
    auto qos = rclcpp::QoS(10);
    vehicle_status_sub_ =
        this->create_subscription<px4_interface::msg::VehicleStatus>(
            "/cache/vehicle_status", qos,
            [this](const px4_interface::msg::VehicleStatus &msg) {
              std::lock_guard<std::mutex> lock(mutex_);
              vehicle_status_msg_ = msg;
            });

    position_sub_ = this->create_subscription<px4_interface::msg::PositionNED>(
        "/cache/vehicle_local_position", qos,
        [this](const px4_interface::msg::PositionNED &msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          position_msg_ = msg;
        });

    battery_sub_ = this->create_subscription<px4_interface::msg::BatteryStatus>(
        "/cache/battery_status", qos,
        [this](const px4_interface::msg::BatteryStatus &msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          battery_msg_ = msg;
        });
  }

  bool publishers_discovered() const {
    return vehicle_status_sub_->get_publisher_count() > 0 &&
           position_sub_->get_publisher_count() > 0 &&
           battery_sub_->get_publisher_count() > 0;
  }

  bool all_received() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return vehicle_status_msg_.has_value() && position_msg_.has_value() &&
           battery_msg_.has_value();
  }

  std::optional<px4_interface::msg::VehicleStatus> last_vehicle_status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return vehicle_status_msg_;
  }

  std::optional<px4_interface::msg::PositionNED> last_position() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return position_msg_;
  }

  std::optional<px4_interface::msg::BatteryStatus> last_battery() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return battery_msg_;
  }

private:
  rclcpp::Subscription<px4_interface::msg::VehicleStatus>::SharedPtr
      vehicle_status_sub_;
  rclcpp::Subscription<px4_interface::msg::PositionNED>::SharedPtr
      position_sub_;
  rclcpp::Subscription<px4_interface::msg::BatteryStatus>::SharedPtr
      battery_sub_;

  mutable std::mutex mutex_;
  std::optional<px4_interface::msg::VehicleStatus> vehicle_status_msg_;
  std::optional<px4_interface::msg::PositionNED> position_msg_;
  std::optional<px4_interface::msg::BatteryStatus> battery_msg_;
};

class Px4GatewayPublishCacheTest : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }
};

TEST_F(Px4GatewayPublishCacheTest, PublishesCachedMessagesToAllTopics) {
  // 经验总结：Gateway 场景的发布测试需要遵循“三步走”——(1) 先准备缓存数据，
  // (2) 让 Gateway 节点加入 executor 并等待发现订阅者，(3) 再触发发布并在
  // executor 内循环 spin 等待消息到达。任何一步省略都会导致测试间歇性失败。
  auto cache = std::make_shared<Px4MsgsCache>();

  rclcpp::Clock clock;
  auto base_time = clock.now();

  px4Status::VehicleStatus vehicle_status{};
  vehicle_status.valid = true;
  vehicle_status.latest_timestamp = base_time;
  vehicle_status.arming_state = 2;
  vehicle_status.nav_state = 4;
  vehicle_status.failsafe = false;
  vehicle_status.pre_flight_checks_pass = true;
  cache->updateVehicleStatus(vehicle_status);

  px4Status::BatteryStatus battery_status{};
  battery_status.valid = true;
  battery_status.timestamp = base_time + rclcpp::Duration(0, 5000);
  battery_status.voltage_v = 12.6f;
  battery_status.current_a = 3.4f;
  battery_status.remaining = 0.82f;
  battery_status.warning = 2;
  cache->updateBatteryStatus(battery_status);

  px4Position::BasicPosition<px4Position::FrameNED> position{};
  position.valid = true;
  position.translation = Eigen::Vector3d(1.0, -2.0, 3.5);
  position.orientation = Eigen::Quaterniond(0.9238795, 0.0, 0.3826834, 0.0);
  position.timestamp = base_time + rclcpp::Duration(0, 10000);
  cache->updatePositionNED(position);

  auto gateway = std::make_shared<PX4Gateway>(rclcpp::NodeOptions(), cache);
  auto listener = std::make_shared<CacheSubscriberNode>();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(gateway);
  exec.add_node(listener);

  // 经验总结：ROS 2 的内存内测试同样需要等待发现完成，否则 publishCache 调用
  // 可能在订阅者尚未连接时触发，导致消息“丢失”。使用细粒度轮询比直接 sleep
  // 更可靠，可避免 ARM 板慢速环境下的偶发超时。
  auto discovery_deadline = std::chrono::steady_clock::now() + 200ms;
  while (!listener->publishers_discovered() &&
         std::chrono::steady_clock::now() < discovery_deadline) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  gateway->publishCache();

  auto receive_deadline = std::chrono::steady_clock::now() + 500ms;
  while (!listener->all_received() &&
         std::chrono::steady_clock::now() < receive_deadline) {
    exec.spin_some();
    std::this_thread::sleep_for(5ms);
  }

  exec.remove_node(listener);
  exec.remove_node(gateway);

  auto received_vehicle = listener->last_vehicle_status();
  ASSERT_TRUE(received_vehicle.has_value());
  EXPECT_TRUE(received_vehicle->valid);
  EXPECT_EQ(received_vehicle->arming_state, vehicle_status.arming_state);
  EXPECT_EQ(received_vehicle->nav_state, vehicle_status.nav_state);
  EXPECT_EQ(received_vehicle->failsafe, vehicle_status.failsafe);
  EXPECT_EQ(received_vehicle->pre_flight_checks_pass,
            vehicle_status.pre_flight_checks_pass);
  EXPECT_EQ(rclcpp::Time(received_vehicle->latest_timestamp).nanoseconds(),
            vehicle_status.latest_timestamp.nanoseconds());

  auto received_position = listener->last_position();
  ASSERT_TRUE(received_position.has_value());
  EXPECT_TRUE(received_position->valid);
  EXPECT_DOUBLE_EQ(received_position->translation[0], position.translation.x());
  EXPECT_DOUBLE_EQ(received_position->translation[1], position.translation.y());
  EXPECT_DOUBLE_EQ(received_position->translation[2], position.translation.z());
  EXPECT_NEAR(received_position->orientation[0], position.orientation.w(),
              1e-6);
  EXPECT_NEAR(received_position->orientation[1], position.orientation.x(),
              1e-6);
  EXPECT_NEAR(received_position->orientation[2], position.orientation.y(),
              1e-6);
  EXPECT_NEAR(received_position->orientation[3], position.orientation.z(),
              1e-6);
  EXPECT_EQ(rclcpp::Time(received_position->timestamp).nanoseconds(),
            position.timestamp.nanoseconds());

  auto received_battery = listener->last_battery();
  ASSERT_TRUE(received_battery.has_value());
  EXPECT_TRUE(received_battery->valid);
  EXPECT_FLOAT_EQ(received_battery->voltage_v, battery_status.voltage_v);
  EXPECT_FLOAT_EQ(received_battery->current_a, battery_status.current_a);
  EXPECT_FLOAT_EQ(received_battery->remaining, battery_status.remaining);
  EXPECT_EQ(received_battery->warning, battery_status.warning);
  EXPECT_EQ(rclcpp::Time(received_battery->timestamp).nanoseconds(),
            battery_status.timestamp.nanoseconds());
}

} // namespace
