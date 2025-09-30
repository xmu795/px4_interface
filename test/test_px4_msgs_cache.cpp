#include "px4_interface/px4_msgs_cache.hpp"
#include <atomic>
#include <chrono>
#include <future>
#include <gtest/gtest.h>
#include <random>
#include <thread>
#include <vector>

class Px4MsgsCacheTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    cache_ = std::make_unique<Px4MsgsCache>();

    // 创建测试数据
    createTestPositionNED();
    createTestBatteryStatus();
    createTestVehicleStatus();
  }

  void TearDown() override {cache_.reset();}

  void createTestPositionNED()
  {
    Eigen::Vector3d translation(1.0, 2.0, 3.0);
    Eigen::Quaterniond orientation(1.0, 0.0, 0.0, 0.0);
    rclcpp::Time timestamp = rclcpp::Clock().now();
    test_position_ =
      px4Position::PositionNED(translation, orientation, timestamp);
  }

  void createTestBatteryStatus()
  {
    test_battery_.valid = true;
    test_battery_.timestamp = rclcpp::Clock().now();
    test_battery_.voltage_v = 12.6f;
    test_battery_.current_a = 2.5f;
    test_battery_.remaining = 0.85f;
    test_battery_.warning = 0;
  }

  void createTestVehicleStatus()
  {
    test_vehicle_.valid = true;
    test_vehicle_.latest_timestamp = rclcpp::Clock().now();
    test_vehicle_.arming_state = 2; // ARMED
    test_vehicle_.nav_state = 4;    // AUTO_MISSION
    test_vehicle_.failsafe = false;
    test_vehicle_.pre_flight_checks_pass = true;
  }

  std::unique_ptr<Px4MsgsCache> cache_;
  px4Position::PositionNED test_position_;
  px4Status::BatteryStatus test_battery_;
  px4Status::VehicleStatus test_vehicle_;
};

// 基本功能测试
TEST_F(Px4MsgsCacheTest, BasicUpdateAndGet) {
  // 测试位置数据
  cache_->updatePositionNED(test_position_);
  auto retrieved_position = cache_->getPositionNED();

  EXPECT_TRUE(retrieved_position.valid);
  EXPECT_DOUBLE_EQ(retrieved_position.translation.x(), 1.0);
  EXPECT_DOUBLE_EQ(retrieved_position.translation.y(), 2.0);
  EXPECT_DOUBLE_EQ(retrieved_position.translation.z(), 3.0);
  EXPECT_DOUBLE_EQ(retrieved_position.orientation.w(), 1.0);

  // 测试电池状态数据
  cache_->updateBatteryStatus(test_battery_);
  auto retrieved_battery = cache_->getBatteryStatus();

  EXPECT_TRUE(retrieved_battery.valid);
  EXPECT_FLOAT_EQ(retrieved_battery.voltage_v, 12.6f);
  EXPECT_FLOAT_EQ(retrieved_battery.current_a, 2.5f);
  EXPECT_FLOAT_EQ(retrieved_battery.remaining, 0.85f);
  EXPECT_EQ(retrieved_battery.warning, 0);

  // 测试飞行状态数据
  cache_->updateVehicleStatus(test_vehicle_);
  auto retrieved_vehicle = cache_->getVehicleStatus();

  EXPECT_TRUE(retrieved_vehicle.valid);
  EXPECT_EQ(retrieved_vehicle.arming_state, 2);
  EXPECT_EQ(retrieved_vehicle.nav_state, 4);
  EXPECT_FALSE(retrieved_vehicle.failsafe);
  EXPECT_TRUE(retrieved_vehicle.pre_flight_checks_pass);
}

// 测试初始状态
TEST_F(Px4MsgsCacheTest, InitialState) {
  auto position = cache_->getPositionNED();
  auto battery = cache_->getBatteryStatus();
  auto vehicle = cache_->getVehicleStatus();

  EXPECT_FALSE(position.valid);
  EXPECT_FALSE(battery.valid);
  EXPECT_FALSE(vehicle.valid);
}

// 多次更新测试
TEST_F(Px4MsgsCacheTest, MultipleUpdates) {
  // 更新位置数据多次
  for (int i = 0; i < 10; ++i) {
    Eigen::Vector3d translation(i, i + 1, i + 2);
    Eigen::Quaterniond orientation(1.0, 0.0, 0.0, 0.0);
    rclcpp::Time timestamp = rclcpp::Clock().now();
    px4Position::PositionNED position(translation, orientation, timestamp);

    cache_->updatePositionNED(position);

    auto retrieved = cache_->getPositionNED();
    EXPECT_DOUBLE_EQ(retrieved.translation.x(), i);
    EXPECT_DOUBLE_EQ(retrieved.translation.y(), i + 1);
    EXPECT_DOUBLE_EQ(retrieved.translation.z(), i + 2);
  }
}

// 并发读写测试 - 基本竞态场景
TEST_F(Px4MsgsCacheTest, ConcurrentReadWrite) {
  const int num_iterations = 1000;
  std::atomic<bool> test_failed{false};

  // 写线程
  auto writer = std::async(
    std::launch::async, [this, num_iterations,
    &test_failed]() {
      for (int i = 0; i < num_iterations; ++i) {
        try {
          Eigen::Vector3d translation(i, i + 1, i + 2);
          Eigen::Quaterniond orientation(1.0, 0.0, 0.0, 0.0);
          rclcpp::Time timestamp = rclcpp::Clock().now();
          px4Position::PositionNED position(translation, orientation, timestamp);

          cache_->updatePositionNED(position);

          // 短暂延迟以增加竞态条件的可能性
          std::this_thread::sleep_for(std::chrono::microseconds(1));
        } catch (...) {
          test_failed = true;
          break;
        }
      }
    });

  // 读线程
  auto reader =
    std::async(
    std::launch::async, [this, num_iterations, &test_failed]() {
      for (int i = 0; i < num_iterations; ++i) {
        try {
          auto position = cache_->getPositionNED();
          // 验证数据的一致性（如果有效的话）
          if (position.valid) {
            EXPECT_GE(position.translation.x(), 0);
            EXPECT_GE(position.translation.y(), 1);
            EXPECT_GE(position.translation.z(), 2);
          }
          std::this_thread::sleep_for(std::chrono::microseconds(1));
        } catch (...) {
          test_failed = true;
          break;
        }
      }
    });

  writer.wait();
  reader.wait();

  EXPECT_FALSE(test_failed);
}

// 多线程写入竞态测试
TEST_F(Px4MsgsCacheTest, MultipleWritersRaceCondition) {
  const int num_threads = 4;
  const int iterations_per_thread = 250;
  std::vector<std::future<void>> futures;
  std::atomic<bool> test_failed{false};

  for (int thread_id = 0; thread_id < num_threads; ++thread_id) {
    futures.push_back(
      std::async(
        std::launch::async, [this, thread_id, iterations_per_thread,
        &test_failed]() {
          std::random_device rd;
          std::mt19937 gen(rd());
          std::uniform_int_distribution<> dis(1, 10);

          for (int i = 0; i < iterations_per_thread; ++i) {
            try {
              // 每个线程使用不同的数据模式
              float base_value = thread_id * 1000 + i;

              // 更新位置数据
              Eigen::Vector3d translation(
                base_value, base_value + 1,
                base_value + 2);
              Eigen::Quaterniond orientation(1.0, 0.0, 0.0, 0.0);
              rclcpp::Time timestamp = rclcpp::Clock().now();
              px4Position::PositionNED position(
                translation, orientation,
                timestamp);
              cache_->updatePositionNED(position);

              // 更新电池状态
              px4Status::BatteryStatus battery;
              battery.valid = true;
              battery.timestamp = timestamp;
              battery.voltage_v = 12.0f + (base_value / 1000.0f);
              battery.current_a = 2.0f + (base_value / 10000.0f);
              battery.remaining = 0.5f + (thread_id * 0.1f);
              battery.warning = thread_id % 3;
              cache_->updateBatteryStatus(battery);

              // 更新飞行状态
              px4Status::VehicleStatus vehicle;
              vehicle.valid = true;
              vehicle.latest_timestamp = timestamp;
              vehicle.arming_state = (thread_id % 2) + 1;
              vehicle.nav_state = thread_id + 1;
              vehicle.failsafe = (thread_id % 2) == 0;
              vehicle.pre_flight_checks_pass = (thread_id % 2) == 1;
              cache_->updateVehicleStatus(vehicle);

              // 随机延迟以增加竞态条件
              std::this_thread::sleep_for(std::chrono::microseconds(dis(gen)));

            } catch (...) {
              test_failed = true;
              break;
            }
          }
        }));
  }

  // 等待所有线程完成
  for (auto & future : futures) {
    future.wait();
  }

  EXPECT_FALSE(test_failed);

  // 验证最终状态的数据完整性
  auto position = cache_->getPositionNED();
  auto battery = cache_->getBatteryStatus();
  auto vehicle = cache_->getVehicleStatus();

  EXPECT_TRUE(position.valid);
  EXPECT_TRUE(battery.valid);
  EXPECT_TRUE(vehicle.valid);
}

// 读取器饥饿测试 - 确保在频繁写入时读取器仍能获得数据
TEST_F(Px4MsgsCacheTest, ReaderStarvationTest) {
  const int num_writers = 3;
  const int num_readers = 2;
  const int test_duration_ms = 500;

  std::atomic<bool> stop_test{false};
  std::atomic<int> successful_reads{0};
  std::atomic<int> successful_writes{0};
  std::vector<std::future<void>> futures;

  // 启动写线程
  for (int i = 0; i < num_writers; ++i) {
    futures.push_back(
      std::async(
        std::launch::async, [this, i, &stop_test, &successful_writes]() {
          int counter = 0;
          while (!stop_test) {
            try {
              Eigen::Vector3d translation(counter, counter + 1, counter + 2);
              Eigen::Quaterniond orientation(1.0, 0.0, 0.0, 0.0);
              rclcpp::Time timestamp = rclcpp::Clock().now();
              px4Position::PositionNED position(
                translation, orientation,
                timestamp);

              cache_->updatePositionNED(position);
              successful_writes++;
              counter++;

              // 快速写入
              std::this_thread::sleep_for(std::chrono::microseconds(10));
            } catch (...) {
              break;
            }
          }
        }));
  }

  // 启动读线程
  for (int i = 0; i < num_readers; ++i) {
    futures.push_back(
      std::async(
        std::launch::async, [this, &stop_test, &successful_reads]() {
          while (!stop_test) {
            try {
              auto position = cache_->getPositionNED();
              auto battery = cache_->getBatteryStatus();
              auto vehicle = cache_->getVehicleStatus();

              successful_reads++;
              std::this_thread::sleep_for(std::chrono::microseconds(50));
            } catch (...) {
              break;
            }
          }
        }));
  }

  // 运行测试指定时间
  std::this_thread::sleep_for(std::chrono::milliseconds(test_duration_ms));
  stop_test = true;

  // 等待所有线程完成
  for (auto & future : futures) {
    future.wait();
  }

  // 验证读取器没有被饥饿
  EXPECT_GT(successful_reads.load(), 0);
  EXPECT_GT(successful_writes.load(), 0);

  // 读取次数应该合理（不会为0）
  std::cout << "Successful reads: " << successful_reads.load() << std::endl;
  std::cout << "Successful writes: " << successful_writes.load() << std::endl;
}

// 高并发压力测试
TEST_F(Px4MsgsCacheTest, HighConcurrencyStressTest) {
  const int num_threads = 8;
  const int iterations_per_thread = 500;
  std::vector<std::future<void>> futures;
  std::atomic<int> total_operations{0};
  std::atomic<bool> test_failed{false};

  for (int thread_id = 0; thread_id < num_threads; ++thread_id) {
    futures.push_back(
      std::async(
        std::launch::async, [this, thread_id, iterations_per_thread,
        &total_operations, &test_failed]() {
          std::random_device rd;
          std::mt19937 gen(rd());
          std::uniform_int_distribution<> operation_dis(0, 5);

          for (int i = 0; i < iterations_per_thread; ++i) {
            try {
              int operation = operation_dis(gen);

              switch (operation) {
                case 0:
                case 1: {
                  // 更新位置
                  Eigen::Vector3d translation(thread_id, i, thread_id + i);
                  Eigen::Quaterniond orientation(1.0, 0.0, 0.0, 0.0);
                  rclcpp::Time timestamp = rclcpp::Clock().now();
                  px4Position::PositionNED position(
                    translation, orientation,
                    timestamp);
                  cache_->updatePositionNED(position);
                  break;
                }
                case 2: {
                  // 更新电池状态
                  px4Status::BatteryStatus battery;
                  battery.valid = true;
                  battery.timestamp = rclcpp::Clock().now();
                  battery.voltage_v = 12.0f + thread_id;
                  battery.current_a = 2.0f + i * 0.1f;
                  battery.remaining = 0.8f;
                  battery.warning = 0;
                  cache_->updateBatteryStatus(battery);
                  break;
                }
                case 3: {
                  // 更新飞行状态
                  px4Status::VehicleStatus vehicle;
                  vehicle.valid = true;
                  vehicle.latest_timestamp = rclcpp::Clock().now();
                  vehicle.arming_state = 2;
                  vehicle.nav_state = 4;
                  vehicle.failsafe = false;
                  vehicle.pre_flight_checks_pass = true;
                  cache_->updateVehicleStatus(vehicle);
                  break;
                }
                case 4:
                case 5: {
                  // 读取所有数据
                  auto position = cache_->getPositionNED();
                  auto battery = cache_->getBatteryStatus();
                  auto vehicle = cache_->getVehicleStatus();

                  // 简单验证数据完整性
                  if (position.valid) {
                    EXPECT_GE(position.translation.x(), 0);
                  }
                  if (battery.valid) {
                    EXPECT_GT(battery.voltage_v, 0);
                  }
                  if (vehicle.valid) {
                    EXPECT_GE(vehicle.arming_state, 0);
                  }
                  break;
                }
              }

              total_operations++;

            } catch (...) {
              test_failed = true;
              break;
            }
          }
        }));
  }

  // 等待所有线程完成
  for (auto & future : futures) {
    future.wait();
  }

  EXPECT_FALSE(test_failed);
  EXPECT_EQ(total_operations.load(), num_threads * iterations_per_thread);

  std::cout << "Total operations completed: " << total_operations.load()
            << std::endl;
}

// 数据一致性验证测试
TEST_F(Px4MsgsCacheTest, DataConsistencyTest) {
  const int num_iterations = 1000;
  std::atomic<bool> inconsistency_detected{false};

  // 写线程 - 写入关联的数据
  auto writer = std::async(
    std::launch::async, [this, num_iterations]() {
      for (int i = 0; i < num_iterations; ++i) {
        // 使用相同的序列号来关联数据
        float sequence = static_cast<float>(i);
        rclcpp::Time timestamp = rclcpp::Clock().now();

        // 位置数据使用序列号
        Eigen::Vector3d translation(sequence, sequence, sequence);
        Eigen::Quaterniond orientation(1.0, 0.0, 0.0, 0.0);
        px4Position::PositionNED position(translation, orientation, timestamp);
        cache_->updatePositionNED(position);

        // 电池电压也使用序列号
        px4Status::BatteryStatus battery;
        battery.valid = true;
        battery.timestamp = timestamp;
        battery.voltage_v = 12.0f + sequence * 0.001f;
        battery.current_a = 2.0f;
        battery.remaining = 0.8f;
        battery.warning = 0;
        cache_->updateBatteryStatus(battery);

        std::this_thread::sleep_for(std::chrono::microseconds(10));
      }
    });

  // 读线程 - 验证数据的一致性
  auto reader = std::async(
    std::launch::async, [this, num_iterations, &inconsistency_detected]() {
      for (int i = 0; i < num_iterations * 2; ++i) {
        auto position = cache_->getPositionNED();
        auto battery = cache_->getBatteryStatus();

        // 如果数据有效，检查是否一致
        if (position.valid && battery.valid) {
          float pos_sequence = position.translation.x();
          float battery_sequence = (battery.voltage_v - 12.0f) / 0.001f;

          // 允许一定的浮点误差和并发更新差异
          if (std::abs(pos_sequence - battery_sequence) > 5.0f) {
            std::cout << "Inconsistency detected: pos=" << pos_sequence
                      << ", battery=" << battery_sequence << std::endl;
            inconsistency_detected = true;
          }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(5));
      }
    });

  writer.wait();
  reader.wait();

  // 在正常情况下，数据应该基本一致（允许少量并发导致的差异）
  EXPECT_FALSE(inconsistency_detected);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
