#pragma once
#include "px4_comm_types.hpp"
#include "px4_msgs_cache.hpp"
#include <memory>
#include <px4_msgs/msg/battery_status.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

class PX4Gateway : public rclcpp::Node {
public:
  /**
   * @brief 构造函数
   * @param options ROS2节点选项
   * @param cache 共享的PX4消息缓存对象
   * 依靠注入的Px4MsgsCache对象进行数据存取，不允许为空
   * 通过ROS2订阅器接收PX4消息并更新缓存
   */
  explicit PX4Gateway(const rclcpp::NodeOptions &options,
                      std::shared_ptr<Px4MsgsCache> cache);
  /**
   * @brief 析构函数
   * 基类rclcpp::Node有虚析构函数
   */
  ~PX4Gateway() override = default;
  //============发布Cache数据============
  /**
   * @brief 读取并发布缓存中的所有数据（车辆状态、位置、电池状态）
   *
   */
  void publishCache() const;

  //===============提供的API================
  /**
   * @brief 检测与PX4的连接状态，必须在init的时候调用
   * @return true 连接正常
   * @return false 连接异常
   * @warning
   * 该函数检测的是px4是否已经实现了/fmu/out/*话题的发布者，不能够检测更深层次的连接问题
   */
  bool checkPx4Publishers() const;
  //=====运动控制命令=====
  /**
   * @brief 选择使用速度控制，位置控制还是速度前馈的位置控制
   *  @param position_control true表示使用位置控制
   *  @param velocity_control true表示使用速度控制
   *  @note
   * 该函数设置OffboardControlMode消息的position和velocity字段
   * 位置控制和速度控制可以同时开启
   */
  void chooseControlMethod(bool position_control, bool velocity_control);
  /**
   * @brief 发送运动控制命令
   * @param target 目标位置和速度
   * @note
   * 该函数使用px4GatewayTypes::setpoint结构体作为输入，只需要包含位置、速度和偏航角，函数内部会添加时间戳和填充不用的字段，简化调用
   */
  void setTarget(const px4GatewayTypes::setpoint &target);
  void setTarget(const px4_msgs::msg::TrajectorySetpoint &target);
  //=====模式控制命令=====
  /**
   * @brief 发布车辆控制命令，高层接口调用此函数来发送具体命令
   * px4_comm_types 中的px4Enum::VehicleCommand枚举定义了具体的车辆命令
   * @param command 车辆命令枚举
   * @param param1 命令参数1
   * @param param2 命令参数2
   */
  void publishVehicleCommand(const px4Enum::VehicleCommand command,
                             const float param1 = 0.0,
                             const float param2 = 0.0);
  //=====常用控制命令=====
  void setArmMode();
  void setDisarmMode();
  void setOffboardMode();
  void setLandMode();
  void triggerEmergencyStop();

private:
  //===============内部函数===============
  /**
   * @brief 初始化函数，在构造函数中调用，构造所有的订阅器和发布器
   */
  void init();

  /**
   * @brief 定期心跳，发布PX4在offboard需要的心跳
   */
  void publishOffboardControlMode(const bool position_control = true,
                                  const bool velocity_control = true);

  //===============成员变量===============

  // 订阅PX4的订阅者
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr
      vehicle_status_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
      vehicle_local_position_subscriber_;
  rclcpp::Subscription<px4_msgs::msg::BatteryStatus>::SharedPtr
      battery_status_subscriber_;

  // 发布给PX4的发布者
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr
      offboard_control_mode_publisher_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
      trajectory_setpoint_publisher_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr
      vehicle_command_publisher_;

  // 发布给外部的发布者
  rclcpp::Publisher<px4Status::VehicleStatus>::SharedPtr
      vehicle_status_publisher_;
  rclcpp::Publisher<px4Position::PositionNED>::SharedPtr
      vehicle_local_position_publisher_;
  rclcpp::Publisher<px4Status::BatteryStatus>::SharedPtr
      battery_status_publisher_;

  // PX4消息缓存
  std::shared_ptr<Px4MsgsCache> px4_msgs_cache_;

  const int offboard_control_rate_hz_ = 50; // 心跳频率50Hz
  bool offboard_position_control_ = true;
  bool offboard_velocity_control_ = true;
  rclcpp::TimerBase::SharedPtr offboard_control_timer_;
  rclcpp::TimerBase::SharedPtr cache_publish_timer_;
};
