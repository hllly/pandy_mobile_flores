// 版权所有 (c) 2022，Stogl Robotics Consulting UG (haftungsbeschränkt)（模板）
//
// 根据 Apache License 2.0 版（下称“许可证”）授权；
// 未遵循许可证条款前不得使用本文件。
// 可在以下地址获取许可证副本：
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// 除非适用法律要求或书面同意，否则依据许可证分发的软件
// 均以“按原样”方式提供，不附带任何明示或暗示的担保或条件。
// 更多权限与限制条款请参阅许可证文本。

#ifndef MDOG_HARDWARE_INTERFACE__MDOG_HARDWARE_INTERFACE_HPP_
#define MDOG_HARDWARE_INTERFACE__MDOG_HARDWARE_INTERFACE_HPP_

#include <string>
#include <vector>
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include "mdog_hardware_interface/visibility_control.h"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/string.hpp"
#include "pd_interfaces/msg/low_cmd.hpp"
#include "pd_interfaces/msg/low_state.hpp"
#include "pd_interfaces/msg/motor_cmd.hpp"
#include "pd_interfaces/msg/motor_state.hpp"

namespace mdog_hardware_interface
{
class MdogHardwareInterface : public hardware_interface::SystemInterface
{
public:
  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  rclcpp::Logger get_logger() const { return *logger_; }
  
private:
  std::shared_ptr<rclcpp::Logger> logger_;

  std::vector<double> joint_torque_command_;
  std::vector<double> joint_position_command_;
  std::vector<double> joint_velocities_command_;
  std::vector<double> joint_kp_command_;
  std::vector<double> joint_kd_command_;

  std::vector<double> joint_position_;
  std::vector<double> joint_velocities_;
  std::vector<double> joint_effort_;
  std::vector<double> joint_error_;

  std::array<double,16> joint_position_temp_;
  std::array<double,16> joint_velocities_temp_;
  std::array<double,16> joint_effort_temp_;
  std::array<double,16> joint_kp_temp_;
  std::array<double,16> joint_kd_temp_;
  std::array<double,16> joint_error_temp_;
  
  std::array<double,4> wheel_init_pos_;

  std::vector<double> imu_states_;
  std::vector<double> foot_force_;

  std::array<double,10> imu_states_buffer_;
  std::array<double,16> direction;
  pd_interfaces::msg::LowCmd motorCMD_msg;

  std::unordered_map<std::string, std::vector<std::string> > joint_interfaces = {
      {"position", {}},
      {"velocity", {}},
      {"effort", {}},
      {"error",{}}
  };

  int init_flag=0;

  std::vector<double> joints_states;
  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  rclcpp::Subscription<pd_interfaces::msg::LowState>::SharedPtr state_subscriber_ = nullptr;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_ = nullptr;
  rclcpp::Publisher<pd_interfaces::msg::LowCmd>::SharedPtr cmd_publisher_ = nullptr;
  // void state_callback(const std_msgs::msg::String::SharedPtr msg);
  void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void low_state_callback(const pd_interfaces::msg::LowState::SharedPtr msg);
};

}  // 命名空间 mdog_hardware_interface

namespace hardware_interface{
  constexpr char HW_IF_KP[] = "kp";
  constexpr char HW_IF_KD[] = "kd";
}

#endif  // MDOG_HARDWARE_INTERFACE__MDOG_HARDWARE_INTERFACE_HPP_
