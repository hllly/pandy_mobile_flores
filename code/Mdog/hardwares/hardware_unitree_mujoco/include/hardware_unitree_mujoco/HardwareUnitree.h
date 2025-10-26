//
// 由 pj 于 24-9-9 创建。
//


#ifndef HARDWAREUNITREE_H
#define HARDWAREUNITREE_H

#include "hardware_interface/system_interface.hpp"

#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <rclcpp/rclcpp.hpp>
#include "pd_interfaces/msg/low_cmd.hpp"
#include "pd_interfaces/msg/low_state.hpp"
#include "pd_interfaces/msg/motor_cmd.hpp"
#include "pd_interfaces/msg/motor_state.hpp"

class HardwareUnitree final : public hardware_interface::SystemInterface {
public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(const rclcpp::Time & /*时间*/, const rclcpp::Duration & /*周期*/) override;

protected:
    std::vector<double> joint_torque_command_;
    std::vector<double> joint_position_command_;
    std::vector<double> joint_velocities_command_;
    std::vector<double> joint_kp_command_;
    std::vector<double> joint_kd_command_;

    std::vector<double> joint_position_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_effort_;
    std::vector<double> joint_error_;

    std::vector<double> imu_states_;
    std::vector<double> foot_force_;

    std::unordered_map<std::string, std::vector<std::string> > joint_interfaces = {
        {"position", {}},
        {"velocity", {}},
        {"effort", {}},
        {"error", {}}
    };


    void initLowCmd();

    void lowStateMessageHandle(const void *messages);
    void SportModeStateMessageHandle(const void *messages);

    unitree_go::msg::dds_::LowCmd_ low_cmd_{}; // 默认初始化
    unitree_go::msg::dds_::LowState_ low_state_{}; // 默认初始化
    unitree_go::msg::dds_::SportModeState_ sport_mode_state_{}; // 默认初始化

    /* 发布器 */
    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> low_cmd_publisher_;
    /* 订阅器 */
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lows_tate_subscriber_;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> sport_mode_state_subscriber_;

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Publisher<pd_interfaces::msg::LowState>::SharedPtr low_state_debug_publisher_;
    rclcpp::Publisher<pd_interfaces::msg::LowCmd>::SharedPtr low_cmd_debug_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr high_state_debug_publisher_;
    pd_interfaces::msg::LowCmd Low_cmd_debug_;
    pd_interfaces::msg::LowState Low_state_debug_;
    nav_msgs::msg::Odometry high_state_debug_;

};


#endif //HARDWAREUNITREE_H
