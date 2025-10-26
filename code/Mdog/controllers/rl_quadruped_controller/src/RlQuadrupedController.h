//
// 由 pj 于 24-10-4 创建。
//

#ifndef LEGGEDGYMCONTROLLER_H
#define LEGGEDGYMCONTROLLER_H
#include <controller_interface/controller_interface.hpp>
#include <rl_quadruped_controller/FSM/StateRL.h>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "rl_quadruped_controller/control/CtrlComponent.h"
#include "rl_quadruped_controller/FSM/StateFixedDown.h"
#include "rl_quadruped_controller/FSM/StateFixedStand.h"
#include "rl_quadruped_controller/FSM/StatePassive.h"

namespace rl_quadruped_controller {
    struct FSMStateList {
        std::shared_ptr<FSMState> invalid;
        std::shared_ptr<StatePassive> passive;
        std::shared_ptr<StateFixedDown> fixedDown;
        std::shared_ptr<StateFixedStand> fixedStand;
        std::shared_ptr<StateRL> rl;
    };

    class LeggedGymController final : public controller_interface::ControllerInterface {
    public:
        CONTROLLER_INTERFACE_PUBLIC
        LeggedGymController() = default;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_init() override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State &previous_state) override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state) override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State &previous_state) override;

    private:
        std::shared_ptr<FSMState> getNextState(FSMStateName stateName) const;

        CtrlComponent ctrl_comp_;
        std::vector<std::string> joint_names_;
        std::string base_name_ = "base";
        std::vector<std::string> feet_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        std::string command_prefix_;

        // IMU 传感器
        std::string imu_name_;
        std::vector<std::string> imu_interface_types_;
        // 足端力传感器
        std::string foot_force_name_;
        std::vector<std::string> foot_force_interface_types_;

        // 顺序：FR FL RR RL（右前、左前、右后、左后）
        // std::vector<double> stand_pos_ = {
        //     0.0, 0.67, -1.3,
        //     0.0, 0.67, -1.3,
        //     0.0, 0.67, -1.3,
        //     0.0, 0.67, -1.3
        // };

        // 顺序：FL FR RL RR（左前、右前、右后、左后）
        std::vector<double> stand_pos_ = {
            0.0, 0.8, 0.5, 1,
            0.0, -0.8, -0.5, -1,
            0.0, 0.8, 0.5, 1,
            0.0, -0.8, -0.5, -1
        };

        // std::vector<double> down_pos_ = {
        //     0.0, 1.3, -2.4,
        //     0.0, 1.3, -2.4,
        //     0.0, 1.3, -2.4,
        //     0.0, 1.3, -2.4
        // };

        std::vector<double> down_pos_ = {
            0.0, 1.3, -1.1, 0.0,
            0.0, -1.3, 1.1, 0.0,
            0.0, 1.3, -1.1, 0.0,
            0.0, -1.3, 1.1, 0.0,
        };


        double stand_kp_ = 500.0;
        double stand_kd_ = 5.0;

        // double stand_kp_ = 250.0;
        // double stand_kd_ = 5.0;

        std::string robot_pkg_;
        std::string model_folder_;

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface> > *>
        command_interface_map_ = {
            {"effort", &ctrl_comp_.joint_torque_command_interface_},
            {"position", &ctrl_comp_.joint_position_command_interface_},
            {"velocity", &ctrl_comp_.joint_velocity_command_interface_},
            {"kp", &ctrl_comp_.joint_kp_command_interface_},
            {"kd", &ctrl_comp_.joint_kd_command_interface_}
        };

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> > *>
        state_interface_map_ = {
            {"position", &ctrl_comp_.joint_position_state_interface_},
            {"effort", &ctrl_comp_.joint_effort_state_interface_},
            {"velocity", &ctrl_comp_.joint_velocity_state_interface_},
            {"error", &ctrl_comp_.joint_error_state_interface_}
        };

        rclcpp::Subscription<control_input_msgs::msg::Inputs>::SharedPtr control_input_subscription_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr high_input_subscription_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;

        FSMMode mode_ = FSMMode::NORMAL;
        std::string state_name_;
        FSMStateName next_state_name_ = FSMStateName::INVALID;
        FSMStateList state_list_;
        std::shared_ptr<FSMState> current_state_;
        std::shared_ptr<FSMState> next_state_;
        bool use_high_cmd_ = false;
    };
}
#endif //LEGGEDGYMCONTROLLER_H
