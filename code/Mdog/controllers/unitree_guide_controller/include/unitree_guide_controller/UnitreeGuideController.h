//
// 由 pj 于 24-9-6 创建。
//

#ifndef QUADRUPEDCONTROLLER_H
#define QUADRUPEDCONTROLLER_H

#include <controller_interface/controller_interface.hpp>
#include <std_msgs/msg/string.hpp>
#include <unitree_guide_controller/FSM/FSMState.h>
#include <unitree_guide_controller/common/enumClass.h>

#include "FSM/StateBalanceTest.h"
#include "FSM/StateFixedDown.h"
#include "FSM/StateFixedStand.h"
#include "FSM/StateFreeStand.h"
#include "FSM/StatePassive.h"
#include "FSM/StateSwingTest.h"
#include "FSM/StateTrotting.h"

namespace unitree_guide_controller {
    struct FSMStateList {
        std::shared_ptr<FSMState> invalid;
        std::shared_ptr<StatePassive> passive;
        std::shared_ptr<StateFixedDown> fixedDown;
        std::shared_ptr<StateFixedStand> fixedStand;
        std::shared_ptr<StateFreeStand> freeStand;
        std::shared_ptr<StateTrotting> trotting;

        std::shared_ptr<StateSwingTest> swingTest;
        std::shared_ptr<StateBalanceTest> balanceTest;
    };

    class UnitreeGuideController final : public controller_interface::ControllerInterface {
    public:
        CONTROLLER_INTERFACE_PUBLIC
        UnitreeGuideController() = default;

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
        controller_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State &previous_state) override;

        CONTROLLER_INTERFACE_PUBLIC
        controller_interface::CallbackReturn on_shutdown(
            const rclcpp_lifecycle::State &previous_state) override;

        CtrlComponent ctrl_comp_;

    protected:
        std::vector<std::string> joint_names_;
        std::vector<std::string> command_interface_types_;
        std::vector<std::string> state_interface_types_;

        std::string imu_name_;
        std::string base_name_;
        std::string command_prefix_;
        std::vector<std::string> imu_interface_types_;
        std::vector<std::string> feet_names_;

        // 顺序：FR FL RR RL
        // 备选站立姿态（原始 Unitree 配置）：
        // 0.0, 0.67, -1.3,
        // 0.0, 0.67, -1.3,
        // 0.0, 0.67, -1.3,
        // 0.0, 0.67, -1.3

        // 顺序：FR FL RR RL
        std::vector<double> stand_pos_ = {
            0.0, -0.7, 0.5, 0.0,
            0.0, 0.7, -0.5, 0.0,
            0.0, -0.7, 0.5, 0.0,
            0.0, 0.7, -0.5, 0.0
        };

        // 备选下蹲姿态（原始 Unitree 配置）：
        // 0.0, 1.3, -2.4,
        // 0.0, 1.3, -2.4,
        // 0.0, 1.3, -2.4,
        // 0.0, 1.3, -2.4

        std::vector<double> down_pos_ = {
            0.0, -0.9, 1.6, 0.0,
            0.0, 0.9, -1.6, 0.0,
            0.0, -0.9, 1.6, 0.0,
            0.0, 0.9, -1.6, 0.0,
        };

        double stand_kp_ = 590.0;
        double stand_kd_ = 20.0;
        // 备选增益：stand_kp_ = 500.0;
        // 备选阻尼：stand_kd_ = 5.0;

        rclcpp::Subscription<control_input_msgs::msg::Inputs>::SharedPtr control_input_subscription_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscription_;

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface> > *>
        command_interface_map_ = {
            {"effort", &ctrl_comp_.joint_torque_command_interface_},
            {"position", &ctrl_comp_.joint_position_command_interface_},
            {"velocity", &ctrl_comp_.joint_velocity_command_interface_},
            {"kp", &ctrl_comp_.joint_kp_command_interface_},
            {"kd", &ctrl_comp_.joint_kd_command_interface_}
        };

        FSMMode mode_ = FSMMode::NORMAL;
        std::string state_name_;
        FSMStateName next_state_name_ = FSMStateName::INVALID;
        FSMStateList state_list_;
        std::shared_ptr<FSMState> current_state_;
        std::shared_ptr<FSMState> next_state_;

        std::chrono::time_point<std::chrono::steady_clock> last_update_time_;
        double update_frequency_;

        std::shared_ptr<FSMState> getNextState(FSMStateName stateName) const;

        std::unordered_map<
            std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface> > *>
        state_interface_map_ = {
            {"position", &ctrl_comp_.joint_position_state_interface_},
            {"effort", &ctrl_comp_.joint_effort_state_interface_},
            {"velocity", &ctrl_comp_.joint_velocity_state_interface_}
        };
    };
}


#endif //QUADRUPEDCONTROLLER_H
