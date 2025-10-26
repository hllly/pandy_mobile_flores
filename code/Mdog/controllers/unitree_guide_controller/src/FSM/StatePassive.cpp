//
// 由 pj 于 24-9-6 创建。
//

#include "unitree_guide_controller/FSM/StatePassive.h"

#include <iostream>
#include <utility>

StatePassive::StatePassive(CtrlComponent &ctrlComp) : FSMState(
    FSMStateName::PASSIVE, "passive", ctrlComp) {
}

void StatePassive::enter() {
    std::cout << "Enter Passive State" << std::endl;
    for (auto i: ctrl_comp_.joint_torque_command_interface_) {
        i.get().set_value(0);
    }
    for (auto i: ctrl_comp_.joint_position_command_interface_) {
        i.get().set_value(0);
    }
    for (auto i: ctrl_comp_.joint_velocity_command_interface_) {
        i.get().set_value(0);
    }
    for (auto i: ctrl_comp_.joint_kp_command_interface_) {
        i.get().set_value(0);
    }
    for (auto i: ctrl_comp_.joint_kd_command_interface_) {
        i.get().set_value(20);
    }
    ctrl_comp_.control_inputs_.command = 0;
}

void StatePassive::run() {
}

void StatePassive::exit() {
}

FSMStateName StatePassive::checkChange() {
    if (ctrl_comp_.control_inputs_.command == 2) {
        return FSMStateName::FIXEDDOWN;
    }
    return FSMStateName::PASSIVE;
}
