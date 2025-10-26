//
// 由 pj 于 24-9-10 创建。
//

#include "unitree_guide_controller/FSM/StateFixedStand.h"

#include <cmath>

StateFixedStand::StateFixedStand(CtrlComponent &ctrlComp, const std::vector<double> &target_pos,
                                 const double kp,
                                 const double kd)
    : FSMState(FSMStateName::FIXEDSTAND, "fixed stand", ctrlComp),
      kp_(kp), kd_(kd) {
    duration_ = ctrl_comp_.frequency_ * 1.2;
    for (int i = 0; i < 16; i++) {
        target_pos_[i] = target_pos[i];
    }
}

void StateFixedStand::enter() {
    for (int i = 0; i < 16; i++) {
        start_pos_[i] = ctrl_comp_.joint_position_state_interface_[i].get().get_value();
    }
    for (int i = 0; i < 16; i++) {
        std::string joint_name = ctrl_comp_.joint_position_command_interface_[i].get().get_name();

        ctrl_comp_.joint_position_command_interface_[i].get().set_value(start_pos_[i]);
        ctrl_comp_.joint_velocity_command_interface_[i].get().set_value(0);
        ctrl_comp_.joint_torque_command_interface_[i].get().set_value(0);

        if(joint_name.find("ankle_joint") != std::string::npos){//if ankle joint, set kp to 0,kd to 1
            ctrl_comp_.joint_kp_command_interface_[i].get().set_value(0);
            ctrl_comp_.joint_kd_command_interface_[i].get().set_value(1);
        }else{
            ctrl_comp_.joint_kp_command_interface_[i].get().set_value(kp_);
            ctrl_comp_.joint_kd_command_interface_[i].get().set_value(kd_);   
        } 
    }
    ctrl_comp_.control_inputs_.command = 0;
}

void StateFixedStand::run() {
    percent_ += 1 / duration_;
    phase = std::tanh(percent_);
    for (int i = 0; i < 16; i++) {
        ctrl_comp_.joint_position_command_interface_[i].get().set_value(
            phase * target_pos_[i] + (1 - phase) * start_pos_[i]);
    }
}

void StateFixedStand::exit() {
    percent_ = 0;
}

FSMStateName StateFixedStand::checkChange() {
    if (percent_ < 1.5) {
        return FSMStateName::FIXEDSTAND;
    }
    switch (ctrl_comp_.control_inputs_.command) {
        case 1:
            return FSMStateName::PASSIVE;
        case 2:
            return FSMStateName::FIXEDDOWN;
        case 3:
            return FSMStateName::FREESTAND;
        case 4:
            return FSMStateName::TROTTING;
        case 5:
            return FSMStateName::SWINGTEST;
        case 6:
            return FSMStateName::BALANCETEST;
        default:
            return FSMStateName::FIXEDSTAND;
    }
}
