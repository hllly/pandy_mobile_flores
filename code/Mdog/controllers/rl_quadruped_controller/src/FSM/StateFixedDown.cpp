//
// 由 pj 于 24-9-11 创建。
//

#include "rl_quadruped_controller/FSM/StateFixedDown.h"

#include <cmath>
#include <iostream>

StateFixedDown::StateFixedDown(CtrlComponent &ctrlComp,
                               const std::vector<double> &target_pos,
                               const double kp,
                               const double kd)
    : FSMState(FSMStateName::FIXEDDOWN, "fixed down", ctrlComp),
      kp_(kp), kd_(kd) {
    duration_ = ctrl_comp_.frequency_ * 1.2;
    for (int i = 0; i < 16; i++) {
        target_pos_[i] = target_pos[i];
    }
}

void StateFixedDown::enter() {
    for (int i = 0; i < 16; i++) {
        start_pos_[i] = ctrl_comp_.joint_position_state_interface_[i].get().get_value();
        std::string joint_name = ctrl_comp_.joint_position_command_interface_[i].get().get_name();
        // std::cout<<joint_name<<start_pos_[i]<<std::endl;
        std::cout << "joint_position target[" << i << "]: " << target_pos_[i] << std::endl;
    }
    ctrl_comp_.control_inputs_.command = 0;
    for (int i = 0; i < 16; i++) {
        ctrl_comp_.joint_position_command_interface_[i].get().set_value(start_pos_[i]);
        ctrl_comp_.joint_velocity_command_interface_[i].get().set_value(0);
        ctrl_comp_.joint_torque_command_interface_[i].get().set_value(0);

        std::string joint_name = ctrl_comp_.joint_position_command_interface_[i].get().get_name();

        if(joint_name.find("ankle_joint") != std::string::npos){// 若为踝关节，则设置 kp=0、kd=1
            ctrl_comp_.joint_kp_command_interface_[i].get().set_value(0);
            ctrl_comp_.joint_kd_command_interface_[i].get().set_value(1);
        }else{
            ctrl_comp_.joint_kp_command_interface_[i].get().set_value(kp_);
            ctrl_comp_.joint_kd_command_interface_[i].get().set_value(kd_);   
        } 
    }
    
}

void StateFixedDown::run() {
    percent_ += 1 / duration_;
    phase = std::tanh(percent_);
    for (int i = 0; i < 16; i++) {
        ctrl_comp_.joint_position_command_interface_[i].get().set_value(
            phase * target_pos_[i] + (1 - phase) * start_pos_[i]);
    }
}

void StateFixedDown::exit() {
    percent_ = 0;
}

FSMStateName StateFixedDown::checkChange() {
    if (percent_ < 1.5) {
        return FSMStateName::FIXEDDOWN;
    }
    switch (ctrl_comp_.control_inputs_.command) {
        case 1:
            return FSMStateName::PASSIVE;
        case 2:
            return FSMStateName::FIXEDSTAND;
        case 3:
            return FSMStateName::RL;
        default:
            return FSMStateName::FIXEDDOWN;
    }
}
