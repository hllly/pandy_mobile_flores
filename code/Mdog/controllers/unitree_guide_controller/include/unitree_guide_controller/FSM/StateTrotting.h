//
// 由 pj 于 24-9-18 创建。
//

#ifndef STATETROTTING_H
#define STATETROTTING_H
#include <unitree_guide_controller/gait/GaitGenerator.h>

#include "FSMState.h"


class StateTrotting final : public FSMState {
public:
    explicit StateTrotting(CtrlComponent &ctrlComp);

    void enter() override;

    void run() override;

    void exit() override;

    FSMStateName checkChange() override;

private:
    void getUserCmd();

    void calcCmd();

    /**
    * 计算输出的力矩指令。
    */
    void calcTau();

    /**
    * 计算关节空间的速度与加速度。
    */
    void calcQQd();

    /**
    * 计算各关节的 PD 增益。
    */
    void calcGain() const;

    /**
     * 判断当前是否需要迈步。
     */
    bool checkStepOrNot();

    std::shared_ptr<Estimator> &estimator_;
    std::shared_ptr<QuadrupedRobot> &robot_model_;
    std::shared_ptr<BalanceCtrl> &balance_ctrl_;
    std::shared_ptr<WaveGenerator> &wave_generator_;

    GaitGenerator gait_generator_;

    // 机器人状态量
    Vec3 pos_body_, vel_body_;
    RotMat B2G_RotMat, G2B_RotMat;

    // 机器人控制指令
    Vec3 pcd_;
    Vec3 vel_target_, v_cmd_body_;
    double dt_;
    double yaw_cmd_{}, d_yaw_cmd_{}, d_yaw_cmd_past_{};
    Vec3 w_cmd_global_;
    Vec34 pos_feet_global_goal_, vel_feet_global_goal_;
    RotMat Rd;

    // 控制参数
    double gait_height_;
    Vec3 pos_error_, vel_error_;
    Mat3 Kpp, Kdp, Kd_w_;
    double kp_w_;
    Mat3 Kp_swing_, Kd_swing_;
    Vec2 v_x_limit_, v_y_limit_, w_yaw_limit_;
};


#endif //STATETROTTING_H
