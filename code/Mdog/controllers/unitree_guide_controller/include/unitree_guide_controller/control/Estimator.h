//
// 由 pj 于 24-9-14 创建。
//

#ifndef ESTIMATOR_H
#define ESTIMATOR_H
#include <memory>
#include <kdl/frames.hpp>
#include <unitree_guide_controller/common/mathTypes.h>
#include <unitree_guide_controller/robot/QuadrupedRobot.h>

#include "LowPassFilter.h"

class WaveGenerator;
class QuadrupedRobot;
struct CtrlComponent;

class Estimator {
public:
    explicit Estimator(CtrlComponent &ctrl_component);

    ~Estimator() = default;

    /**
     * 获取估计的机体质心位置。
     */
    Vec3 getPosition() {
        return x_hat_.segment(0, 3);
    }

    /**
     * 获取估计的机体质心速度。
     */
    Vec3 getVelocity() {
        return x_hat_.segment(3, 3);
    }

    /**
     * 获取世界坐标系下的估计足端位置。
     * @param index 腿索引
     */
    Vec3 getFootPos(const int index) {
        return getPosition() + rotation_ * Vec3(foot_poses_[index].p.data);
    }

    /**
     * 获取世界坐标系下的估计足端位置集合。
     */
    Vec34 getFeetPos() {
        Vec34 feet_pos;
        for (int i(0); i < 4; ++i) {
            feet_pos.col(i) = getFootPos(i);
        }
        return feet_pos;
    }

    /**
     * 获取世界坐标系下的估计足端速度。
     */
    Vec34 getFeetVel() {
        const std::vector<KDL::Vector> feet_vel = robot_model_->getFeet2BVelocities();
        Vec34 result;
        for (int i(0); i < 4; ++i) {
            result.col(i) = Vec3(feet_vel[i].data) + getVelocity();
        }
        return result;
    }

    /**
     * 获取机体坐标系下的估计足端位置。
     */
    Vec34 getFeetPos2Body() {
        Vec34 foot_pos;
        const Vec3 body_pos = getPosition();
        for (int i = 0; i < 4; i++) {
            foot_pos.col(i) = getFootPos(i) - body_pos;
        }
        return foot_pos;
    }

    RotMat getRotation() {
        return rotation_;
    }

    Vec3 getGyro() {
        return gyro_;
    }

    [[nodiscard]] Vec3 getGyroGlobal() const {
        return rotation_ * gyro_;
    }

    [[nodiscard]] double getYaw() const;

    [[nodiscard]] double getDYaw() const {
        return getGyroGlobal()(2);
    }

    void update();

private:
    CtrlComponent &ctrl_component_;
    std::shared_ptr<QuadrupedRobot> &robot_model_;
    std::shared_ptr<WaveGenerator> &wave_generator_;

    Eigen::Matrix<double, 18, 1> x_hat_; // 估计器状态向量：位置(3)+速度(3)+足端位置(3x4)

    Eigen::Matrix<double, 3, 1> u_; // 估计器输入量

    Eigen::Matrix<double, 28, 1> y_; // 输出量的量测值
    Eigen::Matrix<double, 28, 1> y_hat_; // 输出量的预测值
    Eigen::Matrix<double, 18, 18> A; // 状态转移矩阵
    Eigen::Matrix<double, 18, 3> B; // 输入矩阵
    Eigen::Matrix<double, 28, 18> C; // 输出矩阵

    // 协方差矩阵
    Eigen::Matrix<double, 18, 18> P; // 预测协方差
    Eigen::Matrix<double, 18, 18> Ppriori; // 先验预测协方差
    Eigen::Matrix<double, 18, 18> Q; // 系统噪声协方差
    Eigen::Matrix<double, 28, 28> R; // 量测噪声协方差
    Eigen::Matrix<double, 18, 18> QInit_; // 系统噪声协方差初值
    Eigen::Matrix<double, 28, 28> RInit_; // 量测噪声协方差初值
    Eigen::Matrix<double, 18, 1> Qdig; // 可调过程噪声协方差
    Eigen::Matrix<double, 3, 3> Cu; // 输入量 u 的协方差

    // 输出量测
    Eigen::Matrix<double, 12, 1> feet_pos_body_; // 全局坐标下相对机体的足端位置
    Eigen::Matrix<double, 12, 1> feet_vel_body_; // 全局坐标下相对机体的足端速度
    Eigen::Matrix<double, 4, 1> feet_h_; // 全局坐标下各足端高度

    Eigen::Matrix<double, 28, 28> S; // _S = C*P*C.T + R
    Eigen::PartialPivLU<Eigen::Matrix<double, 28, 28> > Slu; // _S 的 LU 分解
    Eigen::Matrix<double, 28, 1> Sy; // _Sy = _S.inv() * (y - yhat)
    Eigen::Matrix<double, 28, 18> Sc; // _Sc = _S.inv() * C
    Eigen::Matrix<double, 28, 28> SR; // _SR = _S.inv() * R
    Eigen::Matrix<double, 28, 18> STC; // _STC = (_S.transpose()).inv() * C
    Eigen::Matrix<double, 18, 18> IKC; // _IKC = I - KC

    Vec3 g_;
    double dt_;

    RotMat rotation_;
    Vec3 acceleration_;
    Vec3 gyro_;

    std::vector<KDL::Frame> foot_poses_;
    std::vector<KDL::Vector> foot_vels_;
    std::vector<std::shared_ptr<LowPassFilter> > low_pass_filters_;

    double large_variance_;
};


#endif //ESTIMATOR_H
