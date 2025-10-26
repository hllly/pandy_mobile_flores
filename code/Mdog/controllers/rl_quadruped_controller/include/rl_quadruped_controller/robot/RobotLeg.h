//
// 由 pj 于 24-9-12 创建。
//


#ifndef ROBOTLEG_H
#define ROBOTLEG_H

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <rl_quadruped_controller/common/mathTypes.h>

class RobotLeg {
public:
    explicit RobotLeg(const KDL::Chain &chain);

    ~RobotLeg() = default;

    /**
     * 使用正运动学计算末端执行器相对机体坐标系的位姿。
     * @param joint_positions 腿部关节位置
     * @return 末端执行器相对机体坐标系的位姿
     */
    [[nodiscard]] KDL::Frame calcPEe2B(const KDL::JntArray &joint_positions) const;

    /**
     * 使用逆运动学求解目标关节位置。
     * @param pEe 末端执行器目标位姿
     * @param q_init 当前关节位置
     * @return 目标关节位置
     */
    [[nodiscard]] KDL::JntArray calcQ(const KDL::Frame &pEe, const KDL::JntArray &q_init) const;

    /**
     * 计算当前雅可比矩阵。
     * @param joint_positions 腿部关节位置
     * @return 雅可比矩阵
     */
    [[nodiscard]] KDL::Jacobian calcJaco(const KDL::JntArray &joint_positions) const;

    /**
     * 根据关节位置与末端受力计算关节力矩。
     * @param joint_positions 当前关节位置
     * @param force 足端受力
     * @return 关节力矩
     */
    [[nodiscard]] KDL::JntArray calcTorque(const KDL::JntArray &joint_positions, const Vec3 &force) const;

protected:
    KDL::Chain chain_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_pose_solver_;
    std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;
    std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_pose_solver_;
};


#endif //ROBOTLEG_H
