//
// 由 pj 于 24-9-12 创建。
//


#ifndef QUADRUPEDROBOT_H
#define QUADRUPEDROBOT_H
#include <string>
#include <kdl_parser/kdl_parser.hpp>
#include <unitree_guide_controller/common/mathTypes.h>

#include "RobotLeg.h"


struct CtrlComponent;

class QuadrupedRobot {
public:
    explicit QuadrupedRobot(CtrlComponent &ctrl_component, const std::string &robot_description,
                            const std::vector<std::string> &feet_names, const std::string &base_name);

    ~QuadrupedRobot() = default;

    /**
     * 根据足端目标位姿计算所需关节位置。
     * @param pEe_list 足端目标位姿列表
     */
    [[nodiscard]] std::vector<KDL::JntArray> getQ(const std::vector<KDL::Frame> &pEe_list) const;

    [[nodiscard]] Vec12 getQ(const Vec34 &vecP) const;

    Vec12 getQd(const std::vector<KDL::Frame> &pos, const Vec34 &vel);

    /**
     * 根据关节位置计算所有足端在机体坐标系下的位姿。
     */
    [[nodiscard]] std::vector<KDL::Frame> getFeet2BPositions() const;

    /**
     * 根据关节位置计算单条腿的足端位姿。
     * @param index 腿索引
     */
    [[nodiscard]] KDL::Frame getFeet2BPositions(int index) const;

    /**
     * 根据关节位置计算雅可比矩阵。
     * @param index 腿索引
     */
    [[nodiscard]] KDL::Jacobian getJacobian(int index) const;

    /**
     * 依据关节位置、关节速度与外力计算关节力矩。
     * @param force 外部作用力
     * @param index 腿索引
     */
    [[nodiscard]] KDL::JntArray getTorque(
        const Vec3 &force, int index) const;

    /**
    * 依据关节位置、关节速度与外力计算关节力矩（KDL 向量版本）。
    * @param force 外部作用力
    * @param index 腿索引
    */
    [[nodiscard]] KDL::JntArray getTorque(
        const KDL::Vector &force, int index) const;

    /**
     * 计算单条腿足端速度。
     * @param index 腿索引
     */
    [[nodiscard]] KDL::Vector getFeet2BVelocities(int index) const;

    /**
     * 计算所有足端速度。
     */
    [[nodiscard]] std::vector<KDL::Vector> getFeet2BVelocities() const;

    double mass_ = 0;
    Vec34 feet_pos_normal_stand_;
    std::vector<KDL::JntArray> current_joint_pos_;
    std::vector<KDL::JntArray> current_joint_vel_;

    void update();

private:
    CtrlComponent &ctrl_component_;
    std::vector<std::shared_ptr<RobotLeg> > robot_legs_;

    KDL::Chain fr_chain_;
    KDL::Chain fl_chain_;
    KDL::Chain rr_chain_;
    KDL::Chain rl_chain_;
};


#endif //QUADRUPEDROBOT_H
