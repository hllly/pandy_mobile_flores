//
// 由 pj 于 24-9-18 创建。
//


#ifndef GAITGENERATOR_H
#define GAITGENERATOR_H
#include <memory>
#include <unitree_guide_controller/common/mathTypes.h>

#include "FeetEndCalc.h"


class Estimator;
class WaveGenerator;
struct CtrlComponent;

class GaitGenerator {
public:
    explicit GaitGenerator(CtrlComponent &ctrl_component);

    ~GaitGenerator() = default;

    void setGait(Vec2 vxy_goal_global, double d_yaw_goal, double gait_height);

    void generate(Vec34 &feet_pos, Vec34 &feet_vel);

    void restart();

private:
    Vec3 getFootPos(int i);

    Vec3 getFootVel(int i);

    /**
     * 计算足端在 XY 平面上的位置。
     * @param startXY 起始坐标
     * @param endXY 目标坐标
     * @param phase 相位
     */
    static double cycloidXYPosition(double startXY, double endXY, double phase);

    /**
     * 计算足端在 Z 方向上的高度。
     * @param startZ 初始高度
     * @param height 抬腿高度
     * @param phase 相位
     */
    static double cycloidZPosition(double startZ, double height, double phase);

    /**
     * 计算足端在 XY 平面上的速度。
     * @param startXY 起始坐标
     * @param endXY 目标坐标
     * @param phase 相位
     */
    [[nodiscard]] double cycloidXYVelocity(double startXY, double endXY, double phase) const;

    /**
     * 计算足端在 Z 方向上的速度。
     * @param height 抬腿高度
     * @param phase 相位
     */
    [[nodiscard]] double cycloidZVelocity(double height, double phase) const;

    std::shared_ptr<WaveGenerator> &wave_generator_;
    std::shared_ptr<Estimator> &estimator_;
    FeetEndCalc feet_end_calc_;

    double gait_height_{};
    Vec2 vxy_goal_;
    double d_yaw_goal_{};
    Vec34 start_p_, end_p_, ideal_p_, past_p_;
    bool first_run_;
};


#endif //GAITGENERATOR_H
