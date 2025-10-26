//
// 由 pj 于 24-9-18 创建。
//


#ifndef WAVEGENERATOR_H
#define WAVEGENERATOR_H
#include <chrono>
#include <unitree_guide_controller/common/enumClass.h>
#include <unitree_guide_controller/common/mathTypes.h>

inline long long getSystemTime() {
    const auto now = std::chrono::system_clock::now();
    const auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}

class WaveGenerator {
public:
    WaveGenerator(double period, double st_ratio, const Vec4 &bias);

    ~WaveGenerator() = default;

    void update();

    [[nodiscard]] double get_t_stance() const { return period_ * st_ratio_; }
    [[nodiscard]] double get_t_swing() const { return period_ * (1 - st_ratio_); }
    [[nodiscard]] double get_t() const { return period_; }

    Vec4 phase_;
    VecInt4 contact_;
    WaveStatus status_{};

private:
    /**
     * 根据当前时间更新步态相位、接触状态与有限状态机状态。
     * @param phase 足端相位
     * @param contact 足端接触标识
     * @param status 波形状态枚举
     */
    void calcWave(Vec4 &phase, VecInt4 &contact, WaveStatus status);

    double period_{};
    double st_ratio_{}; // 支撑相所占比例
    Vec4 bias_;

    Vec4 normal_t_; // 归一化时间区间 [0,1)
    Vec4 phase_past_; // 历史足端相位
    VecInt4 contact_past_; // 历史足端接触状态
    VecInt4 switch_status_;
    WaveStatus status_past_;

    long start_t_{};
};


#endif //WAVEGENERATOR_H
