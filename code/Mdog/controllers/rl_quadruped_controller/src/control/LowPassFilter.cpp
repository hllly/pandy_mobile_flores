//
// 由 pj 于 24-9-16 创建。
//
#include <cmath>
#include "rl_quadruped_controller/control/LowPassFilter.h"

/**
 * 低通滤波器，用于抑制高频信号。
 * @param samplePeriod 采样周期
 * @param cutFrequency 截止频率
 */
LowPassFilter::LowPassFilter(const double samplePeriod, const double cutFrequency) {
    weight_ = 1.0 / (1.0 + 1.0 / (2.0 * M_PI * samplePeriod * cutFrequency));
    start_ = false;
}

void LowPassFilter::addValue(const double newValue) {
    if (!start_) {
        start_ = true;
        pass_value_ = newValue;
    }
    pass_value_ = weight_ * newValue + (1 - weight_) * pass_value_;
}

double LowPassFilter::getValue() const {
    return pass_value_;
}

void LowPassFilter::clear() {
    start_ = false;
}
