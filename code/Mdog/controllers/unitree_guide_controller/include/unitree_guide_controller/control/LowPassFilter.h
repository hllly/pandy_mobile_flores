//
// 由 pj 于 24-9-16 创建。
//


#ifndef LOWPASSFILTER_H
#define LOWPASSFILTER_H


class LowPassFilter {
public:
    LowPassFilter(double samplePeriod, double cutFrequency);

    ~LowPassFilter() = default;

    void addValue(double newValue);

    [[nodiscard]] double getValue() const;

    void clear();

private:
    double weight_;
    double pass_value_{};
    bool start_;
};


#endif //LOWPASSFILTER_H
