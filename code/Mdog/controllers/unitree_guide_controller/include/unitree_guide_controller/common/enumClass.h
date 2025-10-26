//
// 由 pj 于 24-9-6 创建。
//

#ifndef ENUMCLASS_H
#define ENUMCLASS_H

enum class FSMStateName {
    // 退出
    INVALID,
    PASSIVE,
    FIXEDDOWN,
    FIXEDSTAND,
    FREESTAND,
    TROTTING,

    SWINGTEST,
    BALANCETEST,
};

enum class FSMMode {
    NORMAL,
    CHANGE
};

enum class FrameType {
    BODY,
    HIP,
    GLOBAL
};

enum class WaveStatus {
    STANCE_ALL,
    SWING_ALL,
    WAVE_ALL
};

#endif //ENUMCLASS_H
