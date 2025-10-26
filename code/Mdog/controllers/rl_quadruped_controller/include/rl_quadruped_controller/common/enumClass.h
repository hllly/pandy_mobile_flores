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
    RL,
};

enum class FSMMode {
    NORMAL,
    CHANGE
};

#endif //ENUMCLASS_H
