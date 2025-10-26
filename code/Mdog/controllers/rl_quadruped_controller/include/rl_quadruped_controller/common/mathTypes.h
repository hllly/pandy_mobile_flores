/**********************************************************************
 版权 (c) 2020-2023，Unitree Robotics.Co.Ltd. 保留所有权利。
***********************************************************************/
#ifndef MATHTYPES_H
#define MATHTYPES_H

#include <eigen3/Eigen/Dense>

/************************/
/******** 向量 *********/
/************************/
// 2x1 向量
using Vec2 = Eigen::Matrix<double, 2, 1>;

// 3x1 向量
using Vec3 = Eigen::Matrix<double, 3, 1>;

// 4x1 向量
using Vec4 = Eigen::Matrix<double, 4, 1>;

// 6x1 向量
using Vec6 = Eigen::Matrix<double, 6, 1>;

// 四元数
using Quat = Eigen::Matrix<double, 4, 1>;

// 4x1 整型向量
using VecInt4 = Eigen::Matrix<int, 4, 1>;

// 12x1 向量
using Vec12 = Eigen::Matrix<double, 12, 1>;

// 18x1 向量
using Vec18 = Eigen::Matrix<double, 18, 1>;

// 动态长度向量
using VecX = Eigen::Matrix<double, Eigen::Dynamic, 1>;

/************************/
/******** 矩阵 *********/
/************************/
// 旋转矩阵
using RotMat = Eigen::Matrix<double, 3, 3>;

// 齐次矩阵
using HomoMat = Eigen::Matrix<double, 4, 4>;

// 2x2 矩阵
using Mat2 = Eigen::Matrix<double, 2, 2>;

// 3x3 矩阵
using Mat3 = Eigen::Matrix<double, 3, 3>;

// 3x3 单位矩阵
#define I3 Eigen::MatrixXd::Identity(3, 3)

// 3x4 矩阵，每列为一个 3x1 向量
using Vec34 = Eigen::Matrix<double, 3, 4>;

// 6x6 矩阵
using Mat6 = Eigen::Matrix<double, 6, 6>;

// 12x12 矩阵
using Mat12 = Eigen::Matrix<double, 12, 12>;

// 12x12 单位矩阵
#define I12 Eigen::MatrixXd::Identity(12, 12)

// 18x18 单位矩阵
#define I18 Eigen::MatrixXd::Identity(18, 18)

// 动态尺寸矩阵
using MatX = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

/************************/
/****** 函数 *******/
/************************/
inline Vec34 vec12ToVec34(Vec12 vec12) {
  Vec34 vec34;
  for (int i(0); i < 4; ++i) {
    vec34.col(i) = vec12.segment(3 * i, 3);
  }
  return vec34;
}

inline Vec12 vec34ToVec12(Vec34 vec34) {
  Vec12 vec12;
  for (int i(0); i < 4; ++i) {
    vec12.segment(3 * i, 3) = vec34.col(i);
  }
  return vec12;
}

#endif  // MATHTYPES_H
