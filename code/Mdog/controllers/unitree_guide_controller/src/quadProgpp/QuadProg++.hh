/*
 文件 $Id: QuadProg++.hh 232 2007-06-21 12:29:00Z digasper $

 函数 quadprog_solve() 实现了 Goldfarb 和 Idnani 的算法，
 通过主动集对偶方法求解（凸）二次规划问题。

问题形式如下：

min 0.5 * x G x + g0 x
约束条件：
    CE^T x + ce0 = 0
    CI^T x + ci0 >= 0

 矩阵与向量的维度如下：
     G: n * n
     g0: n

     CE: n * p
     ce0: p

     CI: n * m
     ci0: m

     x: n

 函数将返回写入向量 x 的解对应的代价；若问题无可行解，则返回
 std::numeric_limits::infinity()，此时向量 x 的值无效。

 参考文献：D. Goldfarb, A. Idnani. A numerically stable dual method for
 solving strictly convex quadratic programs. Mathematical Programming 27 (1983)
 pp. 1-33.

 注意：
  1. 配置向量 ce0 和 ci0 时需格外留意。
     如果约束写成 A^T x = b 和 C^T x >= d，
     则应设置 ce0 = -b，ci0 = -d。
  2. 这些矩阵的列维度等于 MATRIX_DIM，
     本文件通过 #define 将其默认设为 20。
     若矩阵尺寸大于 20 x 20，可在编译器命令行
     通过 -DMATRIX_DIM=n 来放宽限制。
  3. 函数内部会修改矩阵 G，以便计算
     G = L^T L 的 Cholesky 分解用于后续计算。
     若需要保留原始矩阵 G，请先复制再传入函数。

 作者：Luca Di Gaspero
       DIEGM - 乌迪内大学（意大利）
       luca.digaspero@uniud.it
       http://www.diegm.uniud.it/digaspero/

 若研究者在其论文中引用本软件，作者将深表感谢。

 版权所有 (c) 2007-2016 Luca Di Gaspero

 本软件可依据 MIT 许可证条款进行修改与分发，
 详情请参阅 LICENSE 文件。
*/

#ifndef _QUADPROGPP
#define _QUADPROGPP

#include "Array.hh"
#include <eigen3/Eigen/Dense>

namespace quadprogpp {

double solve_quadprog(Matrix<double>& G, Vector<double>& g0,
                      const Matrix<double>& CE, const Vector<double>& ce0,
                      const Matrix<double>& CI, const Vector<double>& ci0,
                      Vector<double>& x);

}  // namespace quadprogpp

#endif  // #define _QUADPROGPP
