#pragma once
#include <Eigen/Eigen>
#include "../so3_math.h"
#include "commons.h"

using M12D = Eigen::Matrix<double, 12, 12>;
using M21D = Eigen::Matrix<double, 21, 21>;

using V12D = Eigen::Matrix<double, 12, 1>;
using V21D = Eigen::Matrix<double, 21, 1>;
using M21X12D = Eigen::Matrix<double, 21, 12>;

M3D Jr(const V3D &inp);
M3D JrInv(const V3D &inp);

struct SharedState
{
    // 点面残差只直接观测前 12 维位姿/外参；其余状态通过协方差耦合更新。
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    M12D H;
    V12D b;
    double res = 1e10;
    bool valid = false;
    size_t iter_num = 0;
};
struct Input
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    V3D acc;
    V3D gyro;
    Input() = default;
    Input(V3D &a, V3D &g) : acc(a), gyro(g) {}
    Input(double a1, double a2, double a3, double g1, double g2, double g3) : acc(a1, a2, a3), gyro(g1, g2, g3) {}
};
struct State
{
    // 21 维误差顺序：世界姿态、位置、雷达到 IMU 旋转/平移、速度、陀螺/加计零偏。
    // r_ab/t_ab 将 b 系点转换到 a 系；重力方向初始化后固定，不在误差向量内估计。
    static double gravity;
    M3D r_wi = M3D::Identity();
    V3D t_wi = V3D::Zero();
    M3D r_il = M3D::Identity();
    V3D t_il = V3D::Zero();
    V3D v = V3D::Zero();
    V3D bg = V3D::Zero();
    V3D ba = V3D::Zero();
    V3D g = V3D(0.0, 0.0, -9.81);

    void initGravityDir(const V3D &gravity_dir) { g = gravity_dir.normalized() * State::gravity; }

    void operator+=(const V21D &delta);

    V21D operator-(const State &other) const;

    friend std::ostream &operator<<(std::ostream &os, const State &state);
};

using loss_func = std::function<void(State &, SharedState &)>;
using stop_func = std::function<bool(const V21D &)>;

class IESKF
{
    // 预测由 IMU 驱动，更新通过回调取得当前线性化点的点面约束与停止条件。
public:
    IESKF() = default;
    void setMaxIter(size_t iter) { m_max_iter = iter; }
    void setLossFunction(loss_func func) { m_loss_func = func; }
    void setStopFunction(stop_func func) { m_stop_func = func; }

    void predict(const Input &inp, double dt, const M12D &Q);

    void update();

    State &x() { return m_x; }

    M21D &P() { return m_P; }

private:
    size_t m_max_iter = 10;
    State m_x;
    M21D m_P;
    loss_func m_loss_func;
    stop_func m_stop_func;
    M21D m_F;
    M21X12D m_G;
};
