#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <stdexcept>

using real_t = double;
constexpr real_t PI = 3.14159265358979323846;
using Vec3      = Eigen::Vector3d;
using Mat3      = Eigen::Matrix3d;
using Vec6      = Eigen::Matrix<real_t, 6, 1>;
using Mat6      = Eigen::Matrix<real_t, 6, 6>;
using Mat3x6    = Eigen::Matrix<real_t, 3, 6>; 


inline Mat3 compute_rotation_matrix(real_t roll, real_t pitch, real_t yaw) noexcept
{
    return (Eigen::AngleAxisd(yaw, Vec3::UnitZ()) *
            Eigen::AngleAxisd(pitch, Vec3::UnitY()) *
            Eigen::AngleAxisd(roll, Vec3::UnitX())).toRotationMatrix();
}


// ----------------- Clarke & Park Transformations -------------
// Clarke: 3-phase → 2-phase stationary frame
// Assumes ia + ib + ic = 0, so ic = -ia - ib
struct AlphaBeta { real_t alpha, beta; };
struct DQ { real_t d, q; };

inline AlphaBeta clarke_transform(real_t ia, real_t ib) noexcept
{
    return { ia , (ia + 2 * ib) / std::sqrt(3) };
}

inline DQ  park_transform(const AlphaBeta& ab, real_t theta_e) noexcept
{
    const real_t c = std::cos(theta_e), s = std::sin(theta_e);

    return {ab.alpha * c + ab.beta * s,
           -ab.alpha * s + ab.beta * c};
}
inline AlphaBeta inverse_park_transform(const DQ& dq, real_t theta_e) noexcept
{
    const real_t c = std::cos(theta_e), s = std::sin(theta_e);

    return { dq.d * c - dq.q * s, dq.d* s + dq.q * c };
}

