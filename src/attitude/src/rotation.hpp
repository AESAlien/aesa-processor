#pragma once
#include <array>

namespace attitude
{

using Mat3 = std::array<double, 9>;

constexpr Mat3 kIdentity{1,0,0, 0,1,0, 0,0,1};

Mat3 mul(const Mat3& A, const Mat3& B);

Mat3 transpose(const Mat3& A);

Mat3 bodyToEnu(double roll_deg, double pitch_deg, double yaw_deg);

}   // namespace attitude