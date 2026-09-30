#include "rotation.hpp"
#include <cmath>

namespace attitude
{

namespace { constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0; }

Mat3 mul(const Mat3& A, const Mat3& B)
{
    Mat3 C{};
    for(int r=0; r<3; ++r) {
        for(int c=0; c<3; ++c) {
            C[r*3 + c] = A[r*3+0] * B[0*3+c]
                        + A[r*3+1] * B[1*3+c]
                        + A[r*3+2] * B[2*3+c];
        }
    }
    return C;
}

Mat3 transpose(const Mat3& A)
{
    return { A[0],A[3],A[6], A[1],A[4],A[7], A[2],A[5],A[8] };
}

Mat3 bodyToEnu(double roll_deg, double pitch_deg, double yaw_deg)
{
    const double cr = std::cos(roll_deg * kDeg2Rad), sr = std::sin(roll_deg * kDeg2Rad);
    const double cp = std::cos(pitch_deg * kDeg2Rad), sp = std::sin(pitch_deg * kDeg2Rad);
    const double cy = std::cos(yaw_deg * kDeg2Rad), sy = std::sin(yaw_deg * kDeg2Rad);

    const Mat3 Rz{ cy,sy,0, -sy,cy,0, 0,0,1 };
    const Mat3 Rx{ 1,0,0, 0,cp,-sp, 0,sp,cp };
    const Mat3 Ry{ cr,0,sr, 0,1,0, -sr,0,cr};

    return mul(Rz, mul(Rx, Ry));
}

}   // namespace attitude
