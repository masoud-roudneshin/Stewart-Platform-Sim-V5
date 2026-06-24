#include "../geometry/Geometry.h"
#include <cmath>

PlatformGeometry::PlatformGeometry( real_t radius_Base_,
                                    real_t radius_Platform_,
                                    real_t jointHalfAngleOffset_Base_,
                                    real_t jointHalfAngleOffset_Platform_,
                                    const Vec3& jointsCentralAngles_Base_,
                                    const Vec3& jointsCentralAngles_Platform_) :
                                    radius_Base(radius_Base_),
                                    radius_Platform(radius_Platform_),
                                    jointHalfAngleOffset_Base(jointHalfAngleOffset_Base_),
                                    jointHalfAngleOffset_Platform(jointHalfAngleOffset_Platform_),
                                    jointsCentralAngles_Base(jointsCentralAngles_Base_),
                                    jointsCentralAngles_Platform(jointsCentralAngles_Platform_)
{

    const std::array<int, 6>    angleIdx_b = { 0,  1,  1,  2,  2,  0 };
    const Vec6 signs_b(+ 1.0, -1.0, +1.0, -1.0, +1.0, -1.0);

    for (int i = 0; i < 6; i++)
    {
        // Base — explicit mapping
        real_t angle_base = jointsCentralAngles_Base(angleIdx_b[i])
            + signs_b(i) * jointHalfAngleOffset_Base;

        jointCoordinates_Base.col(i) << radius_Base * std::cos(angle_base), radius_Base * std::sin(angle_base), 0.0;

        // Platform — original pattern unchanged
        real_t sign_p = (i % 2 == 0) ? -1.0 : +1.0;
        int    angleIndex_p = i / 2;

        real_t angle_platform = jointsCentralAngles_Platform(angleIndex_p)
            + sign_p * jointHalfAngleOffset_Platform;

        jointCoordinates_Platform.col(i) << radius_Platform * std::cos(angle_platform), radius_Platform* std::sin(angle_platform), 0.0;

    }



}

// Constructor C
PlatformGeometry::PlatformGeometry(
    const Mat3x6& base,
    const Mat3x6& platform)
    : radius_Base(0.0)
    , radius_Platform(0.0)
    , jointHalfAngleOffset_Base(0.0)
    , jointHalfAngleOffset_Platform(0.0)
    , jointCoordinates_Base(base)
    , jointCoordinates_Platform(platform)
{
    // central angles not meaningful here — we zero them
    jointsCentralAngles_Base        << 0.0,0.0,0.0;
    jointsCentralAngles_Platform    << 0.0, 0.0, 0.0;
}