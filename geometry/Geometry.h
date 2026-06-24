#pragma once
#include <array>
#include <cmath>
#include "../math/Math.h"

struct Pose6DoF
{
    real_t x{ 0.0 }, y{ 0.0 }, z{ 0.0 }; // Translational DoFs surge, sway and heave in m or mm
    real_t roll{ 0.0 }, pitch{ 0.0 }, yaw{ 0.0 }; // Rotation DoFs as Euler Angles in radians where R = R_z * R_y * R_x;

};

struct PlatformGeometry
{

    // Joints Data Assuming Circular Arrangment
    real_t radius_Base; // base radius in m or mm
    real_t radius_Platform; // platform radius in m or mm
    real_t jointHalfAngleOffset_Base; // half offset angle between joints on the base
    real_t jointHalfAngleOffset_Platform;// half offset angle between joints on the paltform

    Vec3 jointsCentralAngles_Base; // Joints Central angle of attachement in radians for the base
    Vec3 jointsCentralAngles_Platform; // Joints Central angle of attachement in radians for the platform

    Mat3x6 jointCoordinates_Base; // joints coordinates as 6 vectors each in R^3 for the base 
    Mat3x6 jointCoordinates_Platform; // joints coordinates as 6 vectors each in R^3 for the platform


    // Constructor A

    PlatformGeometry(real_t radius_Base_, 
                     real_t radius_Platform_, 
                     real_t jointHalfAngleOffset_Base_, 
                     real_t jointHalfAngleOffset_Platform_) :
                     PlatformGeometry(radius_Base_, 
                                      radius_Platform_, 
                                      jointHalfAngleOffset_Base_, 
                                      jointHalfAngleOffset_Platform_,
                                      { 0, 2 * PI / 3.0, 4 * PI / 3.0 }, 
                                      { PI / 3, PI / 3 + 2 * PI / 3.0, PI / 3 + 4 * PI / 3.0 }) {};

    // Constructor B
    PlatformGeometry(   real_t radius_Base_,
                        real_t radius_Platform_,
                        real_t jointHalfAngleOffset_Base_,
                        real_t jointHalfAngleOffset_Platform_,
                        const Vec3& jointsCentralAngles_Base_,
                        const Vec3& jointsCentralAngles_Platform_);

    // Constructor C
    PlatformGeometry(const Mat3x6& base,
                     const Mat3x6& platform);


    // Default Constructor

    PlatformGeometry() : PlatformGeometry(0.3, 0.2, 20.0 * PI / 180.0, 20.0 * PI / 180.0) {}


};
