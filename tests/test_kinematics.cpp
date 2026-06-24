#include <gtest/gtest.h>
#include "math/Math.h"
#include "geometry/Geometry.h"
#include "geometry/Kinematics.h"
#include "platform/StewartPlatform.h"


// ─── Test Fixture ───────────────────────────────────────────
// Shared setup reused across all kinematics tests
class KinematicsTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Same geometry used throughout V1-V3
        geom = PlatformGeometry(1.0, 0.5,
            5.0 * PI / 180.0,
            5.0 * PI / 180.0);

        real_t body_length = 0.8;
        StewartPlatform platform(geom, body_length, 0.6, 500.0, 10.0, 0.707, 0.0001);
        mid_heave = platform.get_mid_heave();
    }

    PlatformGeometry geom;
    real_t           mid_heave;
};

// ─── IK Tests ───────────────────────────────────────────────

TEST_F(KinematicsTest, PureHeaveGivesEqualLegLengths)
{
    // Pure heave with symmetric geometry → all 6 legs equal
    Pose6DoF pose;
    pose.z = mid_heave;

    Vec6 leg_lengths;
    Mat3x6 unit_vectors, joints_world;
    Kinematics::compute_InverseKinematics(geom, pose,
        leg_lengths, unit_vectors, joints_world);

    for (int i = 1; i < 6; i++)
        EXPECT_NEAR(leg_lengths(i), leg_lengths(0), 1e-6)
        << "Leg " << i << " differs from leg 0";
}



TEST_F(KinematicsTest, UnitVectorsHaveUnitLength)
{
    Pose6DoF pose;
    pose.z = 0.05 + mid_heave;
    pose.roll = 0.1;

    Vec6 leg_lengths;
    Mat3x6 unit_vectors, joints_world;
    Kinematics::compute_InverseKinematics(geom, pose,
        leg_lengths, unit_vectors, joints_world);

    for (int i = 0; i < 6; i++)
        EXPECT_NEAR(unit_vectors.col(i).norm(), 1.0, 1e-10)
        << "Unit vector " << i << " is not unit length";
}

// ─── FK Tests ───────────────────────────────────────────────

TEST_F(KinematicsTest, FKConvergesFromGoodInitialGuess)
{
    Pose6DoF true_pose;
    true_pose.z = 0.05 + mid_heave;
    true_pose.roll = 0.1;

    // Get leg lengths from IK
    Vec6 leg_lengths;
    Mat3x6 unit_vectors, joints_world;
    Kinematics::compute_InverseKinematics(geom, true_pose,
        leg_lengths, unit_vectors, joints_world);

    // Run FK from close initial guess
    Pose6DoF estimated;
    estimated.z = mid_heave;

    bool converged = Kinematics::compute_forward_kinematics(
        geom, leg_lengths, estimated);

    EXPECT_TRUE(converged);
}

TEST_F(KinematicsTest, FKRecoversTruePosition)
{
    Pose6DoF true_pose;
    true_pose.z = 0.05 + mid_heave;
    true_pose.roll = 0.1;

    Vec6 leg_lengths;
    Mat3x6 unit_vectors, joints_world;
    Kinematics::compute_InverseKinematics(geom, true_pose,
        leg_lengths, unit_vectors, joints_world);

    Pose6DoF estimated;
    estimated.z = mid_heave;
    Kinematics::compute_forward_kinematics(geom, leg_lengths, estimated);

    EXPECT_NEAR(estimated.z, true_pose.z, 1e-4);
    EXPECT_NEAR(estimated.roll, true_pose.roll, 1e-4);
}

TEST_F(KinematicsTest, FKIKRoundTrip)
{
    // FK(IK(pose)) should recover original pose
    Pose6DoF original;
    original.z = 0.05 + mid_heave;
    original.roll = 0.1;
    original.pitch = 0.05;

    // IK → get leg lengths
    Vec6 leg_lengths;
    Mat3x6 unit_vectors, joints_world;
    Kinematics::compute_InverseKinematics(geom, original,
        leg_lengths, unit_vectors, joints_world);

    // FK → recover pose
    Pose6DoF recovered;
    recovered.z = mid_heave;
    bool converged = Kinematics::compute_forward_kinematics(
        geom, leg_lengths, recovered);

    EXPECT_TRUE(converged);
    EXPECT_NEAR(recovered.z, original.z, 1e-4);
    EXPECT_NEAR(recovered.roll, original.roll, 1e-4);
    EXPECT_NEAR(recovered.pitch, original.pitch, 1e-4);
}

TEST_F(KinematicsTest, NeutralPoseGivesZeroRoll)
{
    // At neutral pose — roll and pitch should be zero
    Pose6DoF pose;
    pose.z = mid_heave;

    Vec6 leg_lengths;
    Mat3x6 unit_vectors, joints_world;
    Kinematics::compute_InverseKinematics(geom, pose,
        leg_lengths, unit_vectors, joints_world);

    Pose6DoF recovered;
    recovered.z = mid_heave;
    Kinematics::compute_forward_kinematics(geom, leg_lengths, recovered);

    EXPECT_NEAR(recovered.roll, 0.0, 1e-6);
    EXPECT_NEAR(recovered.pitch, 0.0, 1e-6);
    EXPECT_NEAR(recovered.yaw, 0.0, 1e-6);
}