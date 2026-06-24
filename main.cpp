#include <Eigen/Dense>
#include <iostream>
#include <iomanip>
#include "math/Math.h"
#include "geometry/Geometry.h"
#include "geometry/Kinematics.h"
#include "platform/StewartPlatform.h"
#include "control/TaskSpaceController.h"
#include "threading/SharedData.h" 
#include "control/IController.h"
#include "control/JointSpaceController.h"
#include "control/StewartController.h"


int main()
{
    ControlMode mode = ControlMode::TASK_SPACE;

    std::cout << "=== Stewart Platform V3 — IK Test ===\n\n";

    // 1. Create Geometry
    PlatformGeometry geom(1.0, 0.5, 5.0 * PI / 180.0, 5.0 * PI / 180.0);
    real_t body_length = 0.8;
    LeadScrewParameters screw;
    MotorParameters motor;

    // Create temporary platform just to get geometry info
    StewartPlatform platform(geom, body_length, 0.6, 500.0, 10.0, 0.707, 0.0001);
    real_t mid_heave = platform.get_mid_heave();

    // Desired pose
    Pose6DoF desired;
    desired.z = 0.05 + mid_heave;
    desired.roll = 0.1;

    // Controller gains
    Vec6 Kp_gains, Kd_gains;
    Kp_gains << 1000, 1000, 1000, 500, 500, 500;
    Kd_gains << 100, 100, 100, 50, 50, 50;

    Kp_gains *= 5.0;
    Kd_gains *= 3.0;

    StewartController controller;
    //controller.set_strategy(std::make_unique<TaskSpaceController>(Kp_gains, Kd_gains));
    controller.set_strategy(std::make_unique<JointSpaceController>(2.0,5.0,10,0.7,0.001));

    // FK initial guess
    Pose6DoF actual_pose;
    actual_pose.z = mid_heave;  // neutral

    // Previous pose for velocity estimation
    Pose6 prev_pose_vec = Kinematics::pose_to_vec(actual_pose);

    double dt = 0.001;


    // For JointSpace Control

    Vec6 target_leg_lengths;
    Mat3x6 dummy1, dummy2;
    Kinematics::compute_InverseKinematics(geom, desired,
        target_leg_lengths, dummy1, dummy2);

    Vec6 L_dot;
    Vec6 desired_vec = target_leg_lengths;
    Vec6 actual_vec;

    for (int step = 0; step <= 5000; step++)
    {
        // 1. Run IK on actual pose to get unit vectors and Jacobian
        Vec6   leg_lengths;
        Mat3x6 unit_vectors;
        Mat3x6 platform_joints_world;
        Kinematics::compute_InverseKinematics(geom, actual_pose,
            leg_lengths, unit_vectors, platform_joints_world);

        Mat6 J;
        Kinematics::compute_Jacobian(actual_pose,
            unit_vectors, platform_joints_world, J);

        // 2. Estimate velocity from finite differences
        Pose6 curr_pose_vec = Kinematics::pose_to_vec(actual_pose);

        for (int i = 0; i < 6; i++)
            L_dot(i) = platform.get_actuator(i).get_velocity();

        // Convert to task-space velocity
        Vec6 actual_velocity = J.colPivHouseholderQr().solve(L_dot);

        // 3. Compute leg forces from task-space controller
        // ------------- TaskSpace Control ---------------------

        // desired_vec = Kinematics::pose_to_vec(desired);
        // actual_vec = Kinematics::pose_to_vec(actual_pose);

        //Vec6 leg_forces = controller.compute(desired_vec, actual_vec, actual_velocity, J);
        //------------ JointSpace Control -----------------------
        for (int i = 0; i < 6; i++)
            actual_vec(i) = platform.get_actuator(i).get_total_length();

        Vec6 leg_forces = controller.compute(desired_vec, actual_vec, L_dot, J);

        //
        Vec6 measured_lengths;

        // 4. Apply forces and update based on mode
        platform.update(leg_forces);

        // 5. Get actual leg lengths from actuators for FK

        for (int i = 0; i < 6; i++)
            measured_lengths(i) = platform.get_actuator(i).get_total_length();

        // 6. Run FK to get actual pose
        Kinematics::compute_forward_kinematics(geom, measured_lengths, actual_pose);

        // 7. Print every 100 steps
        if (step % 500 == 0)
        {
            std::cout << "t=" << step * dt * 1000 << "ms"
                << "  actual z = " << std::fixed << std::setprecision(4) << actual_pose.z
                << "  actual roll = " << actual_pose.roll
                << "  (target z = " << desired.z << " target roll = " << desired.roll << ")\n";

            std::cout << "leg_forces: " << leg_forces.transpose() << "\n";
            std::cout << "actual forces: ";
            for (int i = 0; i < 6; i++)
                std::cout << platform.get_actuator(i).get_force() << " ";
            std::cout << "\n";
            std::cout << "actual strokes: ";
            for (int i = 0; i < 6; i++)
                std::cout << platform.get_actuator(i).get_stroke() << " ";
            std::cout << "\n\n";
        }
    }



    return 0;
}