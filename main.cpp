#include <Eigen/Dense>
#include<array>
#include <memory>
#include <thread>
#include <iostream>
#include "geometry/Geometry.h"
#include "threading/SharedData.h"
#include "threading/ThreadedFOCDriver.h"
#include "threading/ThreadedController.h"
#include "threading/Logger.h"
#include "actuator/MotorDriver.h"
#include "geometry/Kinematics.h"
#include "platform/StewartPlatform.h"
#include "threading/ThreadedSafetyMonitor.h"
#include "math/Math.h"
#include "control/IController.h"
#include "control/JointSpaceController.h"
#include "control/StewartController.h"
#include "control/TaskSpaceController.h"

int main()
{

    // Make Platform Geometry

    PlatformGeometry geom(1.0, 0.5, 5.0 * PI / 180.0, 5.0 * PI / 180.0);
    double body_length = 0.8;
    MotorParameters motor;
    LeadScrewParameters screw;
    real_t force_to_iq_gain = screw.lead / (2.0 * PI * motor.Kt * screw.efficiency);

    // Create temporary platform just to get geometry info
    StewartPlatform platform(geom, body_length, 0.6, 500.0, 10.0, 0.707, 0.0001);
    real_t mid_heave = platform.get_mid_heave();

    // Make Actuators Shared Data between the threads (and also their pointers)

    std::array<std::unique_ptr<ActuatorSharedData>, 6> shared_data;

    for (size_t i = 0; i < 6; i++)
    {
        shared_data[i] = std::make_unique<ActuatorSharedData>();
    }

    std::array<ActuatorSharedData*, 6> shared_ptrs;

    for (size_t i = 0; i < 6; i++)
    {
        shared_ptrs[i] = shared_data[i].get();
    }

    // Making the threaded objects

    // Make FOC Threaded Driver
    ThreadedSafetyMonitor safety(shared_ptrs, 2.0, 500.0, 0.6);

    std::array<std::unique_ptr<ThreadedFOCDriver>, 6> foc_drivers;

    for (size_t i = 0; i < 6; i++)
    {
        foc_drivers[i] = std::make_unique<ThreadedFOCDriver>(*shared_data[i], safety);
    }

    // Controller
    Vec6 Kp_gains, Kd_gains;
    Kp_gains << 1000, 1000, 1000, 500, 500, 500;
    Kd_gains << 100, 100, 100, 50, 50, 50;

    Kp_gains *= 5.0;
    Kd_gains *= 3.0;
    real_t dt = 0.001;

    

    ThreadedController controller(shared_ptrs,
        safety,
        std::make_unique<TaskSpaceController>(Kp_gains, Kd_gains),
        geom,
        mid_heave,
        force_to_iq_gain,
        dt); // computed above);
    Logger logger(shared_ptrs, safety);

    // Set Controller
    Pose6DoF target_pos;
    target_pos.z = 0.05;
    target_pos.roll = 0.1;

    target_pos.z += mid_heave;   // offset by mid_heave

    Vec6 leg_lengths;
    Mat3x6 actuator_unit_vector;
    Mat3x6 platform_joints_world_coords;

    Kinematics::compute_InverseKinematics(geom, target_pos, leg_lengths, actuator_unit_vector, platform_joints_world_coords);
    Vec6 desired_strokes = leg_lengths.array() - body_length;
    controller.set_target(desired_strokes, target_pos);  // target_pos is already Pose6DoF
    



    // Starting the threads
    safety.start();
    for (int i = 0; i < 6; i++) { foc_drivers[i]->start(); }

    controller.start();

    logger.start();

    // Wait for the user's input

    /*
    std::cout << "Press Enter to stop...\n";
    std::cin.get();
    safety.estop();
    */

    // Wait 2 seconds then trigger ESTOP test
    std::this_thread::sleep_for(std::chrono::seconds(2));
    std::cout << "\nTriggering ESTOP...\n" << std::flush;
    safety.estop();
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // longer wait
    std::cout << "\n\n=== ESTOP TRIGGERED ===\n" << std::flush;
    std::cout << "State: " << static_cast<int>(safety.get_state()) << "\n" << std::flush;
    std::cout << "Press Enter to exit...\n" << std::flush;

    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::cout << "Stopping logger...\n" << std::flush;
    logger.stop();
    std::cout << "Stopping controller...\n" << std::flush;
    controller.stop();
    std::cout << "Stopping FOC...\n" << std::flush;
    for (int i = 0; i < 6; i++) foc_drivers[i]->stop();
    std::cout << "Stopping safety...\n" << std::flush;
    safety.stop();
    std::cout << "All stopped.\n" << std::flush;

    return 0;
}