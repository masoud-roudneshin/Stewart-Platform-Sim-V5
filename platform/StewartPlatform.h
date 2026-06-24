#pragma once
#include <memory>
#include <array>
#include <stdexcept>
#include "../geometry/Geometry.h"
#include "../actuator/ActuatorState.h"

class StewartPlatform
{
private:
    Pose6DoF current_pose;
    PlatformGeometry geometry;
    std::array<std::unique_ptr<ActuatorState>, 6>   actuators;
    Vec6                                            jointToJoint_lengths = Vec6::Zero();
    Mat3x6                                          actuators_unit_vector = Mat3x6::Zero();
    Mat3x6                                          jointCoordinates_Platform_World = Mat3x6::Zero();
    real_t                                          min_heave{ 0.0 };
    real_t                                          max_heave{ 0.0 };
    real_t                                          mid_heave{ 0.0 };

    void validate_geometry(real_t body_length_, real_t max_stroke_);

public:
    StewartPlatform(const PlatformGeometry& geometry_,
        real_t body_length = 0.45,
        real_t max_stroke = 0.3,
        real_t max_force = 500.0,
        real_t wn = 50.0,
        real_t zeta = 0.707,
        real_t dt = 0.0001,
        MotorParameters motor = MotorParameters(),
        LeadScrewParameters screw = LeadScrewParameters(),
        real_t V_supply = 200.0);

    StewartPlatform(const StewartPlatform&)                 = delete;
    StewartPlatform& operator=(const StewartPlatform&)      = delete;
    StewartPlatform(StewartPlatform&&) noexcept             = default;
    StewartPlatform& operator=(StewartPlatform&&) noexcept  = default;

    void set_actuator_target(int index, real_t total_length);
    void set_pose(const Pose6DoF& pose_);
    void update(real_t dt_step);
    void update_task_space(const Vec6& leg_forces, real_t dt);

    const ActuatorState& get_actuator(int index) const;
    ActuatorState& get_actuator_mutable(int index);
    Pose6DoF get_pose()      const noexcept;
    real_t   get_min_heave() const noexcept;
    real_t   get_max_heave() const noexcept;
    real_t   get_mid_heave() const noexcept;
    void     print_all()     const;
};