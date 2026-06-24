#include "../platform/StewartPlatform.h"  
#include "../geometry/Kinematics.h"        
#include <iostream>                        
#include <string>                          
#include <cmath>                           


StewartPlatform::StewartPlatform(   const PlatformGeometry& geometry_,
                                    real_t body_length,
                                    real_t max_stroke,
                                    real_t max_force,
                                    real_t wn,    // rad/s — 10Hz bandwidth
                                    real_t zeta,   // critically damped
                                    real_t dt,   // 10kHz control rate
                                    MotorParameters motor,
                                    LeadScrewParameters screw,
                                    real_t V_supply)
                                    : geometry(geometry_)
                                    , min_heave(0.0)
                                    , max_heave(0.0)
                                    , mid_heave(0.0)
{
    validate_geometry(body_length, max_stroke); // set min, max, and mid heave and throw error if not working

    for (int i = 0; i < 6; i++)
        actuators[i] = std::make_unique<ActuatorState>(body_length, max_stroke, max_force,
            wn, zeta, dt, motor, screw, V_supply);
}

// Private

void StewartPlatform::validate_geometry(real_t body_length_, real_t max_stroke_)
{
    // This function checks the feasibility of the geometry: If the actuators are very short, 
    // They cannot even attach the floor to the platform
    Vec3 v_base = geometry.jointCoordinates_Base.col(0).transpose();
    Vec3 v_platform = geometry.jointCoordinates_Platform.col(0).transpose();

    real_t min_body_length = (v_base - v_platform).norm();

    if (body_length_ <= min_body_length) // Checking triangle inequalityc
    {
        throw std::invalid_argument("Actuator body length too short for given geometry. "
            "Minimum required body length is: " +
            std::to_string(1.2 * min_body_length) + " m"); // The body length must be at least 20% more 
    }

    min_heave = std::sqrt(body_length_ * body_length_ - min_body_length * min_body_length);
    max_heave = std::sqrt((body_length_ + max_stroke_) * (body_length_ + max_stroke_) - min_body_length * min_body_length);
    mid_heave = 0.5 * (min_heave + max_heave);

}


void StewartPlatform::set_actuator_target(int index, real_t total_length)
{
    if (index < 0 || index >= 6)
        throw std::out_of_range("actuator index out of bounds");
    actuators[index]->set_target_length(total_length);
}

void StewartPlatform::set_pose(const Pose6DoF& pose_)
{
    Pose6DoF inputPose = pose_;
    inputPose.z = inputPose.z + mid_heave;

    Kinematics::compute_InverseKinematics(
        geometry,
        inputPose,
        jointToJoint_lengths,
        actuators_unit_vector,
        jointCoordinates_Platform_World);
 

    for (int i = 0; i < 6; i++)
    {
        try { set_actuator_target(i, jointToJoint_lengths[i]); }
        catch (std::out_of_range&)
        {
            real_t bl = actuators[i]->get_body_length();
            real_t ms = actuators[i]->get_max_stroke();
            throw std::runtime_error(
                "Leg " + std::to_string(i) +
                " required length " + std::to_string(jointToJoint_lengths[i]) + " m"
                " outside actuator range [" +
                std::to_string(bl) + ", " +
                std::to_string(bl + ms) + "] m"
            );
        }

    }

    current_pose = pose_;

}

void StewartPlatform::update(real_t dt_step)
{
    for (int i = 0; i < 6; i++)
        actuators[i]->update(0.0);   // /in this version we assume F_load = 0 for now
}

void StewartPlatform::update_task_space(const Vec6& leg_forces, double dt)
{
    for (int i = 0; i < 6; i++)
        actuators[i]->force_control_update(leg_forces(i));
}
// All Getters

const ActuatorState& StewartPlatform::get_actuator(int index) const
{
    if (index < 0 || index >= 6)
    {
        throw std::out_of_range("The input actuator index is out of bound! must be between 0 to 5 ");
    }

    return *actuators[index];
}


ActuatorState& StewartPlatform::get_actuator_mutable(int index)
{
    if (index < 0 || index >= 6)
        throw std::out_of_range("actuator index out of bounds");
    return *actuators[index];
}

Pose6DoF StewartPlatform::get_pose() const noexcept
{
    return current_pose;
}

real_t StewartPlatform::get_min_heave() const noexcept { return min_heave; }
real_t StewartPlatform::get_max_heave() const noexcept { return max_heave; }
real_t StewartPlatform::get_mid_heave() const noexcept { return mid_heave; }

void StewartPlatform::print_all() const
{
    for (int i = 0; i < 6; i++)
    {
        std::cout << "Actuator " << i << " ";
        actuators[i]->print();


    }
    std::cout << "\n";
}