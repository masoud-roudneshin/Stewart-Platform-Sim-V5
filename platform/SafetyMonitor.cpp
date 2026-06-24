#include "../platform/SafetyMonitor.h"
#include <iostream>

SafetyMonitor::SafetyMonitor(StewartPlatform&& platform_,
    real_t max_velocity_,
    real_t max_force_)
    : platform(std::move(platform_))
    , state(PlatformState::READY)
    , max_velocity(max_velocity_)
    , max_force(max_force_)
    , fault_message("")
{}

void SafetyMonitor::check_actuator_limits() const
{
    for (int i = 0; i < 6; i++)
    {
        const ActuatorState& a = platform.get_actuator(i);

        if (std::abs(a.get_velocity()) > max_velocity)
            throw std::runtime_error(
                "Actuator " + std::to_string(i) +
                " velocity " + std::to_string(a.get_velocity()) +
                " m/s exceeds limit " + std::to_string(max_velocity) + " m/s");

        if (std::abs(a.get_force()) > max_force)
            throw std::runtime_error(
                "Actuator " + std::to_string(i) +
                " force " + std::to_string(a.get_force()) +
                " N exceeds limit " + std::to_string(max_force) + " N");
    }
}

void SafetyMonitor::set_pose(const Pose6DoF& pose)
{
    if (state != PlatformState::READY && state != PlatformState::RUNNING)
        throw std::runtime_error(
            "Cannot set pose — platform state: " + get_fault_message());

    platform.set_pose(pose);
    state = PlatformState::RUNNING;
}

void SafetyMonitor::update(real_t dt)
{
    if (state == PlatformState::ESTOP || state == PlatformState::FAULT)
        return;   // we silently do nothing — platform frozen

    platform.update(dt);

    try { check_actuator_limits(); }
    catch (const std::runtime_error& e)
    {
        state = PlatformState::FAULT;
        fault_message = e.what();
        std::cerr << "[FAULT] " << fault_message << "\n";
    }
}

void SafetyMonitor::estop()
{
    state = PlatformState::ESTOP;
    fault_message = "Emergency stop activated";
    std::cerr << "[ESTOP] Platform stopped\n";
    // platform.update() will no longer be called — actuators freeze at current state
}

void SafetyMonitor::reset()
{
    if (state == PlatformState::RUNNING)
        return;   // nothing to reset

    // Check all actuators are in safe state before allowing reset
    try { check_actuator_limits(); }
    catch (const std::runtime_error& e)
    {
        std::cerr << "[RESET DENIED] " << e.what() << "\n";
        return;
    }

    state = PlatformState::READY;
    fault_message = "";
    std::cerr << "[RESET] Platform ready\n";
}

// State observation
PlatformState SafetyMonitor::get_state()         const noexcept { return state; }
std::string   SafetyMonitor::get_fault_message() const noexcept { return fault_message; }
bool          SafetyMonitor::is_operational()    const noexcept
{
    return state == PlatformState::READY || state == PlatformState::RUNNING;
}

// Platform observation
const ActuatorState& SafetyMonitor::get_actuator(int index) const
{
    return platform.get_actuator(index);
}

Pose6DoF SafetyMonitor::get_pose()      const noexcept { return platform.get_pose(); }
real_t   SafetyMonitor::get_min_heave() const noexcept { return platform.get_min_heave(); }
real_t   SafetyMonitor::get_max_heave() const noexcept { return platform.get_max_heave(); }
real_t   SafetyMonitor::get_mid_heave() const noexcept { return platform.get_mid_heave(); }

void SafetyMonitor::print_all() const { platform.print_all(); }

void SafetyMonitor::print_state() const
{
    std::string state_str;
    switch (state)
    {
    case PlatformState::READY:   state_str = "READY";   break;
    case PlatformState::RUNNING: state_str = "RUNNING"; break;
    case PlatformState::ESTOP:   state_str = "ESTOP";   break;
    case PlatformState::FAULT:   state_str = "FAULT";   break;
    }
    std::cout << "Platform state: " << state_str;
    if (!fault_message.empty())
        std::cout << " — " << fault_message;
    std::cout << "\n";
}