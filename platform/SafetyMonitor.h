#pragma once
#include <stdexcept>
#include <string>
#include "../threading/SharedData.h"
#include "../platform/StewartPlatform.h"

//enum class PlatformState { READY, RUNNING, ESTOP, FAULT };

class SafetyMonitor
{
private:
    StewartPlatform platform;
    PlatformState   state;
    real_t          max_velocity;   // m/s — per actuator
    real_t          max_force;      // N — per actuator
    std::string     fault_message;  // stores last fault reason

    void check_actuator_limits() const;

public:
    SafetyMonitor(StewartPlatform&& platform_,
        real_t max_velocity_ = 2.0,
        real_t max_force_ = 500.0);

    // Non-copyable, non-movable — single owner, fixed location
    SafetyMonitor(const SafetyMonitor&) = delete;
    SafetyMonitor& operator=(const SafetyMonitor&) = delete;
    SafetyMonitor(SafetyMonitor&&) = delete;
    SafetyMonitor& operator=(SafetyMonitor&&) = delete;

    // Commands — safety checked
    void set_pose(const Pose6DoF& pose);
    void update(real_t dt);
    void estop();
    void reset();   // clears FAULT/ESTOP → READY if safe

    // State observation
    PlatformState   get_state()       const noexcept;
    std::string     get_fault_message() const noexcept;
    bool            is_operational()  const noexcept;

    // Platform observation — read only
    const ActuatorState& get_actuator(int index)  const;
    Pose6DoF             get_pose()               const noexcept;
    real_t               get_min_heave()          const noexcept;
    real_t               get_max_heave()          const noexcept;
    real_t               get_mid_heave()          const noexcept;
    void                 print_all()              const;
    void                 print_state()            const;
};