#pragma once
#include "../math/Math.h"
// struct MotorParameters

struct MotorParameters
{
    real_t R;          // phase resistance (Ω)
    real_t L;          // phase inductance (H)
    real_t Kt;         // torque constant (Nm/A)
    real_t psi;         // BLDC flux linkage (V·s/rad) 
    real_t J;          // rotor inertia (kg·m²)
    int    pole_pairs; // number of pole pairs

    // Default: Maxon EC-i 40, 48V winding
    MotorParameters()
        : R(0.431)
        , L(0.000461)
        , Kt(0.0978)
        , psi(2.0 * 0.0978 / (3.0 * 3.0))   // = 0.00707 Wb (psi = Kt/(1.5 * p)
        , J(7.8e-6)
        , pole_pairs(3)
    {}
};

// ------------ Lead Screw Parameters -----------------------

struct LeadScrewParameters
{
    real_t lead;          // m/rev — linear travel per revolution
    real_t efficiency;    // mechanical efficiency
    real_t moving_mass;   // kg
    real_t viscous_friction; // N·s/m

    LeadScrewParameters()
        : lead(0.005)
        , efficiency(0.90)
        , moving_mass(2.0)
        , viscous_friction(2.0)
    {}
};

class FOCMotorDriver
{
private:
    MotorParameters    motor;
    LeadScrewParameters screw;

    // Electrical state
    real_t id;        // d-axis current (A)
    real_t iq;        // q-axis current (A)
    real_t theta_e;   // electrical angle (rad)
    real_t omega_m;   // mechanical angular velocity (rad/s)
    real_t integral_id;   // integrator state for d-axis
    real_t integral_iq;   // integrator state for q-axis

    // Mechanical state
    real_t stroke;    // current stroke (m)
    real_t velocity;  // linear velocity (m/s)
    real_t max_stroke;

    // Supply voltage
    real_t V_supply;

    // Subcycle count
    int N_subcycles;

    // Current Control Gains
    real_t dt_control;
    real_t Kp_current;   // PI current controller proportional gain
    real_t Ki_current;   // PI current controller integral gain
    real_t force_gain;
    real_t vel_to_omega;

public:
    // Constructor — takes parameters, supply voltage
    FOCMotorDriver( MotorParameters motor_,
                    LeadScrewParameters screw_,
                    real_t max_stroke_,
                    real_t dt_,
                    real_t V_supply_ = 150.0,
                    int    N_sub = 10);

    // Non-copyable, move-only
    FOCMotorDriver(const FOCMotorDriver&) = delete;
    FOCMotorDriver& operator=(const FOCMotorDriver&) = delete;
    FOCMotorDriver(FOCMotorDriver&&) noexcept = default;
    FOCMotorDriver& operator=(FOCMotorDriver&&) noexcept = default;

    // Main update — call once per mechanical timestep
    // iq_ref: q-axis current reference (from outer controller)
    // F_load: external load force on actuator (N)
    // dt:     mechanical timestep (s)
    // returns: actual applied force (N)
    real_t update(real_t iq_ref, real_t F_load, real_t id_ref = 0.0);


    // Getters
    real_t get_stroke()             const noexcept;
    real_t get_max_stroke()         const noexcept;
    real_t get_velocity()           const noexcept;
    real_t get_iq()                 const noexcept;
    real_t get_id()                 const noexcept;
    real_t get_omega_m()            const noexcept;
    real_t get_force()              const noexcept;
    real_t get_force_to_iq_gain()   const noexcept;

};
