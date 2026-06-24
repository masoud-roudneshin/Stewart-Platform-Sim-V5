#pragma once
#include <stdexcept>
#include "../math/Math.h"
#include "../actuator/MotorDriver.h"
#include "../control/PDController.h"



class ActuatorState
{
private:
    FOCMotorDriver driver;     // owns true physical state
    real_t body_length;        // fixed geometry (m)
    real_t max_force;          // safety force limit (N)
    real_t target_stroke;      // where controller wants to go (m)
    real_t iq_ref;             // current reference for FOC driver
    PDController pd;          // position controller
    real_t F_gravity;         // gravity feedforward (N), default 0

public:
    // Constructor
    ActuatorState(  real_t body_length_,
                    real_t max_stroke_,
                    real_t max_force_,
                    real_t wn_,
                    real_t zeta_,
                    real_t dt_,
                    MotorParameters motor_ = MotorParameters(),
                    LeadScrewParameters screw_ = LeadScrewParameters(),
                    real_t V_supply = 48.0);

    // Rule of 5 — we delete copy (because FOCMotorDriver is non-copyable)
    
    ActuatorState(const ActuatorState&) = delete;
    ActuatorState& operator=(const ActuatorState&) = delete;
    ActuatorState(ActuatorState&&) noexcept = default;
    ActuatorState& operator=(ActuatorState&&) noexcept = default;

    // Controller interface
    void set_target_length(real_t total_length);

    void apply_force(real_t f);

    void set_gravity_feedforward(real_t F);

    // Physics update — call once per control timestep
    real_t update(real_t F_load);

    double force_control_update(double F_cmd, double F_load = 0.0);

    // Getters 
    real_t get_stroke()        const noexcept;
    real_t get_body_length()   const noexcept;
    real_t get_max_stroke()    const noexcept;
    real_t get_total_length()  const noexcept;
    real_t get_velocity()      const noexcept;
    real_t get_force()         const noexcept;
    real_t get_target_stroke() const noexcept;
    real_t get_target_length() const noexcept;
    real_t get_iq()            const noexcept;

    void print() const;
};