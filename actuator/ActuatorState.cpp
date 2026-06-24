#include "../actuator/ActuatorState.h"
#include <iostream>


ActuatorState::ActuatorState(   real_t body_length_,
                                real_t max_stroke_,
                                real_t max_force_,
                                real_t wn_,
                                real_t zeta_,
                                real_t dt_,
                                MotorParameters motor_,
                                LeadScrewParameters screw_,
                                real_t V_supply)
                                : driver(motor_, screw_, max_stroke_, dt_, V_supply)
                                , body_length(body_length_)
                                , max_force(max_force_)
                                , target_stroke(0.0)
                                , iq_ref(0.0)
                                , pd(screw_.moving_mass, screw_.viscous_friction, wn_, zeta_, dt_)
                                , F_gravity(0.0)
{}


void ActuatorState::set_target_length(real_t total_length)
{
    real_t s = total_length - body_length;
    if (s < 0.0 || s > driver.get_max_stroke())
        throw std::out_of_range("Target length outside actuator stroke range");
    target_stroke = s;
}

void ActuatorState::apply_force(real_t f)
{
    // clamp to max_force
    f = std::max(-max_force, std::min(max_force, f));
    // convert force → iq_ref

    iq_ref = f * driver.get_force_to_iq_gain();

}

void ActuatorState::set_gravity_feedforward(real_t F) { F_gravity = F; }

// Physics update — call once per control timestep
real_t ActuatorState::update(real_t F_load)
{
    // PD controller computes force command from position error
    real_t F_cmd = pd.compute(  target_stroke,
                                driver.get_stroke(),
                                driver.get_velocity(),
                                F_gravity);
    // Convert force → iq_ref
    apply_force(F_cmd);

    // FOC driver executes
    return driver.update(iq_ref, F_load);

}

real_t ActuatorState::force_control_update(double F_cmd, double F_load)
{
    apply_force(F_cmd);              // set iq_ref from force
    return driver.update(iq_ref, F_load);  // bypass PD
}

// Getters
real_t ActuatorState::get_stroke()        const noexcept { return driver.get_stroke(); }
real_t ActuatorState::get_body_length()   const noexcept { return body_length; }
real_t ActuatorState::get_max_stroke()    const noexcept { return driver.get_max_stroke(); }
real_t ActuatorState::get_total_length()  const noexcept { return body_length + driver.get_stroke(); }
real_t ActuatorState::get_velocity()      const noexcept { return driver.get_velocity(); }
real_t ActuatorState::get_force()         const noexcept { return driver.get_force(); }
real_t ActuatorState::get_target_stroke() const noexcept { return target_stroke; }
real_t ActuatorState::get_target_length() const noexcept { return body_length + target_stroke; }
real_t ActuatorState::get_iq()            const noexcept { return driver.get_iq(); }

void ActuatorState::print() const
{
    std::cout << "  Stroke=" << driver.get_stroke()
        << "  Velocity=" << driver.get_velocity()
        << "  Force=" << driver.get_force()
        << "  Target_Stroke=" << target_stroke
        << "  iq=" << driver.get_iq() << "\n";
}