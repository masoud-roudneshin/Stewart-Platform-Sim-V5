#include "../actuator/MotorDriver.h"
#include "../math/Math.h"
#include <cmath>


// Constructor — takes parameters, supply voltage
FOCMotorDriver::FOCMotorDriver( MotorParameters motor_,
                                LeadScrewParameters screw_,
                                real_t max_stroke_,
                                real_t dt_,
                                real_t V_supply_,
                                int    N_sub) :
                                motor(motor_)
                                , screw(screw_)
                                , max_stroke(max_stroke_)
                                , dt_control(dt_)
                                , V_supply(V_supply_)
                                , N_subcycles(N_sub)
                                , id(0.0)          // ← add all of these
                                , iq(0.0)
                                , theta_e(0.0)
                                , omega_m(0.0)
                                , stroke(0.0)
                                , velocity(0.0)
                                , integral_id(0.0)
                                , integral_iq(0.0)
                                
{
    real_t wc = 2.0 * PI * (1.0 / dt_control) / 20.0;
    Kp_current = motor.L * wc;
    Ki_current = motor.R * wc;

    force_gain = motor.Kt * (2.0 * PI / screw.lead) * screw.efficiency;
    vel_to_omega = 2.0 * PI / screw.lead;
}


// Main update — call once per mechanical timestep
// iq_ref: q-axis current reference (from outer controller)
// F_load: external load force on actuator (N)
// dt:     mechanical timestep (s)
// returns: actual applied force (N)
real_t FOCMotorDriver::update(real_t iq_ref, real_t F_load, real_t id_ref)
{
    real_t dt_sim = dt_control / N_subcycles;
    real_t force_sum = 0.0;


    // 1. Current controller runs ONCE per dt
    real_t error_d = id_ref - id;
    real_t error_q = iq_ref - iq;

    integral_id += error_d * dt_control;
    integral_iq += error_q * dt_control;

    real_t omega_e = omega_m * motor.pole_pairs;

    real_t Vd_cmd = Kp_current * error_d + Ki_current * integral_id
        - omega_e * motor.L * iq;

    real_t Vq_cmd = Kp_current * error_q + Ki_current * integral_iq
        + omega_e * (motor.L * id + motor.psi);

    real_t V_mag = std::sqrt(Vd_cmd * Vd_cmd + Vq_cmd * Vq_cmd);
    if (V_mag > V_supply)
    {
        Vd_cmd *= V_supply / V_mag;
        Vq_cmd *= V_supply / V_mag;
    }

    // 2. Plant simulation runs N_subcycles times with dt_sim
    for (int i = 0; i < N_subcycles; i++)
    {
        real_t omega_e_sim = omega_m * motor.pole_pairs;

        // Electrical dynamics
        real_t did_dt = (Vd_cmd - motor.R * id
            + omega_e_sim * motor.L * iq) / motor.L;

        real_t diq_dt = (Vq_cmd - motor.R * iq
            - omega_e_sim * (motor.L * id + motor.psi)) / motor.L;

        id += did_dt * dt_sim;
        iq += diq_dt * dt_sim;

        theta_e = std::fmod(theta_e + omega_e_sim * dt_sim, 2.0 * PI);

        // Torque to linear force
        real_t F_act = force_gain * iq;
        force_sum += F_act;

        // Mechanical dynamics also updated with dt_sim
        real_t F_net = F_act - F_load - screw.viscous_friction * velocity;
        real_t accel = F_net / screw.moving_mass;

        velocity += accel * dt_sim;
        stroke += velocity * dt_sim;

        if (stroke > max_stroke)
        {
            stroke = max_stroke;
            velocity = 0.0;        // hard stop
            
        }
        if (stroke < 0.0)
        {
            stroke = 0.0;
            velocity = 0.0;
        }

        omega_m = velocity * vel_to_omega;
    }

    real_t F_avg = force_sum / N_subcycles;
    return F_avg;
}

// Getters
real_t FOCMotorDriver::get_stroke()         const noexcept { return stroke; }
real_t FOCMotorDriver::get_max_stroke()     const noexcept { return max_stroke; }
real_t FOCMotorDriver::get_velocity()       const noexcept { return velocity; }
real_t FOCMotorDriver::get_iq()             const noexcept { return iq; }
real_t FOCMotorDriver::get_id()             const noexcept { return id; }
real_t FOCMotorDriver::get_omega_m()        const noexcept { return omega_m; }

real_t FOCMotorDriver::get_force()          const noexcept
{
    return force_gain * iq;
}

real_t FOCMotorDriver::get_force_to_iq_gain() const noexcept
{
    return 1.0 / force_gain;
}

