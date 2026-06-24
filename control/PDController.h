#pragma once
#include <algorithm>
#include "../math/Math.h"

class PDController
{
private:
    real_t Kp;
    real_t Kd;
    real_t Ki;
    real_t integral_action{ 0.0 };
    real_t dt;

public:
    PDController() : Kp(0.0), Kd(0.0), Ki(0.0), dt(0.0), integral_action(0.0){}
    PDController(real_t M, real_t b, real_t wn, real_t zeta, real_t dt_)
        : Kp(wn* wn* M)
        , Kd(2.0 * zeta * wn * M - b)
        , Ki(0)
        , dt(dt_)
        , integral_action(0.0)
    {
        if (Kd < 0.0)
        {
            Kd = 0.0;
            // In production: log a warning here
        }
    }

    real_t compute(real_t target_stroke, real_t actual_stroke,
        real_t actual_velocity, real_t F_gravity = 0.0)  noexcept
    {
        real_t error = target_stroke - actual_stroke;
        integral_action += error * dt;
        real_t F_cmd = Kp * error - Kd * actual_velocity + Ki * integral_action + F_gravity;
        return F_cmd;
    }
};