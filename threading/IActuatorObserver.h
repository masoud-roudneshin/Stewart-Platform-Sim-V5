#pragma once
#include "../threading/SharedData.h"

class IActuatorObserver
{
public:
    virtual void on_state_updated(int actuator_index, const FOCState& state) = 0;
    virtual ~IActuatorObserver() = default;
};