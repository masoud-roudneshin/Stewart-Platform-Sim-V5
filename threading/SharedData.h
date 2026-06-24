#pragma once
#include <chrono>
#include <mutex>
#include <atomic>
#include "../utils/RingBuffer.h"
#include "../math/Math.h"


enum class ControlMode { JOINT_SPACE, TASK_SPACE };

enum class PlatformState { READY, RUNNING, ESTOP, FAULT };

struct FOCState
{
	real_t stroke{ 0.0 }, velocity{ 0.0 }, iq{ 0.0 }, id{ 0.0 }, force{ 0.0 };
	int iteration;
	std::chrono::steady_clock::time_point timestamp;
};


struct ActuatorSharedData
{
	// FOC to control
	FOCState latest_state;
	std::mutex mtx_foc;

	// Control to FOC

	std::atomic<real_t> iq_ref;

	// Write to Logger

	RingBuffer<FOCState, 128> log_buffer;


};
