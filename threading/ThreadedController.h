#pragma once
#include <thread>
#include <chrono>
#include <atomic>
#include <mutex>
#include <array>
#include "../control/PDController.h"
#include "../utils/LoopTimer.h"
#include "../threading/SharedData.h"
#include "../platform/SafetyMonitor.h"
#include "../threading/ThreadedSafetyMonitor.h"
#include "../math/Math.h"

class ThreadedController
{
	std::mutex targets_mtx_;
	std::array<PDController, 6>				pd_controller_;

	Vec6									target_strokes_ = Vec6::Zero();

	LoopTimer								timer_{ std::chrono::microseconds(1000) };
	std::array<ActuatorSharedData*, 6>		shared_;
	ThreadedSafetyMonitor&					safety_;
	std::thread								thread_;
	std::atomic<bool>						running_{ false };
	real_t									force_to_iq_gain_{ 0.0 };

public:

	ThreadedController(std::array<ActuatorSharedData*, 6>		shared,
		ThreadedSafetyMonitor& safety,
		real_t wn,
		real_t zeta,
		real_t dt,
		real_t force_to_iq_gain,
		real_t moving_mass = 2.0,
		real_t viscous_friction = 50.0);
	
	ThreadedController(const ThreadedController&)				= delete;
	ThreadedController& operator = (const ThreadedController&)	= delete;
	ThreadedController(ThreadedController&&)					= delete;
	ThreadedController& operator = (ThreadedController&&)		= delete;

	void start();
	void stop();

	void set_target(const std::array<real_t,6>& target_strokes);

	void set_target(const Vec6& target_strokes);


private:
	void run();
};
