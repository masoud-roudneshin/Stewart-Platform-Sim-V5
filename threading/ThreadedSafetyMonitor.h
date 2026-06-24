#pragma once

#include <thread>
#include <atomic>
#include <array>
#include <mutex>
#include <chrono>
#include <string>
#include "../threading/SharedData.h"
#include "../utils/LoopTimer.h"
#include "../math/Math.h"


class ThreadedSafetyMonitor
{
	std::array <ActuatorSharedData*, 6>		shared_;
	std::atomic<PlatformState>				state_{ PlatformState::READY };
	real_t									max_velocity_;
	real_t									max_force_;
	real_t									max_stroke_;
	LoopTimer								timer_{std::chrono::microseconds(1000)};
	std::thread								thread_;
	std::atomic <bool>						running_{ false };
	std::atomic <bool>						estop_{ false };
	std::string								fault_message_;
	std::mutex								fault_mtx_;

public:

	ThreadedSafetyMonitor(	std::array <ActuatorSharedData*, 6>		shared,
							real_t max_velocity = 2.0,
							real_t max_force = 500.0,
							real_t max_stroke = 0.6);

	ThreadedSafetyMonitor(const ThreadedSafetyMonitor&)						= delete;
	ThreadedSafetyMonitor& operator = (const ThreadedSafetyMonitor&)		= delete;
	ThreadedSafetyMonitor(ThreadedSafetyMonitor&&)					= delete;
	ThreadedSafetyMonitor& operator = (ThreadedSafetyMonitor&&)		= delete;

	void start();
	void stop();

	void estop();
	void reset();

	bool is_operational() const noexcept;
	PlatformState get_state()const noexcept;
	std::string get_fault_message() const noexcept;
	bool is_estop() const noexcept;

private:
	void run();
	void check_limits();
};
