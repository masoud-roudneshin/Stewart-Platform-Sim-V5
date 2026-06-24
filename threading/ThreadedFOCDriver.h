#pragma once
#include <mutex>
#include <atomic>
#include "../utils/LoopTimer.h"
#include <thread>
#include "../threading/SharedData.h"
#include "../actuator/MotorDriver.h"
#include "../threading/ThreadedSafetyMonitor.h"
#include "../math/Math.h"

class ThreadedFOCDriver
{
	FOCMotorDriver driver_;
	LoopTimer timer_;
	ActuatorSharedData& shared_;
	ThreadedSafetyMonitor& safety_;
	std::thread thread_;
	std::atomic<bool> running_{ false };

public:

	ThreadedFOCDriver(ActuatorSharedData& shared_data,
		ThreadedSafetyMonitor& safety,
		MotorParameters motor = MotorParameters(),
		LeadScrewParameters screw = LeadScrewParameters(),
		real_t max_stroke = 0.6,
		real_t dt = 0.0001,
		real_t V_supply = 150);

	ThreadedFOCDriver(const ThreadedFOCDriver&)					= delete;
	ThreadedFOCDriver& operator = (const ThreadedFOCDriver&)	= delete;
	ThreadedFOCDriver(ThreadedFOCDriver&&)						= delete;
	ThreadedFOCDriver& operator = (ThreadedFOCDriver&&)			= delete;

	void start();
	void stop();

private:
	void run();

};