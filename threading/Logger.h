#pragma once
#include <thread>
#include <chrono>
#include <atomic>
#include <array>
#include "../threading/SharedData.h"
#include "../threading/ThreadedSafetyMonitor.h"
#include "../math/Math.h"

class Logger
{
	std::mutex print_mtx_;
	ThreadedSafetyMonitor& safety_;
	std::array<ActuatorSharedData*, 6> shared_;
	std::thread thread_;
	std::atomic<bool> running_{ false };

public:

	Logger(std::array<ActuatorSharedData*, 6> shared, ThreadedSafetyMonitor& safety);

	Logger(const Logger&) = delete;
	Logger& operator = (const Logger&) = delete;
	Logger( Logger&&) = delete;
	Logger& operator = ( Logger&&) = delete;

	void start();
	void stop();

private: 
	void run();
};