#pragma once

#include <thread>
#include <chrono>
#include <iostream>
#include <atomic>
#include <iomanip>
#include <limits>
#include <array>
#include "../math/Math.h"


class LoopTimer
{
private:
    using Clock = std::chrono::steady_clock;
    using Duration = std::chrono::microseconds;
    using TimePoint = Clock::time_point;

    Duration period;
    TimePoint loop_start_time;
    TimePoint loop_last_iteration;
    TimePoint loop_next_time;

    real_t dt_physics{ 0.0 };
    int iteration{ 0 };
    int missed_deadlines{ 0 };
    real_t jitter{ 0.0 };
    real_t min_jitter{std::numeric_limits<real_t>::max()};
    real_t max_jitter{ 0.0 };
    real_t avg_jitter{ 0.0 };

public:

    explicit LoopTimer(Duration period_):period(period_){}

    void start()
    {
        loop_start_time     = Clock::now();
        loop_last_iteration = loop_start_time;
        loop_next_time      = loop_start_time;
    }

    real_t wait_until()
    {
       
        loop_next_time += period;
        
      

        if (Clock::now() > loop_next_time)
        {
            missed_deadlines++;
        }
        
        std::this_thread::sleep_until(loop_next_time);

        TimePoint actual_wakeup = Clock::now();

        jitter = std::chrono::duration_cast<std::chrono::microseconds>(actual_wakeup - loop_next_time).count();

        std::chrono::duration<double> dt_wake_up = (actual_wakeup - loop_last_iteration);
        dt_physics = dt_wake_up.count();


        min_jitter = std::min(min_jitter, jitter);
        max_jitter = std::max(max_jitter, jitter);
        avg_jitter = (avg_jitter * iteration + jitter) / (iteration + 1);

        loop_last_iteration = actual_wakeup;
        iteration += 1;
        
        return jitter;
    }

    // Getters
    real_t get_min_jitter()             const noexcept { return min_jitter; }
    real_t get_max_jitter()             const noexcept { return max_jitter; }
    real_t get_average_jitter()         const noexcept { return avg_jitter; }
    int get_missed_deadlines()          const noexcept { return missed_deadlines; }
    real_t get_dt_seconds()             const noexcept { return dt_physics; }
    void print_stats() const
    {
        std::cout << "\n--- LoopTimer Stats ---\n"
            << "Missed deadlines: " << missed_deadlines << "\n"
            << "Min jitter: " << min_jitter << " us\n"
            << "Max jitter: " << max_jitter << " us\n"
            << "Avg jitter: " << avg_jitter << " us\n";
    }
};
