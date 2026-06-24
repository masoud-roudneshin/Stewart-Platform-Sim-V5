#pragma once
#include <thread>
#include <atomic>
#include <array>

using real_t = double;
template <typename T, size_t N>

class RingBuffer

{
private:

    std::array<T, N> buffer_;
    std::atomic<size_t> write_index_{ 0 };
    std::atomic<size_t> read_index_{ 0 };

public:

    bool push(const T& value)
    {
        size_t write = write_index_.load(std::memory_order_relaxed);
        size_t next = (write + 1) % N;

        if (next == read_index_.load(std::memory_order_acquire))
        {
            return false;
        }

        buffer_[write] = value;

        write_index_.store(next, std::memory_order_release);



    }

    void push_overwrite(const T& value)
    {
        size_t write = write_index_.load(std::memory_order_relaxed);
        size_t next = (write + 1) % N;

        if (next == read_index_.load(std::memory_order_acquire))
        {
            read_index_.store((read_index_.load(std::memory_order_relaxed) + 1) % N,
                std::memory_order_release);
        }

        buffer_[write] = value;

        write_index_.store(next, std::memory_order_release);



    }

    bool pop(T& value)
    {
        size_t read = read_index_.load(std::memory_order_relaxed);

        if (read == write_index_.load(std::memory_order_acquire))
        {
            return false;
        }

        size_t next = (read + 1) % N;

        value = buffer_[read];

        read_index_.store(next, std::memory_order_release);
        return true;
    }

    bool empty()
    {
        return read_index_.load(std::memory_order_acquire) ==
            write_index_.load(std::memory_order_acquire);
        
    }

    size_t size()
    {
        size_t read  = read_index_.load(std::memory_order_acquire);
        size_t write = write_index_.load(std::memory_order_acquire);

        return (write - read + N) % N;

    }

};

