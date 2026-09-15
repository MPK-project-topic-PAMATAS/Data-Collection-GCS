#pragma once

#include <vector>
#include <mutex>
#include <optional>
#include <algorithm>
#include <cstdint>

namespace msf {

template <typename T>
class RingBuffer {
public:
    explicit RingBuffer(size_t capacity)
        : _buffer(capacity), _capacity(capacity), _head(0), _count(0) {}

    void push(T item) {
        std::lock_guard<std::mutex> lock(_mutex);
        _buffer[_head] = std::move(item);
        _head = (_head + 1) % _capacity;
        if (_count < _capacity) _count++;
    }

    std::optional<T> latest() const {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_count == 0) return std::nullopt;
        size_t idx = (_head + _capacity - 1) % _capacity;
        return _buffer[idx];
    }

    std::optional<T> nearest(uint64_t target_ts, uint64_t max_delta_ns = 0) const {
        std::lock_guard<std::mutex> lock(_mutex);
        if (_count == 0) return std::nullopt;

        const T* best = nullptr;
        uint64_t best_delta = UINT64_MAX;

        for (size_t i = 0; i < _count; i++) {
            size_t idx = (_head + _capacity - 1 - i) % _capacity;
            uint64_t buf_ts = get_timestamp(_buffer[idx]);
            uint64_t delta = (buf_ts > target_ts)
                ? buf_ts - target_ts
                : target_ts - buf_ts;

            if (delta >= best_delta) break; // timestamps are sorted, no improvement possible
            best = &_buffer[idx];
            best_delta = delta;
        }

        if (!best) return std::nullopt;
        if (max_delta_ns > 0 && best_delta > max_delta_ns) return std::nullopt;
        return *best;
    }

    size_t count() const {
        std::lock_guard<std::mutex> lock(_mutex);
        return _count;
    }

    size_t capacity() const { return _capacity; }

    void clear() {
        std::lock_guard<std::mutex> lock(_mutex);
        _count = 0;
        _head = 0;
    }

private:
    // All frame types in this system expose `timestamp_ns`.
    static uint64_t get_timestamp(const T& item) {
        return item.timestamp_ns;
    }

    mutable std::mutex _mutex;
    std::vector<T> _buffer;
    size_t _capacity;
    size_t _head;
    size_t _count;
};

} // namespace msf
