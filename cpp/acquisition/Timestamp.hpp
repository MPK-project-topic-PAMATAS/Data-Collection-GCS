#pragma once

#include <cstdint>
#include <chrono>
#include <ctime>

namespace msf {

struct Timestamp {
    uint64_t device_ns = 0;
    uint64_t host_monotonic_ns = 0;
    uint64_t host_wall_ns = 0;

    static Timestamp now() {
        Timestamp t;
        t.host_monotonic_ns = static_cast<uint64_t>(
            std::chrono::steady_clock::now().time_since_epoch().count());
        t.host_wall_ns = static_cast<uint64_t>(
            std::chrono::system_clock::now().time_since_epoch().count());
        return t;
    }

    double delta_ms(const Timestamp& other) const {
        return static_cast<double>(
            host_monotonic_ns > other.host_monotonic_ns
                ? host_monotonic_ns - other.host_monotonic_ns
                : other.host_monotonic_ns - host_monotonic_ns
        ) / 1e6;
    }
};

} // namespace msf
