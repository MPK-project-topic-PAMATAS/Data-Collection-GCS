#pragma once

#include <string>
#include <cstdint>

namespace msf {

struct SensorDiagnostics {
    std::string sensor_id;
    double configured_rate = 0.0;
    double measured_rate = 0.0;
    uint64_t received_count = 0;
    uint64_t dropped_count = 0;
    uint64_t packet_count = 0;
    size_t buffer_size = 0;
    size_t buffer_capacity = 0;
    uint64_t last_timestamp_ns = 0;
    double latency_ms = 0.0;
    double timestamp_delta_ms = 0.0;
    std::string device_status = "disconnected";
    std::string connection_status = "disconnected";
    std::string last_error;
};

} // namespace msf
