#pragma once

#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include <mutex>
#include "acquisition/SensorWorker.hpp"
#include "acquisition/RingBuffer.hpp"
#include "acquisition/Diagnostics.hpp"

namespace msf {

struct RadarPoint {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float velocity_mps = 0.0f;
};

struct RadarFrame {
    uint64_t timestamp_ns = 0;
    uint64_t device_timestamp_ns = 0;
    uint32_t frame_id = 0;
    uint32_t num_objects = 0;

    std::vector<RadarPoint> points;
    std::vector<uint16_t> range_doppler; // num_range * num_doppler
    std::vector<uint16_t> range_profile; // num_range
    uint16_t num_range_bins = 0;
    uint16_t num_doppler_bins = 0;
};

class AWR1843Worker : public SensorWorker {
public:
    struct Config {
        std::string cfg_port = "/dev/ttyACM0";
        std::string data_port = "/dev/ttyACM1";
        int cfg_baud = 115200;
        int data_baud = 921600;
        uint16_t num_range_bins = 256;
        uint16_t num_doppler_bins = 16;
    };

    explicit AWR1843Worker(const Config& cfg, size_t buffer_size = 60);
    ~AWR1843Worker() override;

    bool start() override;
    SensorDiagnostics diagnostics() const override;

    std::optional<RadarFrame> get_latest();
    std::optional<RadarFrame> get_nearest(uint64_t target_ts, uint64_t max_delta_ns = 0);

    bool connected() const;

private:
    void _data_loop();
    bool _open_ports();
    bool _apply_config();
    bool _wait_prompt(int timeout_s);
    void _close_ports();
    void _reconnect_loop();
    void _extract_frames();

    Config _cfg;
    RingBuffer<RadarFrame> _buffer;

    int _cfg_fd = -1;
    int _data_fd = -1;
    mutable std::mutex _port_mutex;

    std::vector<uint8_t> _buf;

    uint32_t _device_frame_id = 0;
    uint64_t _start_ns = 0;
    uint64_t _frame_count = 0;
    uint64_t _error_count = 0;
};

} // namespace msf
