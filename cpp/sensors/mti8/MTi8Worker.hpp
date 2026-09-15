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

struct MTiFrame {
    uint64_t timestamp_ns = 0;
    uint64_t device_timestamp_ns = 0;

    uint16_t packet_counter = 0;
    uint32_t sample_time_fine = 0;
    uint32_t sample_time_coarse = 0;

    float quaternion[4] = {0, 0, 0, 1};
    float euler[3] = {};
    float rotation_matrix[9] = {};

    float acceleration[3] = {};
    float gyroscope[3] = {};
    float magnetometer[3] = {};

    float delta_velocity[3] = {};
    float delta_quaternion[4] = {};

    double latitude = 0.0;
    double longitude = 0.0;
    double altitude_msl = 0.0;
    double altitude_ellipsoid = 0.0;

    float velocity[3] = {};

    float temperature = 0.0f;
    float baro_pressure = 0.0f;

    uint32_t status_word = 0;
    uint8_t status_byte = 0;

    bool has_quaternion = false;
    bool has_acceleration = false;
    bool has_gyroscope = false;
    bool has_magnetometer = false;
};

class MTi8Worker : public SensorWorker {
public:
    struct Config {
        std::string port = "/dev/ttyUSB0";
        int baudrate = 2000000;
        double timeout_s = 0.01;
    };

    explicit MTi8Worker(const Config& cfg, size_t buffer_size = 300);
    ~MTi8Worker() override;

    bool start() override;
    SensorDiagnostics diagnostics() const override;

    std::optional<MTiFrame> get_latest();
    std::optional<MTiFrame> get_nearest(uint64_t target_ts, uint64_t max_delta_ns = 0);

    void start_raw_log(const std::string& path);
    void stop_raw_log();
    std::string firmware() const { return _firmware; }

private:
    void _acquisition_loop();
    bool _open_port();
    void _send_command(const uint8_t* cmd, size_t len);

    Config _cfg;
    RingBuffer<MTiFrame> _buffer;

    struct SerialState;
    std::unique_ptr<SerialState> _serial_state;

    // Xbus parser state
    std::vector<uint8_t> _buf;

    std::string _firmware;
    uint64_t _start_ns = 0;
    uint64_t _frame_count = 0;
    uint64_t _error_count = 0;

    // Raw log
    std::mutex _raw_mutex;
    FILE* _raw_file = nullptr;
};

} // namespace msf
