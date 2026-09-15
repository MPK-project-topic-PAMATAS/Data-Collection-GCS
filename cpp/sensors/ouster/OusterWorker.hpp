#pragma once

#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include <mutex>
#include <thread>
#include <atomic>
#include <cstdio>

#include "acquisition/SensorWorker.hpp"
#include "acquisition/RingBuffer.hpp"
#include "acquisition/Diagnostics.hpp"

// Ouster SDK forward decls (see OusterWorker.cpp for includes)
namespace ouster { namespace sdk { namespace sensor {
class Sensor;
class SensorPacketSource;
}}
namespace sdk { namespace core {
class FrameBatcher;
class LidarFrame;
template <typename T> class XYZLutT;
}}
namespace sdk { namespace core { struct SensorInfo; }}
}

namespace msf {

struct OusterFrame {
    uint64_t timestamp_ns = 0;
    uint64_t device_timestamp_ns = 0;
    int frame_id = 0;
    int num_points = 0;
    bool complete = false;

    // Point cloud: interleaved [x0,y0,z0, x1,y1,z1, ...] float32
    std::vector<float> points_xyz;
    std::vector<float> reflectivity;
};

// Ouster OS-0 UDP client backed by the official Ouster C++ SDK.
// Uses SensorPacketSource to receive packets and FrameBatcher + XYZLut to
// decode them into 3D point clouds, replacing the previous hand-rolled parser.
class OusterWorker : public SensorWorker {
public:
    struct Config {
        std::string hostname = "169.254.197.185";
        uint16_t lidar_port = 7502;
        uint16_t imu_port = 7503;
        size_t buffer_size = 200;
        uint16_t columns_per_packet = 16;
        uint16_t pixels_per_column = 64;
    };

    explicit OusterWorker(const Config& cfg);
    ~OusterWorker() override;

    bool start() override;
    SensorDiagnostics diagnostics() const override;

    std::optional<OusterFrame> get_latest();
    std::optional<OusterFrame> get_nearest(uint64_t target_ts, uint64_t max_delta_ns = 0);

    bool streaming() const;
    std::string sensor_name() const { return _sensor_name; }

    // pcap recording
    void start_pcap(const std::string& path);
    void stop_pcap();

private:
    void _loop();
    void _write_pcap_record(const uint8_t* data, size_t len);
    void _reset_sdk();

    Config _cfg;
    RingBuffer<OusterFrame> _buffer;

    // Ouster SDK objects (owned here)
    std::unique_ptr<ouster::sdk::sensor::Sensor> _sensor;
    std::unique_ptr<ouster::sdk::sensor::SensorPacketSource> _client;
    std::unique_ptr<ouster::sdk::core::FrameBatcher> _batcher;
    std::unique_ptr<ouster::sdk::core::LidarFrame> _frame;
    std::unique_ptr<ouster::sdk::core::XYZLutT<double>> _xyz_lut;
    std::shared_ptr<ouster::sdk::core::SensorInfo> _info;

    uint64_t _start_ns = 0;
    uint64_t _frame_count = 0;
    uint64_t _packet_count = 0;
    uint64_t _error_count = 0;
    uint64_t _last_frame_ns = 0;

    std::string _sensor_name;
    std::string _last_error;

    std::mutex _pcap_mutex;
    FILE* _pcap_file = nullptr;
    uint64_t _pcap_packets = 0;
};

} // namespace msf
