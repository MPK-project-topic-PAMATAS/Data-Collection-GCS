#pragma once

#include <memory>
#include <string>
#include <vector>
#include <cstdint>
#include "acquisition/SensorWorker.hpp"
#include "acquisition/RingBuffer.hpp"
#include "acquisition/Timestamp.hpp"
#include "acquisition/Diagnostics.hpp"

namespace msf {

struct ZedFrame {
    uint64_t timestamp_ns = 0;
    uint64_t device_timestamp_ns = 0;
    int frame_id = 0;

    std::vector<uint8_t> left_rgb;   // H*W*3 uint8
    std::vector<uint8_t> right_rgb;  // H*W*3 uint8
    std::vector<float> depth;        // H*W float32
    std::vector<float> confidence;   // H*W float32 (optional)
    std::vector<float> disparity;    // H*W float32 (optional)
    int width = 0;
    int height = 0;

    // ZED IMU
    float zed_imu_accel[3] = {};
    float zed_imu_gyro[3] = {};
    float zed_imu_quat[4] = {0, 0, 0, 1};
    float zed_imu_temperature = 0.0f;
    uint64_t zed_imu_timestamp_ns = 0;

    float exposure = 0.0f;
    float gain = 0.0f;
    float temperature = 0.0f;
};

class ZEDWorker : public SensorWorker {
public:
    struct Config {
        int resolution = 2;       // HD720
        int fps = 30;
        int depth_mode = 1;       // NEURAL_LIGHT
        int unit = 1;             // METER
        int coord_system = 1;     // RIGHT_HANDED_Z_UP
        float depth_min = 0.3f;
        float depth_max = 30.0f;
        int depth_stabilization = 1;
        int confidence_thresh = 100;
        int texture_conf_thresh = 100;
        bool flip_180 = false;
        bool save_confidence = false;
        bool save_disparity = false;
    };

    explicit ZEDWorker(const Config& cfg, size_t buffer_size = 30);
    ~ZEDWorker() override;

    bool start() override;
    SensorDiagnostics diagnostics() const override;

    std::optional<ZedFrame> get_latest();
    std::optional<ZedFrame> get_nearest(uint64_t target_ts, uint64_t max_delta_ns = 0);

    bool is_opened() const;
    void start_recording(const std::string& svo_path);
    void stop_recording();

    int width() const { return _width; }
    int height() const { return _height; }

private:
    void _acquisition_loop();
    ZedFrame _grab_frame();

    Config _cfg;
    RingBuffer<ZedFrame> _buffer;

    // Opaque pointers to avoid ZED SDK headers in this header
    struct ZedState;
    std::unique_ptr<ZedState> _state;

    int _width = 0;
    int _height = 0;
    int _frame_id = 0;
    uint64_t _start_ns = 0;
    uint64_t _frame_count = 0;
    uint64_t _dropped_count = 0;

    bool _recording = false;
    std::string _svo_path;
};

} // namespace msf
