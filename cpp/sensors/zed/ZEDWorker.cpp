#include "ZEDWorker.hpp"

#include <chrono>
#include <cstring>
#include <sl/Camera.hpp>

namespace msf {

struct ZEDWorker::ZedState {
    sl::Camera camera;
    sl::InitParameters init_params;
    sl::RuntimeParameters runtime_params;
    sl::Mat left_mat, right_mat, depth_mat, conf_mat, disp_mat;
    sl::Mat view_depth_mat, view_conf_mat;
    sl::SensorsData sensors_data;
};

ZEDWorker::ZEDWorker(const Config& cfg, size_t buffer_size)
    : _cfg(cfg), _buffer(buffer_size), _state(std::make_unique<ZedState>()) {
    _sensor_id = "zed";
}

ZEDWorker::~ZEDWorker() {
    stop();
    if (_state->camera.isOpened()) {
        _state->camera.close();
    }
}

bool ZEDWorker::start() {
    auto& ip = _state->init_params;
    ip.camera_resolution = static_cast<sl::RESOLUTION>(_cfg.resolution);
    ip.camera_fps = _cfg.fps;
    ip.depth_mode = static_cast<sl::DEPTH_MODE>(_cfg.depth_mode);
    ip.coordinate_units = static_cast<sl::UNIT>(_cfg.unit);
    ip.coordinate_system = static_cast<sl::COORDINATE_SYSTEM>(_cfg.coord_system);
    ip.depth_minimum_distance = _cfg.depth_min;
    ip.depth_maximum_distance = _cfg.depth_max;
    ip.depth_stabilization = _cfg.depth_stabilization;

    auto& rp = _state->runtime_params;
    rp.confidence_threshold = _cfg.confidence_thresh;
    rp.texture_confidence_threshold = _cfg.texture_conf_thresh;

    auto err = _state->camera.open(_state->init_params);
    if (err != sl::ERROR_CODE::SUCCESS) return false;

    auto info = _state->camera.getCameraInformation();
    auto left = info.camera_configuration.calibration_parameters.left_cam;
    _width = info.camera_configuration.resolution.width;
    _height = info.camera_configuration.resolution.height;

    _start_ns = Timestamp::now().host_monotonic_ns;
    start_thread([this]() { _acquisition_loop(); });
    return true;
}

void ZEDWorker::_acquisition_loop() {
    while (_running) {
        if (_state->camera.grab(_state->runtime_params) != sl::ERROR_CODE::SUCCESS) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }

        auto frame = _grab_frame();
        _buffer.push(std::move(frame));
        _frame_count++;
    }
}

ZedFrame ZEDWorker::_grab_frame() {
    auto now = Timestamp::now();
    ZedFrame f;
    f.timestamp_ns = now.host_monotonic_ns;
    f.frame_id = _frame_id++;
    f.width = _width;
    f.height = _height;

    _state->camera.retrieveImage(_state->left_mat, sl::VIEW::LEFT, sl::MEM::CPU);
    _state->camera.retrieveImage(_state->right_mat, sl::VIEW::RIGHT, sl::MEM::CPU);
    _state->camera.retrieveMeasure(_state->depth_mat, sl::MEASURE::DEPTH, sl::MEM::CPU);

    if (_cfg.save_confidence) {
        _state->camera.retrieveMeasure(_state->conf_mat, sl::MEASURE::CONFIDENCE, sl::MEM::CPU);
    }
    if (_cfg.save_disparity) {
        _state->camera.retrieveMeasure(_state->disp_mat, sl::MEASURE::DISPARITY, sl::MEM::CPU);
    }

    // Left RGB (U8_C4 -> RGB888; manual row stride via getStepBytes)
    {
        auto& mat = _state->left_mat;
        f.left_rgb.resize(_width * _height * 3);
        auto base = mat.getPtr<sl::uchar1>(sl::MEM::CPU);
        size_t step = mat.getStepBytes(sl::MEM::CPU);
        for (int r = 0; r < _height; r++) {
            auto ptr = reinterpret_cast<const unsigned char*>(base) + r * step;
            for (int c = 0; c < _width; c++) {
                size_t idx = (r * _width + c) * 3;
                f.left_rgb[idx]     = ptr[c * 4];
                f.left_rgb[idx + 1] = ptr[c * 4 + 1];
                f.left_rgb[idx + 2] = ptr[c * 4 + 2];
            }
        }
    }

    // Right RGB
    {
        auto& mat = _state->right_mat;
        f.right_rgb.resize(_width * _height * 3);
        auto base = mat.getPtr<sl::uchar1>(sl::MEM::CPU);
        size_t step = mat.getStepBytes(sl::MEM::CPU);
        for (int r = 0; r < _height; r++) {
            auto ptr = reinterpret_cast<const unsigned char*>(base) + r * step;
            for (int c = 0; c < _width; c++) {
                size_t idx = (r * _width + c) * 3;
                f.right_rgb[idx]     = ptr[c * 4];
                f.right_rgb[idx + 1] = ptr[c * 4 + 1];
                f.right_rgb[idx + 2] = ptr[c * 4 + 2];
            }
        }
    }

    // Depth
    {
        f.depth.resize(_width * _height);
        auto ptr = _state->depth_mat.getPtr<float>(sl::MEM::CPU);
        size_t step = _state->depth_mat.getStepBytes(sl::MEM::CPU);
        if (step == _width * sizeof(float)) {
            std::memcpy(f.depth.data(), ptr, _width * _height * sizeof(float));
        } else {
            for (int r = 0; r < _height; r++) {
                std::memcpy(f.depth.data() + r * _width,
                            reinterpret_cast<const char*>(ptr) + r * step,
                            _width * sizeof(float));
            }
        }
    }

    // Confidence
    if (_cfg.save_confidence) {
        f.confidence.resize(_width * _height);
        auto ptr = _state->conf_mat.getPtr<float>(sl::MEM::CPU);
        std::memcpy(f.confidence.data(), ptr, _width * _height * sizeof(float));
    }

    // Disparity
    if (_cfg.save_disparity) {
        f.disparity.resize(_width * _height);
        auto ptr = _state->disp_mat.getPtr<float>(sl::MEM::CPU);
        std::memcpy(f.disparity.data(), ptr, _width * _height * sizeof(float));
    }

    // ZED IMU
    if (_state->camera.getSensorsData(_state->sensors_data, sl::TIME_REFERENCE::CURRENT)
        == sl::ERROR_CODE::SUCCESS) {
        auto& imu = _state->sensors_data.imu;
        auto& mag = _state->sensors_data.magnetometer;
        auto& temp = _state->sensors_data.temperature;

        f.zed_imu_gyro[0] = imu.angular_velocity.x;
        f.zed_imu_gyro[1] = imu.angular_velocity.y;
        f.zed_imu_gyro[2] = imu.angular_velocity.z;
        f.zed_imu_accel[0] = imu.linear_acceleration.x;
        f.zed_imu_accel[1] = imu.linear_acceleration.y;
        f.zed_imu_accel[2] = imu.linear_acceleration.z;

        auto orient = imu.pose.getOrientation();
        f.zed_imu_quat[0] = orient.x;
        f.zed_imu_quat[1] = orient.y;
        f.zed_imu_quat[2] = orient.z;
        f.zed_imu_quat[3] = orient.w;
        f.zed_imu_timestamp_ns = imu.timestamp.getNanoseconds();

        float t = 0.0f;
        if (temp.get(sl::SensorsData::TemperatureData::SENSOR_LOCATION::ONBOARD_LEFT, t)
            == sl::ERROR_CODE::SUCCESS) {
            f.temperature = t;
        }
    }

    // Camera metrics - getExposure/getGain may not exist; use defaults
    // ponytail: ZED SDK versions vary; keep exposure/gain at 0 when unavailable
    f.exposure = 0.0f;
    f.gain = 0.0f;

    return f;
}

std::optional<ZedFrame> ZEDWorker::get_latest() { return _buffer.latest(); }
std::optional<ZedFrame> ZEDWorker::get_nearest(uint64_t ts, uint64_t max_delta) {
    return _buffer.nearest(ts, max_delta);
}

bool ZEDWorker::is_opened() const { return _state->camera.isOpened(); }

void ZEDWorker::start_recording(const std::string& svo_path) {
    if (_recording) return;
    sl::RecordingParameters rp;
    rp.video_filename = svo_path.c_str();
    rp.compression_mode = sl::SVO_COMPRESSION_MODE::H264_LOSSLESS;
    _state->camera.enableRecording(rp);
    _recording = true;
    _svo_path = svo_path;
}

void ZEDWorker::stop_recording() {
    if (!_recording) return;
    _state->camera.disableRecording();
    _recording = false;
}

SensorDiagnostics ZEDWorker::diagnostics() const {
    SensorDiagnostics d;
    d.sensor_id = _sensor_id;
    d.configured_rate = static_cast<double>(_cfg.fps);
    auto elapsed_ns = Timestamp::now().host_monotonic_ns - _start_ns;
    d.measured_rate = (elapsed_ns > 0)
        ? static_cast<double>(_frame_count) / (static_cast<double>(elapsed_ns) / 1e9)
        : 0.0;
    d.received_count = _frame_count;
    d.dropped_count = _dropped_count;
    d.buffer_size = _buffer.count();
    d.buffer_capacity = _buffer.capacity();
    d.device_status = _state->camera.isOpened() ? "connected" : "disconnected";
    d.connection_status = d.device_status;
    return d;
}

} // namespace msf
