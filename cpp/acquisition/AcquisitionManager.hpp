#pragma once

#include <memory>
#include <string>
#include <map>
#include <functional>
#include <mutex>

#include "sensors/zed/ZEDWorker.hpp"
#include "sensors/mti8/MTi8Worker.hpp"
#include "sensors/radar/AWR1843Worker.hpp"
#include "sensors/ouster/OusterWorker.hpp"
#include "sensors/gnss/GNSSWorker.hpp"

namespace msf {

struct AcquisitionConfig {
    ZEDWorker::Config zed;
    MTi8Worker::Config mti8;
    AWR1843Worker::Config radar;
    OusterWorker::Config ouster;
    GNSSWorker::Config gnss;

    bool enable_zed = false;
    bool enable_mti8 = false;
    bool enable_radar = false;
    bool enable_ouster = false;
    bool enable_gnss = false;

    size_t zed_buffer_size = 30;
    size_t mti8_buffer_size = 300;
    size_t radar_buffer_size = 60;
    size_t ouster_buffer_size = 200;
    size_t gnss_buffer_size = 10;
};

class AcquisitionManager {
public:
    explicit AcquisitionManager(const AcquisitionConfig& cfg);
    ~AcquisitionManager();

    // Lifecycle
    bool start();
    void stop();

    // Per-sensor access
    std::optional<ZedFrame> zed_latest();
    std::optional<MTiFrame> mti8_latest();
    std::optional<RadarFrame> radar_latest();
    std::optional<OusterFrame> ouster_latest();
    std::optional<GNSSFix> gnss_latest();

    std::optional<MTiFrame> mti8_nearest(uint64_t ts, uint64_t max_delta);
    std::optional<RadarFrame> radar_nearest(uint64_t ts, uint64_t max_delta);
    std::optional<OusterFrame> ouster_nearest(uint64_t ts, uint64_t max_delta);

    // Diagnostics
    std::map<std::string, SensorDiagnostics> all_diagnostics() const;

    bool is_running(const std::string& sensor_id) const;
    bool zed_opened() const;
    int zed_width() const { return _zed ? _zed->width() : 0; }
    int zed_height() const { return _zed ? _zed->height() : 0; }
    bool radar_connected() const;
    bool ouster_streaming() const;

    // Recording control
    void zed_start_recording(const std::string& path);
    void zed_stop_recording();
    void mti8_start_raw_log(const std::string& path);
    void mti8_stop_raw_log();
    void ouster_start_pcap(const std::string& path);
    void ouster_stop_pcap();

private:
    AcquisitionConfig _cfg;
    std::unique_ptr<ZEDWorker> _zed;
    std::unique_ptr<MTi8Worker> _mti8;
    std::unique_ptr<AWR1843Worker> _radar;
    std::unique_ptr<OusterWorker> _ouster;
    std::unique_ptr<GNSSWorker> _gnss;

    std::mutex _state_mutex;
    bool _started = false;
};

} // namespace msf
