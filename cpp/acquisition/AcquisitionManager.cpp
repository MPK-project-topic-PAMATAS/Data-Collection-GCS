#include "AcquisitionManager.hpp"

namespace msf {

AcquisitionManager::AcquisitionManager(const AcquisitionConfig& cfg)
    : _cfg(cfg) {
    if (_cfg.enable_zed)
        _zed = std::make_unique<ZEDWorker>(_cfg.zed, _cfg.zed_buffer_size);
    if (_cfg.enable_mti8)
        _mti8 = std::make_unique<MTi8Worker>(_cfg.mti8, _cfg.mti8_buffer_size);
    if (_cfg.enable_radar)
        _radar = std::make_unique<AWR1843Worker>(_cfg.radar, _cfg.radar_buffer_size);
    if (_cfg.enable_ouster)
        _ouster = std::make_unique<OusterWorker>(_cfg.ouster);
    if (_cfg.enable_gnss)
        _gnss = std::make_unique<GNSSWorker>(_cfg.gnss);
}

AcquisitionManager::~AcquisitionManager() {
    stop();
}

bool AcquisitionManager::start() {
    std::lock_guard<std::mutex> lock(_state_mutex);
    if (_started) return true;

    bool any_started = false;

    if (_zed && _zed->start()) any_started = true;
    if (_mti8 && _mti8->start()) any_started = true;
    if (_radar && _radar->start()) any_started = true;
    if (_ouster && _ouster->start()) any_started = true;
    if (_gnss && _gnss->start()) any_started = true;

    _started = any_started;
    return _started;
}

void AcquisitionManager::stop() {
    std::lock_guard<std::mutex> lock(_state_mutex);
    if (!_started) return;

    if (_zed) _zed->stop();
    if (_mti8) _mti8->stop();
    if (_radar) _radar->stop();
    if (_ouster) _ouster->stop();
    if (_gnss) _gnss->stop();

    _started = false;
}

std::optional<ZedFrame> AcquisitionManager::zed_latest() {
    return _zed ? _zed->get_latest() : std::nullopt;
}

std::optional<MTiFrame> AcquisitionManager::mti8_latest() {
    return _mti8 ? _mti8->get_latest() : std::nullopt;
}

std::optional<RadarFrame> AcquisitionManager::radar_latest() {
    return _radar ? _radar->get_latest() : std::nullopt;
}

std::optional<OusterFrame> AcquisitionManager::ouster_latest() {
    return _ouster ? _ouster->get_latest() : std::nullopt;
}

std::optional<GNSSFix> AcquisitionManager::gnss_latest() {
    return _gnss ? _gnss->get_latest() : std::nullopt;
}

std::optional<MTiFrame> AcquisitionManager::mti8_nearest(uint64_t ts, uint64_t max_delta) {
    return _mti8 ? _mti8->get_nearest(ts, max_delta) : std::nullopt;
}

std::optional<RadarFrame> AcquisitionManager::radar_nearest(uint64_t ts, uint64_t max_delta) {
    return _radar ? _radar->get_nearest(ts, max_delta) : std::nullopt;
}

std::optional<OusterFrame> AcquisitionManager::ouster_nearest(uint64_t ts, uint64_t max_delta) {
    return _ouster ? _ouster->get_nearest(ts, max_delta) : std::nullopt;
}

std::map<std::string, SensorDiagnostics> AcquisitionManager::all_diagnostics() const {
    std::map<std::string, SensorDiagnostics> result;
    if (_zed) result["zed"] = _zed->diagnostics();
    if (_mti8) result["mti8"] = _mti8->diagnostics();
    if (_radar) result["radar"] = _radar->diagnostics();
    if (_ouster) result["ouster"] = _ouster->diagnostics();
    if (_gnss) result["gnss"] = _gnss->diagnostics();
    return result;
}

bool AcquisitionManager::is_running(const std::string& sensor_id) const {
    if (sensor_id == "zed") return _zed && _zed->is_running();
    if (sensor_id == "mti8") return _mti8 && _mti8->is_running();
    if (sensor_id == "radar") return _radar && _radar->is_running();
    if (sensor_id == "ouster") return _ouster && _ouster->is_running();
    if (sensor_id == "gnss") return _gnss && _gnss->is_running();
    return false;
}

bool AcquisitionManager::zed_opened() const {
    return _zed && _zed->is_opened();
}

bool AcquisitionManager::radar_connected() const {
    return _radar && _radar->connected();
}

bool AcquisitionManager::ouster_streaming() const {
    return _ouster && _ouster->streaming();
}

void AcquisitionManager::zed_start_recording(const std::string& path) {
    if (_zed) _zed->start_recording(path);
}

void AcquisitionManager::zed_stop_recording() {
    if (_zed) _zed->stop_recording();
}

void AcquisitionManager::mti8_start_raw_log(const std::string& path) {
    if (_mti8) _mti8->start_raw_log(path);
}

void AcquisitionManager::mti8_stop_raw_log() {
    if (_mti8) _mti8->stop_raw_log();
}

void AcquisitionManager::ouster_start_pcap(const std::string& path) {
    if (_ouster) _ouster->start_pcap(path);
}

void AcquisitionManager::ouster_stop_pcap() {
    if (_ouster) _ouster->stop_pcap();
}

} // namespace msf
