#include "OusterWorker.hpp"

#include <chrono>
#include <cstring>
#include <sys/time.h>

#include "ouster/core/lidar_frame.h"
#include "ouster/core/types.h"
#include "ouster/core/xyzlut.h"
#include "ouster/sensor/sensor_packet_source.h"

namespace msf {

using ouster::sdk::core::FrameBatcher;
using ouster::sdk::core::LidarFrame;
using ouster::sdk::core::PacketType;
using ouster::sdk::core::PointCloudXYZd;
using ouster::sdk::core::SensorConfig;
using ouster::sdk::core::SensorInfo;
using ouster::sdk::core::XYZLut;
using ouster::sdk::sensor::ClientEvent;
using ouster::sdk::sensor::Sensor;
using ouster::sdk::sensor::SensorPacketSource;

OusterWorker::OusterWorker(const Config& cfg)
    : _cfg(cfg), _buffer(cfg.buffer_size) {
    _sensor_id = "ouster";
}

OusterWorker::~OusterWorker() {
    stop();
    stop_pcap();
    _reset_sdk();
}

void OusterWorker::_reset_sdk() {
    _batcher.reset();
    _frame.reset();
    _xyz_lut.reset();
    _info.reset();
    _client.reset();
    _sensor.reset();
}

bool OusterWorker::start() {
    try {
        // Configure the sensor to stream to this host (udp_dest = "@auto" makes
        // the SDK pick the destination interface automatically).
        SensorConfig config;
        config.udp_dest = "@auto";
        _sensor = std::make_unique<Sensor>(_cfg.hostname, config);

        // The packet source configures the sensor over HTTP and receives its
        // UDP data stream, caching the sensor metadata (SensorInfo).
        _client = std::make_unique<SensorPacketSource>(std::vector<Sensor>{*_sensor}, 30.0, 1.0);
        if (_client->sensor_info().empty()) {
            _reset_sdk();
            return false;
        }
        _info = _client->sensor_info().front();

        // Build the frame decoder + point-cloud lookup table.
        _batcher = std::make_unique<FrameBatcher>(_info);
        _frame = std::make_unique<LidarFrame>(_info);
        _xyz_lut = std::make_unique<XYZLut>(*_info, true);

        _sensor_name = "OS-" + _cfg.hostname;
        _start_ns = Timestamp::now().host_monotonic_ns;
        // RingBuffer was constructed empty; nothing to reset.
        start_thread([this]() { _loop(); });
        return true;
    } catch (const std::exception& e) {
        _last_error = e.what();
        _reset_sdk();
        return false;
    }
}

void OusterWorker::_loop() {
    while (_running) {
        auto event = _client->get_packet(1.0);
        switch (event.type) {
            case ClientEvent::PACKET:
                if (event.packet().type() == PacketType::Lidar) {
                    auto& lidar_packet = static_cast<ouster::sdk::core::LidarPacket&>(
                        event.packet());
                    _packet_count++;
                    _write_pcap_record(lidar_packet.buf.data(), lidar_packet.buf.size());
                    if (_batcher->batch(lidar_packet, *_frame) &&
                        _frame->complete(_info->format.column_window)) {
                        OusterFrame f;
                        f.timestamp_ns = Timestamp::now().host_monotonic_ns;
                        f.frame_id = static_cast<int>(_frame->frame_id);
                        f.complete = true;

                        PointCloudXYZd cloud = (*_xyz_lut)(*_frame);
                        const int n = static_cast<int>(cloud.rows());
                        f.num_points = n;
                        f.points_xyz.resize(static_cast<size_t>(n) * 3);
                        for (int i = 0; i < n; i++) {
                            f.points_xyz[static_cast<size_t>(i) * 3 + 0] =
                                static_cast<float>(cloud.row(i)(0));
                            f.points_xyz[static_cast<size_t>(i) * 3 + 1] =
                                static_cast<float>(cloud.row(i)(1));
                            f.points_xyz[static_cast<size_t>(i) * 3 + 2] =
                                static_cast<float>(cloud.row(i)(2));
                        }

                        try {
                            auto refl = _frame->field<uint32_t>(ouster::sdk::core::ChanField::REFLECTIVITY);
                            f.reflectivity.resize(static_cast<size_t>(n));
                            for (int i = 0; i < n; i++) {
                                f.reflectivity[static_cast<size_t>(i)] =
                                    static_cast<float>(refl.data()[i]);
                            }
                        } catch (...) {
                            f.reflectivity.clear();
                        }

                        _buffer.push(std::move(f));
                        _frame_count++;
                        _last_frame_ns = f.timestamp_ns;
                    }
                }
                break;
            case ClientEvent::ERR:
                if (_running) {
                    _error_count++;
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
                break;
            case ClientEvent::POLL_TIMEOUT:
            case ClientEvent::EXIT:
            default:
                break;
        }
    }
}

void OusterWorker::_write_pcap_record(const uint8_t* data, size_t len) {
    std::lock_guard<std::mutex> lock(_pcap_mutex);
    if (!_pcap_file) return;
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    uint32_t us_sec = static_cast<uint32_t>(tv.tv_sec);
    uint32_t us_usec = static_cast<uint32_t>(tv.tv_usec);
    uint32_t cap_len = static_cast<uint32_t>(len);
    fwrite(&us_sec, 4, 1, _pcap_file);
    fwrite(&us_usec, 4, 1, _pcap_file);
    fwrite(&cap_len, 4, 1, _pcap_file);
    fwrite(&cap_len, 4, 1, _pcap_file);
    fwrite(data, 1, len, _pcap_file);
    _pcap_packets++;
}

std::optional<OusterFrame> OusterWorker::get_latest() { return _buffer.latest(); }
std::optional<OusterFrame> OusterWorker::get_nearest(uint64_t ts, uint64_t max_delta) {
    return _buffer.nearest(ts, max_delta);
}

bool OusterWorker::streaming() const {
    return _last_frame_ns > 0 &&
           (Timestamp::now().host_monotonic_ns - _last_frame_ns) < 1000000000ull;
}

void OusterWorker::start_pcap(const std::string& path) {
    std::lock_guard<std::mutex> lock(_pcap_mutex);
    if (_pcap_file) {
        fclose(_pcap_file);
        _pcap_file = nullptr;
    }
    _pcap_file = fopen(path.c_str(), "wb");
    if (_pcap_file) {
        // PCAP global header - 24 bytes
        uint32_t magic = 0xa1b2c3d4;
        uint16_t vmajor = 2, vminor = 4;
        int32_t tz = 0, sigfigs = 0;
        uint32_t snaplen = 65535;
        uint32_t network = 1; // Ethernet
        fwrite(&magic, 4, 1, _pcap_file);
        fwrite(&vmajor, 2, 1, _pcap_file);
        fwrite(&vminor, 2, 1, _pcap_file);
        fwrite(&tz, 4, 1, _pcap_file);
        fwrite(&sigfigs, 4, 1, _pcap_file);
        fwrite(&snaplen, 4, 1, _pcap_file);
        fwrite(&network, 4, 1, _pcap_file);
        _pcap_packets = 0;
    }
}

void OusterWorker::stop_pcap() {
    std::lock_guard<std::mutex> lock(_pcap_mutex);
    if (_pcap_file) {
        fclose(_pcap_file);
        _pcap_file = nullptr;
    }
}

SensorDiagnostics OusterWorker::diagnostics() const {
    SensorDiagnostics d;
    d.sensor_id = _sensor_id;
    d.configured_rate = 10.0;
    auto elapsed_ns = Timestamp::now().host_monotonic_ns - _start_ns;
    d.measured_rate = (elapsed_ns > 0)
        ? static_cast<double>(_frame_count) / (static_cast<double>(elapsed_ns) / 1e9)
        : 0.0;
    d.received_count = _frame_count;
    d.packet_count = _packet_count;
    d.dropped_count = _error_count;
    d.buffer_size = _buffer.count();
    d.buffer_capacity = _buffer.capacity();
    d.device_status = streaming() ? "connected" : "disconnected";
    d.connection_status = d.device_status;
    d.last_error = _last_error;
    return d;
}

} // namespace msf
