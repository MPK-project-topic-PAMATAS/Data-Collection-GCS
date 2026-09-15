#include "AWR1843Worker.hpp"

#include <chrono>
#include <cstring>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <algorithm>

namespace msf {

// AWR1843 frame magic
static const uint8_t MAGIC[8] = {0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07};
static constexpr size_t HEADER_SIZE = 40;
static constexpr size_t TOTAL_LEN_OFFSET = 12;
static constexpr uint32_t MAX_FRAME = 0x20000; // 128 KiB

// TLV types
static constexpr uint32_t TLV_DETECTED_POINTS = 1;
static constexpr uint32_t TLV_RANGE_PROFILE = 2;
static constexpr uint32_t TLV_RANGE_DOPPLER = 5;

// Radar config commands (matches Python _SENSOR_CONFIG_CMDS)
static const std::vector<std::string> SENSOR_CONFIG_CMDS = {
    "sensorStop",
    "flushCfg",
    "dfeDataOutputMode 1",
    "channelCfg 15 5 0",
    "adcCfg 2 1",
    "adcbufCfg -1 0 1 1 1",
    "profileCfg 0 77 429 7 57.14 0 0 70 1 256 5209 0 0 30",
    "chirpCfg 0 0 0 0 0 0 0 1",
    "chirpCfg 1 1 0 0 0 0 0 4",
    "frameCfg 0 1 16 0 150 1 0",
    "lowPower 0 0",
    "guiMonitor -1 1 0 0 0 1 0",
    "cfarCfg -1 0 2 8 4 3 0 15 1",
    "cfarCfg -1 1 0 4 2 3 1 15 1",
    "multiObjBeamForming -1 1 0.5",
    "clutterRemoval -1 0",
    "calibDcRangeSig -1 0 -5 8 256",
    "extendedMaxVelocity -1 0",
    "lvdsStreamCfg -1 0 0 0",
    "compRangeBiasAndRxChanPhase 0.0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0 1 0",
    "measureRangeBiasAndRxChanPhase 0 1.5 0.2",
    "CQRxSatMonitor 0 3 5 121 0",
    "CQSigImgMonitor 0 127 4",
    "analogMonitor 0 0",
    "aoaFovCfg -1 -90 90 -90 90",
    "cfarFovCfg -1 0 0 8.92",
    "cfarFovCfg -1 1 -1 1.00",
};

// ── serial helpers ───────────────────────────────────────────────────

static bool open_serial(int& fd, const std::string& port, int baudrate) {
    fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return false;

    struct termios tty{};
    if (tcgetattr(fd, &tty) != 0) { ::close(fd); fd = -1; return false; }

    speed_t speed = B921600;
    switch (baudrate) {
        case 115200: speed = B115200; break;
        case 921600: speed = B921600; break;
        default: speed = B921600; break;
    }
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    tcsetattr(fd, TCSANOW, &tty);
    tcflush(fd, TCIOFLUSH);
    return true;
}

static uint32_t le_u32(const uint8_t* p) {
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8)
         | (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

static float le_float(const uint8_t* p) {
    float v;
    uint8_t b[4] = {p[0], p[1], p[2], p[3]};
    std::memcpy(&v, b, 4);
    return v;
}

// ── AWR1843Worker ────────────────────────────────────────────────────

AWR1843Worker::AWR1843Worker(const Config& cfg, size_t buffer_size)
    : _cfg(cfg), _buffer(buffer_size) {
    _sensor_id = "radar";
}

AWR1843Worker::~AWR1843Worker() {
    stop();
    _close_ports();
}

bool AWR1843Worker::_open_ports() {
    std::lock_guard<std::mutex> lock(_port_mutex);
    if (!open_serial(_cfg_fd, _cfg.cfg_port, _cfg.cfg_baud)) return false;
    if (!open_serial(_data_fd, _cfg.data_port, _cfg.data_baud)) {
        ::close(_cfg_fd);
        _cfg_fd = -1;
        return false;
    }
    return true;
}

bool AWR1843Worker::_wait_prompt(int timeout_s) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(timeout_s);
    std::vector<uint8_t> buf;

    while (std::chrono::steady_clock::now() < deadline) {
        struct pollfd pfd{};
        pfd.fd = _cfg_fd;
        pfd.events = POLLIN;
        int ret = ::poll(&pfd, 1, 50);
        if (ret > 0) {
            uint8_t chunk[256];
            ssize_t n = ::read(_cfg_fd, chunk, sizeof(chunk));
            if (n > 0) {
                buf.insert(buf.end(), chunk, chunk + n);
                std::string s(buf.begin(), buf.end());
                if (s.find("mmwDemo:/>") != std::string::npos ||
                    s.find("mmwDemo: >") != std::string::npos) {
                    return true;
                }
            }
        }
    }
    return false;
}

bool AWR1843Worker::_apply_config() {
    for (const auto& cmd : SENSOR_CONFIG_CMDS) {
        std::string line = cmd + "\n";
        ::write(_cfg_fd, line.c_str(), line.size());
        if (!_wait_prompt(3)) {
            return false;
        }
    }
    const char* start_cmd = "sensorStart\n";
    ::write(_cfg_fd, start_cmd, strlen(start_cmd));
    return true;
}

bool AWR1843Worker::start() {
    if (!_open_ports()) return false;
    if (!_apply_config()) {
        _close_ports();
        return false;
    }

    _start_ns = Timestamp::now().host_monotonic_ns;
    start_thread([this]() { _data_loop(); });
    return true;
}

void AWR1843Worker::_data_loop() {
    uint8_t read_buf[4096];

    while (_running) {
        struct pollfd pfd{};
        pfd.fd = _data_fd;
        pfd.events = POLLIN;
        int ret = ::poll(&pfd, 1, 20);

        if (ret <= 0 || !(pfd.revents & POLLIN)) continue;

        ssize_t n = ::read(_data_fd, read_buf, sizeof(read_buf));
        if (n <= 0) {
            _error_count++;
            if (_running) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            continue;
        }

        _buf.insert(_buf.end(), read_buf, read_buf + n);
        _extract_frames();
    }
}

void AWR1843Worker::_extract_frames() {
    while (true) {
        // Align to 8-byte magic
        auto it = std::search(_buf.begin(), _buf.end(), MAGIC, MAGIC + 8);
        if (it == _buf.end()) {
            if (_buf.size() >= 8) {
                // Drop all but last 7 bytes for partial magic
                _buf.erase(_buf.begin(), _buf.end() - 7);
            }
            return;
        }
        if (it != _buf.begin()) {
            _buf.erase(_buf.begin(), it);
        }

        if (_buf.size() < TOTAL_LEN_OFFSET + 4) return;
        uint32_t total_len = le_u32(_buf.data() + TOTAL_LEN_OFFSET);
        if (total_len < 36 || total_len > MAX_FRAME) {
            _buf.erase(_buf.begin());
            continue;
        }

        if (_buf.size() < total_len) return;

        // Extract full frame
        std::vector<uint8_t> frame(_buf.begin(), _buf.begin() + total_len);
        _buf.erase(_buf.begin(), _buf.begin() + total_len);

        // Parse header
        if (frame.size() < HEADER_SIZE) continue;
        if (std::memcmp(frame.data(), MAGIC, 8) != 0) continue;

        uint32_t frame_number = le_u32(frame.data() + 20);
        uint32_t num_detected_obj = le_u32(frame.data() + 28);
        uint32_t num_tlvs = le_u32(frame.data() + 32);

        RadarFrame rf;
        rf.timestamp_ns = Timestamp::now().host_monotonic_ns;
        rf.frame_id = frame_number;
        rf.num_objects = num_detected_obj;
        rf.num_range_bins = _cfg.num_range_bins;
        rf.num_doppler_bins = _cfg.num_doppler_bins;

        bool have_points = false;
        bool have_rd = false;
        bool have_rp = false;

        size_t offset = HEADER_SIZE;
        size_t n = frame.size();

        for (uint32_t tlv_idx = 0; tlv_idx < num_tlvs && offset + 8 <= n; tlv_idx++) {
            uint32_t tlv_type = le_u32(frame.data() + offset);
            uint32_t tlv_len = le_u32(frame.data() + offset + 4);
            size_t body = offset + 8;
            if (body + tlv_len > n) break;
            const uint8_t* payload = frame.data() + body;

            if (tlv_type == TLV_DETECTED_POINTS && !have_points) {
                uint32_t count = std::min(num_detected_obj, tlv_len / 16u);
                rf.points.reserve(count);
                for (uint32_t i = 0; i < count; i++) {
                    const uint8_t* p = payload + i * 16;
                    RadarPoint pt;
                    pt.x = le_float(p);
                    pt.y = le_float(p + 4);
                    pt.z = le_float(p + 8);
                    pt.velocity_mps = le_float(p + 12);
                    rf.points.push_back(pt);
                }
                have_points = true;
            }
            else if (tlv_type == TLV_RANGE_DOPPLER && !have_rd) {
                size_t expected = static_cast<size_t>(_cfg.num_range_bins) *
                                  _cfg.num_doppler_bins * 2;
                if (tlv_len >= expected && body + expected <= n) {
                    rf.range_doppler.resize(_cfg.num_range_bins * _cfg.num_doppler_bins);
                    std::memcpy(rf.range_doppler.data(), payload, expected);
                    have_rd = true;
                }
            }
            else if (tlv_type == TLV_RANGE_PROFILE && !have_rp) {
                size_t expected = static_cast<size_t>(_cfg.num_range_bins) * 2;
                if (tlv_len >= expected && body + expected <= n) {
                    rf.range_profile.resize(_cfg.num_range_bins);
                    std::memcpy(rf.range_profile.data(), payload, expected);
                    have_rp = true;
                }
            }

            offset = body + tlv_len;
        }

        if (have_points || have_rd || have_rp) {
            _buffer.push(std::move(rf));
            _frame_count++;
        }
    }
}

std::optional<RadarFrame> AWR1843Worker::get_latest() { return _buffer.latest(); }
std::optional<RadarFrame> AWR1843Worker::get_nearest(uint64_t ts, uint64_t max_delta) {
    return _buffer.nearest(ts, max_delta);
}

bool AWR1843Worker::connected() const {
    std::lock_guard<std::mutex> lock(_port_mutex);
    return _data_fd >= 0;
}

void AWR1843Worker::_close_ports() {
    std::lock_guard<std::mutex> lock(_port_mutex);
    if (_cfg_fd >= 0) {
        // Try to stop the sensor
        const char* stop_cmd = "sensorStop\n";
        ::write(_cfg_fd, stop_cmd, strlen(stop_cmd));
        ::close(_cfg_fd);
        _cfg_fd = -1;
    }
    if (_data_fd >= 0) {
        ::close(_data_fd);
        _data_fd = -1;
    }
}

SensorDiagnostics AWR1843Worker::diagnostics() const {
    SensorDiagnostics d;
    d.sensor_id = _sensor_id;
    d.configured_rate = 6.67; // 150ms frame period
    auto elapsed_ns = Timestamp::now().host_monotonic_ns - _start_ns;
    d.measured_rate = (elapsed_ns > 0)
        ? static_cast<double>(_frame_count) / (static_cast<double>(elapsed_ns) / 1e9)
        : 0.0;
    d.received_count = _frame_count;
    d.dropped_count = _error_count;
    d.buffer_size = _buffer.count();
    d.buffer_capacity = _buffer.capacity();
    d.device_status = connected() ? "connected" : "disconnected";
    d.connection_status = d.device_status;
    return d;
}

} // namespace msf
