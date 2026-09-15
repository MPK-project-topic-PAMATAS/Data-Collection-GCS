#include "MTi8Worker.hpp"

#include <chrono>
#include <cstring>
#include <cstdio>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

namespace msf {

// Xbus constants
static constexpr uint8_t PREAMBLE = 0xFA;
static constexpr uint8_t BID = 0xFF;
static constexpr uint8_t EXTENDED_LEN = 0xFF;
static constexpr uint8_t MTDATA2 = 0x36;
static constexpr uint8_t CMD_GO_MEASUREMENT[] = {0xFA, 0xFF, 0x31, 0x00, 0x31};

// XDI Data IDs
static constexpr uint16_t DI_PACKET_COUNTER = 0x1020;
static constexpr uint16_t DI_SAMPLE_TIME_FINE = 0x1060;
static constexpr uint16_t DI_SAMPLE_TIME_COARSE = 0x1070;
static constexpr uint16_t DI_TEMPERATURE = 0x0810;
static constexpr uint16_t DI_QUATERNION = 0x2010;
static constexpr uint16_t DI_ROTATION_MATRIX = 0x2020;
static constexpr uint16_t DI_EULER = 0x2030;
static constexpr uint16_t DI_BARO_PRESSURE = 0x3010;
static constexpr uint16_t DI_DELTA_VELOCITY = 0x4010;
static constexpr uint16_t DI_ACCELERATION = 0x4020;
static constexpr uint16_t DI_RATE_OF_TURN = 0x8020;
static constexpr uint16_t DI_DELTA_QUATERNION = 0x8030;
static constexpr uint16_t DI_LAT_LON = 0x5040;
static constexpr uint16_t DI_ALTITUDE_MSL = 0x5010;
static constexpr uint16_t DI_ALTITUDE_ELLIPSOID = 0x5020;
static constexpr uint16_t DI_MAGNETIC_FIELD = 0xC020;
static constexpr uint16_t DI_VELOCITY_XYZ = 0xD010;
static constexpr uint16_t DI_STATUS_BYTE = 0xE010;
static constexpr uint16_t DI_STATUS_WORD = 0xE020;

struct MTi8Worker::SerialState {
    int fd = -1;
};

// ── helpers ──────────────────────────────────────────────────────────

static bool open_serial_port(int& fd, const std::string& port, int baudrate) {
    fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return false;

    struct termios tty{};
    if (tcgetattr(fd, &tty) != 0) { ::close(fd); fd = -1; return false; }

    // Map baudrate to speed_t
    speed_t speed;
    switch (baudrate) {
        case 9600: speed = B9600; break;
        case 19200: speed = B19200; break;
        case 38400: speed = B38400; break;
        case 57600: speed = B57600; break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        case 460800: speed = B460800; break;
        case 500000: speed = B500000; break;
        case 921600: speed = B921600; break;
        case 1000000: speed = B1000000; break;
        case 1500000: speed = B1500000; break;
        case 2000000: speed = B2000000; break;
        default: speed = B2000000; break;
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
    tty.c_cc[VTIME] = 1; // 100ms inter-byte timeout

    tcsetattr(fd, TCSANOW, &tty);
    tcflush(fd, TCIOFLUSH);
    return true;
}

static float be_float(const uint8_t* p) {
    float v;
    uint8_t b[4] = {p[3], p[2], p[1], p[0]};
    std::memcpy(&v, b, 4);
    return v;
}

static float be_float_le(const uint8_t* p) {
    float v;
    std::memcpy(&v, p, 4);
    return v;
}

static uint16_t be_u16(const uint8_t* p) {
    return (static_cast<uint16_t>(p[0]) << 8) | p[1];
}

static uint32_t be_u32(const uint8_t* p) {
    return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16)
         | (static_cast<uint32_t>(p[2]) << 8) | p[3];
}

static double be_double(const uint8_t* p) {
    double v;
    uint8_t b[8];
    for (int i = 0; i < 8; i++) b[7 - i] = p[i];
    std::memcpy(&v, b, 8);
    return v;
}

// ── MTData2 decoder ──────────────────────────────────────────────────

static MTiFrame decode_mtdata2(const uint8_t* payload, size_t len) {
    MTiFrame f;
    size_t offset = 0;

    while (offset + 3 <= len) {
        uint16_t data_id = be_u16(payload + offset);
        uint8_t size = payload[offset + 2];
        offset += 3;
        if (offset + size > len) break;
        const uint8_t* raw = payload + offset;
        offset += size;

        switch (data_id) {
            case DI_PACKET_COUNTER:
                if (size >= 2) f.packet_counter = be_u16(raw);
                break;
            case DI_SAMPLE_TIME_FINE:
                if (size >= 4) f.sample_time_fine = be_u32(raw);
                break;
            case DI_SAMPLE_TIME_COARSE:
                if (size >= 4) f.sample_time_coarse = be_u32(raw);
                break;
            case DI_TEMPERATURE:
                if (size >= 4) f.temperature = be_float(raw);
                break;
            case DI_QUATERNION:
                if (size >= 16) {
                    f.quaternion[0] = be_float(raw);
                    f.quaternion[1] = be_float(raw + 4);
                    f.quaternion[2] = be_float(raw + 8);
                    f.quaternion[3] = be_float(raw + 12);
                    f.has_quaternion = true;
                }
                break;
            case DI_EULER:
                if (size >= 12) {
                    f.euler[0] = be_float(raw);
                    f.euler[1] = be_float(raw + 4);
                    f.euler[2] = be_float(raw + 8);
                }
                break;
            case DI_ROTATION_MATRIX:
                if (size >= 36) {
                    for (int i = 0; i < 9; i++)
                        f.rotation_matrix[i] = be_float(raw + i * 4);
                }
                break;
            case DI_BARO_PRESSURE:
                if (size >= 4) f.baro_pressure = be_float(raw);
                break;
            case DI_ACCELERATION:
                if (size >= 12) {
                    f.acceleration[0] = be_float(raw);
                    f.acceleration[1] = be_float(raw + 4);
                    f.acceleration[2] = be_float(raw + 8);
                    f.has_acceleration = true;
                }
                break;
            case DI_DELTA_VELOCITY:
                if (size >= 12) {
                    f.delta_velocity[0] = be_float(raw);
                    f.delta_velocity[1] = be_float(raw + 4);
                    f.delta_velocity[2] = be_float(raw + 8);
                }
                break;
            case DI_RATE_OF_TURN:
                if (size >= 12) {
                    f.gyroscope[0] = be_float(raw);
                    f.gyroscope[1] = be_float(raw + 4);
                    f.gyroscope[2] = be_float(raw + 8);
                    f.has_gyroscope = true;
                }
                break;
            case DI_DELTA_QUATERNION:
                if (size >= 16) {
                    f.delta_quaternion[0] = be_float(raw);
                    f.delta_quaternion[1] = be_float(raw + 4);
                    f.delta_quaternion[2] = be_float(raw + 8);
                    f.delta_quaternion[3] = be_float(raw + 12);
                }
                break;
            case DI_LAT_LON:
                if (size >= 16) {
                    f.latitude = be_double(raw);
                    f.longitude = be_double(raw + 8);
                }
                break;
            case DI_ALTITUDE_MSL:
                if (size >= 8) f.altitude_msl = be_double(raw);
                break;
            case DI_ALTITUDE_ELLIPSOID:
                if (size >= 8) f.altitude_ellipsoid = be_double(raw);
                break;
            case DI_MAGNETIC_FIELD:
                if (size >= 12) {
                    f.magnetometer[0] = be_float(raw);
                    f.magnetometer[1] = be_float(raw + 4);
                    f.magnetometer[2] = be_float(raw + 8);
                    f.has_magnetometer = true;
                }
                break;
            case DI_VELOCITY_XYZ:
                if (size >= 12) {
                    f.velocity[0] = be_float(raw);
                    f.velocity[1] = be_float(raw + 4);
                    f.velocity[2] = be_float(raw + 8);
                }
                break;
            case DI_STATUS_BYTE:
                if (size >= 1) f.status_byte = raw[0];
                break;
            case DI_STATUS_WORD:
                if (size >= 4) f.status_word = be_u32(raw);
                break;
            default:
                break;
        }
    }
    return f;
}

// ── Xbus parser ──────────────────────────────────────────────────────

struct XbusPacket {
    uint8_t mid = 0;
    std::vector<uint8_t> payload;
};

static std::vector<XbusPacket> parse_xbus(std::vector<uint8_t>& buf) {
    std::vector<XbusPacket> packets;

    while (true) {
        // Find preamble
        while (!buf.empty() && buf[0] != PREAMBLE) {
            buf.erase(buf.begin());
        }
        if (buf.size() < 4) break;

        if (buf[1] != BID) {
            buf.erase(buf.begin());
            continue;
        }

        uint8_t mid = buf[2];
        uint8_t len_byte = buf[3];

        size_t header_size;
        uint16_t length;
        bool extended = false;

        if (len_byte == EXTENDED_LEN) {
            if (buf.size() < 6) break;
            length = (static_cast<uint16_t>(buf[4]) << 8) | buf[5];
            header_size = 6;
            extended = true;
        } else {
            length = len_byte;
            header_size = 4;
        }

        size_t total = header_size + length + 1; // +1 for checksum
        if (buf.size() < total) break;

        // Checksum: sum of bytes from BID through checksum == 0 mod 256
        uint8_t sum = 0;
        for (size_t i = 1; i < total; i++) sum += buf[i];
        if (sum != 0) {
            buf.erase(buf.begin());
            continue;
        }

        XbusPacket pkt;
        pkt.mid = mid;
        pkt.payload.assign(buf.begin() + header_size, buf.begin() + header_size + length);
        packets.push_back(std::move(pkt));

        buf.erase(buf.begin(), buf.begin() + total);
    }
    return packets;
}

// ── MTi8Worker ───────────────────────────────────────────────────────

MTi8Worker::MTi8Worker(const Config& cfg, size_t buffer_size)
    : _cfg(cfg), _buffer(buffer_size), _serial_state(std::make_unique<SerialState>()) {
    _sensor_id = "mti8";
}

MTi8Worker::~MTi8Worker() {
    stop();
    stop_raw_log();
    if (_serial_state->fd >= 0) {
        ::close(_serial_state->fd);
        _serial_state->fd = -1;
    }
}

bool MTi8Worker::_open_port() {
    return open_serial_port(_serial_state->fd, _cfg.port, _cfg.baudrate);
}

void MTi8Worker::_send_command(const uint8_t* cmd, size_t len) {
    if (_serial_state->fd >= 0) {
        ::write(_serial_state->fd, cmd, len);
    }
}

bool MTi8Worker::start() {
    if (!_open_port()) return false;
    _send_command(CMD_GO_MEASUREMENT, sizeof(CMD_GO_MEASUREMENT));
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    _start_ns = Timestamp::now().host_monotonic_ns;
    start_thread([this]() { _acquisition_loop(); });
    return true;
}

void MTi8Worker::_acquisition_loop() {
    uint8_t read_buf[4096];

    while (_running) {
        struct pollfd pfd{};
        pfd.fd = _serial_state->fd;
        pfd.events = POLLIN;
        int ret = ::poll(&pfd, 1, 10); // 10ms timeout

        if (ret <= 0 || !(pfd.revents & POLLIN)) continue;

        ssize_t n = ::read(_serial_state->fd, read_buf, sizeof(read_buf));
        if (n <= 0) continue;

        _buf.insert(_buf.end(), read_buf, read_buf + n);

        // Raw log
        {
            std::lock_guard<std::mutex> lock(_raw_mutex);
            if (_raw_file) {
                fwrite(read_buf, 1, n, _raw_file);
                fflush(_raw_file);
            }
        }

        auto packets = parse_xbus(_buf);
        for (auto& pkt : packets) {
            if (pkt.mid != MTDATA2) continue;

            auto frame = decode_mtdata2(pkt.payload.data(), pkt.payload.size());
            frame.timestamp_ns = Timestamp::now().host_monotonic_ns;
            _buffer.push(frame);
            _frame_count++;
        }
    }
}

void MTi8Worker::start_raw_log(const std::string& path) {
    std::lock_guard<std::mutex> lock(_raw_mutex);
    _raw_file = fopen(path.c_str(), "wb");
}

void MTi8Worker::stop_raw_log() {
    std::lock_guard<std::mutex> lock(_raw_mutex);
    if (_raw_file) {
        fclose(_raw_file);
        _raw_file = nullptr;
    }
}

std::optional<MTiFrame> MTi8Worker::get_latest() { return _buffer.latest(); }
std::optional<MTiFrame> MTi8Worker::get_nearest(uint64_t ts, uint64_t max_delta) {
    return _buffer.nearest(ts, max_delta);
}

SensorDiagnostics MTi8Worker::diagnostics() const {
    SensorDiagnostics d;
    d.sensor_id = _sensor_id;
    d.configured_rate = 120.0;
    auto elapsed_ns = Timestamp::now().host_monotonic_ns - _start_ns;
    d.measured_rate = (elapsed_ns > 0)
        ? static_cast<double>(_frame_count) / (static_cast<double>(elapsed_ns) / 1e9)
        : 0.0;
    d.received_count = _frame_count;
    d.dropped_count = _error_count;
    d.buffer_size = _buffer.count();
    d.buffer_capacity = _buffer.capacity();
    d.device_status = (_serial_state->fd >= 0) ? "connected" : "disconnected";
    d.connection_status = d.device_status;
    return d;
}

} // namespace msf
