#include "GNSSWorker.hpp"

#include <chrono>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>

namespace msf {

static bool open_serial_gnss(int& fd, const std::string& port, int baudrate) {
    fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return false;

    struct termios tty{};
    if (tcgetattr(fd, &tty) != 0) { ::close(fd); fd = -1; return false; }

    speed_t speed = B115200;
    if (baudrate == 9600) speed = B9600;
    else if (baudrate == 57600) speed = B57600;
    else if (baudrate == 115200) speed = B115200;

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;
    tcsetattr(fd, TCSANOW, &tty);
    tcflush(fd, TCIOFLUSH);
    return true;
}

GNSSWorker::GNSSWorker(const Config& cfg) : _cfg(cfg), _buffer(10) {
    _sensor_id = "gnss";
}

GNSSWorker::~GNSSWorker() {
    stop();
    if (_serial_fd >= 0) ::close(_serial_fd);
    if (_ntrip_fd >= 0) ::close(_ntrip_fd);
}

bool GNSSWorker::start() {
    if (!open_serial_gnss(_serial_fd, _cfg.serial_port, _cfg.baudrate)) return false;

    // Connect NTRIP
    if (!_cfg.ntrip_host.empty()) {
        struct addrinfo hints{};
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        struct addrinfo* res = nullptr;
        if (getaddrinfo(_cfg.ntrip_host.c_str(), std::to_string(_cfg.ntrip_port).c_str(),
                        &hints, &res) == 0) {
            _ntrip_fd = ::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
            if (_ntrip_fd >= 0) {
                if (::connect(_ntrip_fd, res->ai_addr, res->ai_addrlen) == 0) {
                    // Send NTRIP request
                    std::string req =
                        "GET /" + _cfg.mountpoint + " HTTP/1.0\r\n"
                        "Host: " + _cfg.ntrip_host + "\r\n"
                        "Ntrip-Version: Ntrip/2.0\r\n"
                        "User-Agent: msf_backend/1.0\r\n";
                    if (!_cfg.ntrip_user.empty()) {
                        req += "Authorization: Basic ";
                        // Base64 of user:pass
                        std::string auth = _cfg.ntrip_user + ":" + _cfg.ntrip_pass;
                        static const char* b64tab =
                            "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
                        std::string b64;
                        for (size_t i = 0; i < auth.size(); i += 3) {
                            uint32_t v = static_cast<uint8_t>(auth[i]) << 16;
                            if (i + 1 < auth.size()) v |= static_cast<uint8_t>(auth[i+1]) << 8;
                            if (i + 2 < auth.size()) v |= static_cast<uint8_t>(auth[i+2]);
                            b64 += b64tab[(v >> 18) & 0x3f];
                            b64 += b64tab[(v >> 12) & 0x3f];
                            b64 += (i + 1 < auth.size()) ? b64tab[(v >> 6) & 0x3f] : '=';
                            b64 += (i + 2 < auth.size()) ? b64tab[v & 0x3f] : '=';
                        }
                        req += b64 + "\r\n";
                    }
                    req += "\r\n";
                    ::send(_ntrip_fd, req.c_str(), req.size(), 0);
                }
            }
            freeaddrinfo(res);
        }
    }

    _start_ns = Timestamp::now().host_monotonic_ns;
    start_thread([this]() { _loop(); });
    return true;
}

void GNSSWorker::_loop() {
    char line[512];
    size_t line_len = 0;

    while (_running) {
        struct pollfd pfd{};
        pfd.fd = _serial_fd;
        pfd.events = POLLIN;
        int ret = ::poll(&pfd, 1, 100);

        if (ret <= 0 || !(pfd.revents & POLLIN)) continue;

        char c;
        ssize_t n = ::read(_serial_fd, &c, 1);
        if (n <= 0) continue;

        if (c == '\n') {
            line[line_len] = '\0';
            if (line_len > 0 && line[0] == '$') {
                // Parse NMEA GGA/RMC
                if (std::strstr(line, "GGA") != nullptr) {
                    // Parse: $GPGGA,time,lat,N,lon,E,quality,numSV,HDOP,alt,M,...
                    // Fields: 1=time,2=lat,3=N/S,4=lon,5=E/W,6=quality,7=sat,8=hdop,9=alt
                    int field = 0;
                    char* saveptr = nullptr;
                    char* tok = strtok_r(line, ",", &saveptr);
                    std::string comps[20];
                    while (tok && field < 20) {
                        comps[field++] = tok;
                        tok = strtok_r(nullptr, ",", &saveptr);
                    }
                    if (field >= 10 && !comps[1].empty() && !comps[2].empty()) {
                        GNSSFix fix;
                        fix.timestamp_ns = Timestamp::now().host_monotonic_ns;
                        double lat = std::atof(comps[2].c_str());
                        double lon = std::atof(comps[4].c_str());
                        // NMEA ddmm.mmmm -> decimal degrees
                        int lat_deg = static_cast<int>(lat / 100);
                        fix.latitude = lat_deg + (lat - lat_deg * 100) / 60.0;
                        int lon_deg = static_cast<int>(lon / 100);
                        fix.longitude = lon_deg + (lon - lon_deg * 100) / 60.0;
                        if (comps[3] == "S") fix.latitude = -fix.latitude;
                        if (comps[5] == "W") fix.longitude = -fix.longitude;
                        fix.fix_quality = std::atoi(comps[6].c_str());
                        fix.satellites = std::atoi(comps[7].c_str());
                        fix.hdop = std::atof(comps[8].c_str());
                        fix.altitude = std::atof(comps[9].c_str());
                        fix.rtk_fixed = (fix.fix_quality == 4 || fix.fix_quality == 5);
                        fix.has_fix = (fix.fix_quality > 0);
                        _buffer.push(fix);
                        _fix_count++;
                    }
                }
            }
            line_len = 0;
        } else if (line_len < sizeof(line) - 1) {
            line[line_len++] = c;
        }
    }
}

std::optional<GNSSFix> GNSSWorker::get_latest() { return _buffer.latest(); }

SensorDiagnostics GNSSWorker::diagnostics() const {
    SensorDiagnostics d;
    d.sensor_id = _sensor_id;
    d.configured_rate = 1.0;
    auto elapsed_ns = Timestamp::now().host_monotonic_ns - _start_ns;
    d.measured_rate = (elapsed_ns > 0)
        ? static_cast<double>(_fix_count) / (static_cast<double>(elapsed_ns) / 1e9)
        : 0.0;
    d.received_count = _fix_count;
    d.dropped_count = _error_count;
    d.buffer_size = _buffer.count();
    d.buffer_capacity = _buffer.capacity();
    d.device_status = (_serial_fd >= 0) ? "connected" : "disconnected";
    d.connection_status = d.device_status;
    return d;
}

} // namespace msf
