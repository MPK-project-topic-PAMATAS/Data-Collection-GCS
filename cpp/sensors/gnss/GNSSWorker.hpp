#pragma once

#include <memory>
#include <string>
#include <cstdint>
#include <mutex>
#include "acquisition/SensorWorker.hpp"
#include "acquisition/RingBuffer.hpp"
#include "acquisition/Diagnostics.hpp"

namespace msf {

struct GNSSFix {
    uint64_t timestamp_ns = 0;
    double latitude = 0.0;
    double longitude = 0.0;
    double altitude = 0.0;
    int fix_quality = 0;
    int satellites = 0;
    double hdop = 0.0;
    double speed_mps = 0.0;
    double course_deg = 0.0;
    bool rtk_fixed = false;
    bool has_fix = false;
};

class GNSSWorker : public SensorWorker {
public:
    struct Config {
        std::string serial_port = "/dev/ttyUSB1";
        int baudrate = 115200;
        std::string ntrip_host;
        int ntrip_port = 2111;
        std::string ntrip_user;
        std::string ntrip_pass;
        std::string mountpoint = "VRS_CMRx";
    };

    explicit GNSSWorker(const Config& cfg);
    ~GNSSWorker() override;

    bool start() override;
    SensorDiagnostics diagnostics() const override;

    std::optional<GNSSFix> get_latest();

private:
    void _loop();

    Config _cfg;
    RingBuffer<GNSSFix> _buffer;

    int _serial_fd = -1;
    int _ntrip_fd = -1;

    uint64_t _start_ns = 0;
    uint64_t _fix_count = 0;
    uint64_t _error_count = 0;
};

} // namespace msf
