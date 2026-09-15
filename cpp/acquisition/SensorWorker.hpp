#pragma once

#include <string>
#include <thread>
#include <atomic>
#include <functional>
#include "Timestamp.hpp"
#include "Diagnostics.hpp"

namespace msf {

class SensorWorker {
public:
    virtual ~SensorWorker() { stop(); }

    virtual bool start() = 0;

    void stop() {
        _running = false;
        if (_thread && _thread->joinable()) {
            _thread->join();
        }
    }

    bool is_running() const { return _running.load(); }
    const std::string& sensor_id() const { return _sensor_id; }
    virtual SensorDiagnostics diagnostics() const = 0;

protected:
    void start_thread(std::function<void()> loop_fn) {
        _running = true;
        _thread = std::make_unique<std::thread>(std::move(loop_fn));
    }

    std::string _sensor_id;
    std::atomic<bool> _running{false};
    std::unique_ptr<std::thread> _thread;
};

} // namespace msf
