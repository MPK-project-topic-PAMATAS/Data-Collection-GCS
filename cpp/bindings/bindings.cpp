#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>
#include <pybind11/functional.h>

#include "acquisition/AcquisitionManager.hpp"

namespace py = pybind11;
using namespace msf;

// Helper to convert std::vector to numpy array
template <typename T>
py::array_t<T> to_numpy(const std::vector<T>& vec) {
    if (vec.empty()) {
        return py::array_t<T>({0});
    }
    return py::array_t<T>(vec.size(), vec.data());
}

PYBIND11_MODULE(_sensor_backend, m) {
    m.doc() = "Multisensor fusion C++ acquisition backend";

    // ── Diagnostics ──────────────────────────────────────────────────
    py::class_<SensorDiagnostics>(m, "SensorDiagnostics")
        .def_readwrite("sensor_id", &SensorDiagnostics::sensor_id)
        .def_readwrite("configured_rate", &SensorDiagnostics::configured_rate)
        .def_readwrite("measured_rate", &SensorDiagnostics::measured_rate)
        .def_readwrite("received_count", &SensorDiagnostics::received_count)
        .def_readwrite("dropped_count", &SensorDiagnostics::dropped_count)
        .def_readwrite("buffer_size", &SensorDiagnostics::buffer_size)
        .def_readwrite("buffer_capacity", &SensorDiagnostics::buffer_capacity)
        .def_readwrite("last_timestamp_ns", &SensorDiagnostics::last_timestamp_ns)
        .def_readwrite("latency_ms", &SensorDiagnostics::latency_ms)
        .def_readwrite("device_status", &SensorDiagnostics::device_status)
        .def_readwrite("connection_status", &SensorDiagnostics::connection_status)
        .def_readwrite("last_error", &SensorDiagnostics::last_error)
        .def("__repr__", [](const SensorDiagnostics& d) {
            return "<SensorDiagnostics sensor_id=" + d.sensor_id +
                   " rate=" + std::to_string(d.measured_rate) +
                   " status=" + d.device_status + ">";
        });

    // ── Config ───────────────────────────────────────────────────────
    py::class_<ZEDWorker::Config>(m, "ZEDConfig")
        .def(py::init<>())
        .def_readwrite("resolution", &ZEDWorker::Config::resolution)
        .def_readwrite("fps", &ZEDWorker::Config::fps)
        .def_readwrite("depth_mode", &ZEDWorker::Config::depth_mode)
        .def_readwrite("unit", &ZEDWorker::Config::unit)
        .def_readwrite("coord_system", &ZEDWorker::Config::coord_system)
        .def_readwrite("depth_min", &ZEDWorker::Config::depth_min)
        .def_readwrite("depth_max", &ZEDWorker::Config::depth_max)
        .def_readwrite("depth_stabilization", &ZEDWorker::Config::depth_stabilization)
        .def_readwrite("confidence_thresh", &ZEDWorker::Config::confidence_thresh)
        .def_readwrite("texture_conf_thresh", &ZEDWorker::Config::texture_conf_thresh)
        .def_readwrite("flip_180", &ZEDWorker::Config::flip_180)
        .def_readwrite("save_confidence", &ZEDWorker::Config::save_confidence)
        .def_readwrite("save_disparity", &ZEDWorker::Config::save_disparity);

    py::class_<MTi8Worker::Config>(m, "MTi8Config")
        .def(py::init<>())
        .def_readwrite("port", &MTi8Worker::Config::port)
        .def_readwrite("baudrate", &MTi8Worker::Config::baudrate)
        .def_readwrite("timeout_s", &MTi8Worker::Config::timeout_s);

    py::class_<AWR1843Worker::Config>(m, "RadarConfig")
        .def(py::init<>())
        .def_readwrite("cfg_port", &AWR1843Worker::Config::cfg_port)
        .def_readwrite("data_port", &AWR1843Worker::Config::data_port)
        .def_readwrite("cfg_baud", &AWR1843Worker::Config::cfg_baud)
        .def_readwrite("data_baud", &AWR1843Worker::Config::data_baud)
        .def_readwrite("num_range_bins", &AWR1843Worker::Config::num_range_bins)
        .def_readwrite("num_doppler_bins", &AWR1843Worker::Config::num_doppler_bins);

    py::class_<OusterWorker::Config>(m, "OusterConfig")
        .def(py::init<>())
        .def_readwrite("hostname", &OusterWorker::Config::hostname)
        .def_readwrite("lidar_port", &OusterWorker::Config::lidar_port)
        .def_readwrite("imu_port", &OusterWorker::Config::imu_port);

    py::class_<GNSSWorker::Config>(m, "GNSSConfig")
        .def(py::init<>())
        .def_readwrite("serial_port", &GNSSWorker::Config::serial_port)
        .def_readwrite("baudrate", &GNSSWorker::Config::baudrate)
        .def_readwrite("ntrip_host", &GNSSWorker::Config::ntrip_host)
        .def_readwrite("ntrip_port", &GNSSWorker::Config::ntrip_port)
        .def_readwrite("ntrip_user", &GNSSWorker::Config::ntrip_user)
        .def_readwrite("ntrip_pass", &GNSSWorker::Config::ntrip_pass)
        .def_readwrite("mountpoint", &GNSSWorker::Config::mountpoint);

    py::class_<AcquisitionConfig>(m, "AcquisitionConfig")
        .def(py::init<>())
        .def_readwrite("zed", &AcquisitionConfig::zed)
        .def_readwrite("mti8", &AcquisitionConfig::mti8)
        .def_readwrite("radar", &AcquisitionConfig::radar)
        .def_readwrite("ouster", &AcquisitionConfig::ouster)
        .def_readwrite("gnss", &AcquisitionConfig::gnss)
        .def_readwrite("enable_zed", &AcquisitionConfig::enable_zed)
        .def_readwrite("enable_mti8", &AcquisitionConfig::enable_mti8)
        .def_readwrite("enable_radar", &AcquisitionConfig::enable_radar)
        .def_readwrite("enable_ouster", &AcquisitionConfig::enable_ouster)
        .def_readwrite("enable_gnss", &AcquisitionConfig::enable_gnss)
        .def_readwrite("zed_buffer_size", &AcquisitionConfig::zed_buffer_size)
        .def_readwrite("mti8_buffer_size", &AcquisitionConfig::mti8_buffer_size)
        .def_readwrite("radar_buffer_size", &AcquisitionConfig::radar_buffer_size)
        .def_readwrite("ouster_buffer_size", &AcquisitionConfig::ouster_buffer_size)
        .def_readwrite("gnss_buffer_size", &AcquisitionConfig::gnss_buffer_size);

    // ── Frame types ──────────────────────────────────────────────────
    py::class_<ZedFrame>(m, "ZedFrame")
        .def_readonly("timestamp_ns", &ZedFrame::timestamp_ns)
        .def_readonly("device_timestamp_ns", &ZedFrame::device_timestamp_ns)
        .def_readonly("frame_id", &ZedFrame::frame_id)
        .def_readonly("width", &ZedFrame::width)
        .def_readonly("height", &ZedFrame::height)
        .def_readonly("zed_imu_temperature", &ZedFrame::zed_imu_temperature)
        .def_readonly("zed_imu_timestamp_ns", &ZedFrame::zed_imu_timestamp_ns)
        .def_readonly("exposure", &ZedFrame::exposure)
        .def_readonly("gain", &ZedFrame::gain)
        .def_readonly("temperature", &ZedFrame::temperature)
        .def_property_readonly("left", [](const ZedFrame& f) {
            return py::array_t<uint8_t>(
                py::array::ShapeContainer({f.height, f.width, 3}),
                f.left_rgb.data());
        })
        .def_property_readonly("right", [](const ZedFrame& f) {
            return py::array_t<uint8_t>(
                py::array::ShapeContainer({f.height, f.width, 3}),
                f.right_rgb.data());
        })
        .def_property_readonly("depth", [](const ZedFrame& f) {
            return py::array_t<float>(
                py::array::ShapeContainer({f.height, f.width}),
                f.depth.data());
        })
        .def_property_readonly("confidence", [](const ZedFrame& f) {
            if (f.confidence.empty()) return py::array_t<float>();
            return py::array_t<float>(
                py::array::ShapeContainer({f.height, f.width}),
                f.confidence.data());
        })
        .def_property_readonly("disparity", [](const ZedFrame& f) {
            if (f.disparity.empty()) return py::array_t<float>();
            return py::array_t<float>(
                py::array::ShapeContainer({f.height, f.width}),
                f.disparity.data());
        })
        .def_property_readonly("zed_accel", [](const ZedFrame& f) {
            return std::vector<float>(f.zed_imu_accel, f.zed_imu_accel + 3);
        })
        .def_property_readonly("zed_gyro", [](const ZedFrame& f) {
            return std::vector<float>(f.zed_imu_gyro, f.zed_imu_gyro + 3);
        })
        .def_property_readonly("zed_quat", [](const ZedFrame& f) {
            return std::vector<float>(f.zed_imu_quat, f.zed_imu_quat + 4);
        });

    py::class_<MTiFrame>(m, "MTiFrame")
        .def_readonly("timestamp_ns", &MTiFrame::timestamp_ns)
        .def_readonly("device_timestamp_ns", &MTiFrame::device_timestamp_ns)
        .def_readonly("packet_counter", &MTiFrame::packet_counter)
        .def_readonly("sample_time_fine", &MTiFrame::sample_time_fine)
        .def_readonly("sample_time_coarse", &MTiFrame::sample_time_coarse)
        .def_readonly("latitude", &MTiFrame::latitude)
        .def_readonly("longitude", &MTiFrame::longitude)
        .def_readonly("altitude_msl", &MTiFrame::altitude_msl)
        .def_readonly("altitude_ellipsoid", &MTiFrame::altitude_ellipsoid)
        .def_readonly("temperature", &MTiFrame::temperature)
        .def_readonly("baro_pressure", &MTiFrame::baro_pressure)
        .def_readonly("status_word", &MTiFrame::status_word)
        .def_readonly("status_byte", &MTiFrame::status_byte)
        .def_property_readonly("quaternion", [](const MTiFrame& f) {
            return std::vector<float>(f.quaternion, f.quaternion + 4);
        })
        .def_property_readonly("acceleration", [](const MTiFrame& f) {
            return std::vector<float>(f.acceleration, f.acceleration + 3);
        })
        .def_property_readonly("gyroscope", [](const MTiFrame& f) {
            return std::vector<float>(f.gyroscope, f.gyroscope + 3);
        })
        .def_property_readonly("magnetometer", [](const MTiFrame& f) {
            return std::vector<float>(f.magnetometer, f.magnetometer + 3);
        })
        .def_property_readonly("velocity", [](const MTiFrame& f) {
            return std::vector<float>(f.velocity, f.velocity + 3);
        })
        .def_property_readonly("euler", [](const MTiFrame& f) {
            return std::vector<float>(f.euler, f.euler + 3);
        });

    py::class_<RadarPoint>(m, "RadarPoint")
        .def_readonly("x", &RadarPoint::x)
        .def_readonly("y", &RadarPoint::y)
        .def_readonly("z", &RadarPoint::z)
        .def_readonly("velocity_mps", &RadarPoint::velocity_mps);

    py::class_<RadarFrame>(m, "RadarFrame")
        .def_readonly("timestamp_ns", &RadarFrame::timestamp_ns)
        .def_readonly("frame_id", &RadarFrame::frame_id)
        .def_readonly("num_objects", &RadarFrame::num_objects)
        .def_readonly("num_range_bins", &RadarFrame::num_range_bins)
        .def_readonly("num_doppler_bins", &RadarFrame::num_doppler_bins)
        .def_readonly("points", &RadarFrame::points)
        .def_property_readonly("range_doppler", [](const RadarFrame& f) {
            if (f.range_doppler.empty()) return py::array_t<uint16_t>();
            return py::array_t<uint16_t>(
                py::array::ShapeContainer({(py::ssize_t)f.num_range_bins,
                                           (py::ssize_t)f.num_doppler_bins}),
                f.range_doppler.data());
        })
        .def_property_readonly("range_profile", [](const RadarFrame& f) {
            if (f.range_profile.empty()) return py::array_t<uint16_t>();
            return py::array_t<uint16_t>(
                py::array::ShapeContainer({(py::ssize_t)f.num_range_bins}),
                f.range_profile.data());
        });

    py::class_<OusterFrame>(m, "OusterFrame")
        .def_readonly("timestamp_ns", &OusterFrame::timestamp_ns)
        .def_readonly("frame_id", &OusterFrame::frame_id)
        .def_readonly("num_points", &OusterFrame::num_points)
        .def_readonly("complete", &OusterFrame::complete)
        .def_property_readonly("points", [](const OusterFrame& f) {
            if (f.points_xyz.empty()) return py::array_t<float>();
            return py::array_t<float>(
                py::array::ShapeContainer({(py::ssize_t)(f.points_xyz.size() / 3), 3}),
                f.points_xyz.data());
        })
        .def_property_readonly("reflectivity", [](const OusterFrame& f) {
            if (f.reflectivity.empty()) return py::array_t<float>();
            return py::array_t<float>(f.reflectivity.size(), f.reflectivity.data());
        });

    py::class_<GNSSFix>(m, "GNSSFix")
        .def_readonly("timestamp_ns", &GNSSFix::timestamp_ns)
        .def_readonly("latitude", &GNSSFix::latitude)
        .def_readonly("longitude", &GNSSFix::longitude)
        .def_readonly("altitude", &GNSSFix::altitude)
        .def_readonly("fix_quality", &GNSSFix::fix_quality)
        .def_readonly("satellites", &GNSSFix::satellites)
        .def_readonly("hdop", &GNSSFix::hdop)
        .def_readonly("speed_mps", &GNSSFix::speed_mps)
        .def_readonly("course_deg", &GNSSFix::course_deg)
        .def_readonly("rtk_fixed", &GNSSFix::rtk_fixed)
        .def_readonly("has_fix", &GNSSFix::has_fix);

    // ── AcquisitionManager ───────────────────────────────────────────
    py::class_<AcquisitionManager>(m, "AcquisitionManager")
        .def(py::init<const AcquisitionConfig&>(), py::arg("config"))
        .def("start", &AcquisitionManager::start)
        .def("stop", &AcquisitionManager::stop)

        .def("zed_latest", &AcquisitionManager::zed_latest)
        .def("mti8_latest", &AcquisitionManager::mti8_latest)
        .def("radar_latest", &AcquisitionManager::radar_latest)
        .def("ouster_latest", &AcquisitionManager::ouster_latest)
        .def("gnss_latest", &AcquisitionManager::gnss_latest)

        .def("mti8_nearest", &AcquisitionManager::mti8_nearest,
             py::arg("timestamp_ns"), py::arg("max_delta_ns") = 0)
        .def("radar_nearest", &AcquisitionManager::radar_nearest,
             py::arg("timestamp_ns"), py::arg("max_delta_ns") = 0)
        .def("ouster_nearest", &AcquisitionManager::ouster_nearest,
             py::arg("timestamp_ns"), py::arg("max_delta_ns") = 0)

        .def("get_statistics", &AcquisitionManager::all_diagnostics)

        .def("is_running", &AcquisitionManager::is_running)
        .def("zed_opened", &AcquisitionManager::zed_opened)
        .def("zed_width", &AcquisitionManager::zed_width)
        .def("zed_height", &AcquisitionManager::zed_height)
        .def("radar_connected", &AcquisitionManager::radar_connected)
        .def("ouster_streaming", &AcquisitionManager::ouster_streaming)

        .def("zed_start_recording", &AcquisitionManager::zed_start_recording)
        .def("zed_stop_recording", &AcquisitionManager::zed_stop_recording)
        .def("mti8_start_raw_log", &AcquisitionManager::mti8_start_raw_log)
        .def("mti8_stop_raw_log", &AcquisitionManager::mti8_stop_raw_log)
        .def("ouster_start_pcap", &AcquisitionManager::ouster_start_pcap)
        .def("ouster_stop_pcap", &AcquisitionManager::ouster_stop_pcap);
}
