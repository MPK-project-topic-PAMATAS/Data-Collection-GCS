cmake_minimum_required(VERSION 3.18)
project(msf_backend CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# The pybind11 addon must compile against the 3.10 interpreter (project default)
set(PYBIND11_PYTHON_VERSION 3.10)

# Find pybind11
find_package(pybind11 CONFIG REQUIRED)

# ── ZED SDK ──────────────────────────────────────────────────────────
set(ZED_FOUND FALSE)
if(EXISTS "/usr/local/zed/include/sl/Camera.hpp")
    set(ZED_INCLUDE_DIRS "/usr/local/zed/include")
    set(ZED_LIBRARY_DIR "/usr/local/zed/lib")
    set(ZED_LIBRARIES "${ZED_LIBRARY_DIR}/libsl_zed.so")
    set(ZED_FOUND TRUE)
endif()

# ── Xsens SDK ────────────────────────────────────────────────────────
set(XSENS_FOUND FALSE)
if(EXISTS "/usr/local/xsens/include/xsensdeviceapi.h")
    set(XSENS_INCLUDE_DIRS "/usr/local/xsens/include")
    set(XSENS_LIBRARY_DIR "/usr/local/xsens/lib")
    set(XSENS_LIBRARIES "${XSENS_LIBRARY_DIR}/libxsensdeviceapi.so"
                        "${XSENS_LIBRARY_DIR}/libxstypes.so")
    set(XSENS_FOUND TRUE)
endif()

# ── Sources ──────────────────────────────────────────────────────────
set(BACKEND_SOURCES
    bindings/bindings.cpp
    acquisition/AcquisitionManager.cpp
    sensors/zed/ZEDWorker.cpp
    sensors/mti8/MTi8Worker.cpp
    sensors/radar/AWR1843Worker.cpp
    sensors/ouster/OusterWorker.cpp
    sensors/gnss/GNSSWorker.cpp
)

pybind11_add_module(_sensor_backend ${BACKEND_SOURCES})

target_include_directories(_sensor_backend PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

if(ZED_FOUND)
    target_include_directories(_sensor_backend PRIVATE ${ZED_INCLUDE_DIRS})
    # ZED SDK headers include cuda.h
    if(EXISTS "/usr/local/cuda/include")
        target_include_directories(_sensor_backend PRIVATE "/usr/local/cuda/include")
    endif()
    target_link_libraries(_sensor_backend PRIVATE ${ZED_LIBRARIES})
    message(STATUS "ZED SDK: found at ${ZED_INCLUDE_DIRS}")
else()
    message(STATUS "ZED SDK: NOT found - ZED support disabled")
endif()

if(XSENS_FOUND)
    target_include_directories(_sensor_backend PRIVATE ${XSENS_INCLUDE_DIRS})
    target_link_libraries(_sensor_backend PRIVATE ${XSENS_LIBRARIES})
    target_link_directories(_sensor_backend PRIVATE ${XSENS_LIBRARY_DIR})
    message(STATUS "Xsens SDK: found at ${XSENS_INCLUDE_DIRS}")
else()
    message(STATUS "Xsens SDK: NOT found - MTi8 support disabled")
endif()

# ── Ouster SDK ──────────────────────────────────────────────────────
set(OUSTER_FOUND FALSE)
find_package(OusterSDK CONFIG QUIET)
if(OusterSDK_FOUND)
    set(OUSTER_FOUND TRUE)
    target_link_libraries(_sensor_backend PRIVATE
        OusterSDK::ouster_sensor
        OusterSDK::ouster_core
        OusterSDK::ouster_pcap
    )
    message(STATUS "Ouster SDK: found at ${OusterSDK_DIR}")
else()
    message(STATUS "Ouster SDK: NOT found - Ouster support disabled")
endif()

# Link pthread for std::thread
find_package(Threads REQUIRED)
target_link_libraries(_sensor_backend PRIVATE Threads::Threads)

if(EXISTS "/usr/local/zed/lib/libsl_zed.so")
    file(COPY "${ZED_LIBRARY_DIR}/libsl_zed.so"
         DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/")
endif()
if(XSENS_FOUND)
    file(COPY "${XSENS_LIBRARY_DIR}/libxsensdeviceapi.so"
         DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/")
    file(COPY "${XSENS_LIBRARY_DIR}/libxstypes.so"
         DESTINATION "${CMAKE_CURRENT_BINARY_DIR}/")
endif()
