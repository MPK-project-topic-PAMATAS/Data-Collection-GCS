**MPK trial recording assessment — 11 September 2026**

**Verdict: keep the native recording formats, but fix the recorder and timing contract before scaling up.** This is a useful capture prototype. The supplied recording does not yet demonstrate reliable GPS-RTK + IMU + LiDAR + camera synchronization or reproducible algorithm replay. Folder organization alone cannot supply acquisition timing or calibration that was never recorded.

The assessment covers the supplied `output` directory: 366 original files, 597,555,093 bytes (597.6 MB decimal). The declared session is `20260911_112718`, lasting 17.080 seconds, from 11:27:29.845030 to 11:27:46.925322 UTC according to the host-derived metadata. These wall-clock values were not independently verified against UTC. The earlier time in the session ID can be an allocation time, but that distinction needs documentation.

I inspected every Parquet table, all LiDAR packet/column headers, checksum-valid MTi messages, radar JSON/NumPy payloads, and the radar bag records. I inspected HDF5 datasets through a diagnostic in-memory copy after the original failed to open. Original recording files were not modified. The SVO2 file was not decoded with the ZED SDK, so its frame count, embedded sensor completeness, image quality, and successful end-to-end playback remain unverified. The recording/replay application source and the algorithms previously researched in MPK were not available here; algorithm examples below are representative, not a claim about the project's selected methods.

**What is already good**

- Native SVO2, LiDAR PCAP plus sensor JSON, and MTi Xbus preserve substantially more future processing flexibility than image/point-cloud exports alone.
- Recording independent sensor streams at their native rates and making synchronization a derived offline operation is the correct architecture.
- Parquet is suitable for searchable indexes; HDF5 is suitable for dense sensor arrays when finalized correctly and supplied with explicit schemas.
- The configuration includes the application Git commit, camera identity, LiDAR serial/firmware, 64 beam calibrations, packet profile, resolution, and recording settings.
- Relative payload paths are portable. Existing nonempty radar index references resolve correctly.
- Missing external calibration is explicitly marked `not_calibrated` rather than silently presented as measured calibration.

**Measured condition of each stream**

| Stream | What the supplied files contain | Assessment |
|---|---|---|
| ZED 2 | 323.5 MB SVO2; 495 metadata rows, frame IDs 272–766; indexed span 16.704 s; median metadata interval 33.340 ms, maximum 127.218 ms | Appropriate container, but no explicit SVO frame-position/native-timestamp mapping in the table. Decode verification is still required. |
| Ouster OS-0-64-U02 | 271.8 MB PCAP; 21,568 LiDAR packets; 337 consecutive complete scans, IDs 5838–6174; 64 channels × 1,024 columns, configured 20 Hz | Strong raw coverage within the recorded PCAP. No missing/duplicate columns or invalid column flags found. This does not prove synchronized acquisition. |
| MTi raw | 1,747 complete checksum-valid messages; 1,679 contain acceleration/gyro, 68 contain GNSS PVT; no missing packet counters between complete messages | Useful native data. Inertial measurements are approximately 100 Hz by device ticks. There are 135 boundary fragment bytes outside complete messages. |
| MTi HDF5 | 1,857 rows, including 128 before the session; ordinary read fails; diagnostic read reveals batching, missing-field zeros, and a pressure decoding error | Structured export needs correction and reliable finalization. |
| GPS/RTK | NMEA log is 0 bytes; `metadata/gnss/` is empty; all 68 raw PVT messages report no fix and zero satellites; no valid GNSS fix flag in raw MTi messages | This trial contains no usable GPS/RTK trajectory. Enabling GNSS/NTRIP in configuration does not demonstrate successful positioning or corrections. |
| Radar, also present | 171 point JSONs, 170 heatmaps, 171 index rows; span 25.506 s; 57 rows before the session; bag contains 112 point messages and 112 `/clock` messages | Useful additional stream, but recording windows and product coverage differ. Frame 89 has no heatmap and its index path is correctly empty. |

The LiDAR packet scan spans are approximately 49.95 ms. The configured 20 Hz scan is therefore an interval of measurements, not one simultaneous 3D exposure. Preserve column timestamps and measurement IDs when decoding. The manufacturer documents those fields and the configured 12,544-byte packet layout. [Ouster packet format](https://static.ouster.dev/sensor-docs/image_route1/image_route3/sensor_data/sensor-data.html)

**Highest-priority findings**

**1. The persisted clock domains contradict the configuration.**

`metadata/config_snapshot_20260911_112718.json` says all sensor `timestamp_ns` fields use host monotonic time. In fact, the camera and radar tables and both HDF5 `timestamp_ns` and `host_timestamp_ns` contain Unix-epoch-sized timestamps. The camera table's `nearest_mti_timestamp_ns` contains monotonic-sized values.

For camera frame 272:

```text
timestamp_ns                 = 1789126050105950627
nearest_mti_timestamp_ns     =      23520566885882
sync_delta_ns               =                112304
sensor_hdf5_file            = ""
nearest_mti_index           = -1
```

Subtracting the two stored timestamps does not produce the reported delta. There is no persisted, versioned clock mapping explaining this relation. Six camera rows have an unavailable IMU association. Even matched rows lack a usable HDF5 row locator. The claimed small delta is therefore not evidence of physical camera–IMU alignment.

The HDF5 time equals host time in every row. Its median adjacent interval is only 0.056 ms, while device ticks usually advance 10 ms: measurements arrive or are handled in batches. Feeding these host intervals into IMU integration would distort time. Use device sample timing, with a calibrated mapping to the common clock.

**2. There is no validated absolute or cross-sensor clock synchronization.**

The snapshot explicitly records PTP and GPS PPS as unimplemented and NTP as assumed but unverified. LiDAR metadata specifies `TIME_FROM_INTERNAL_OSC`. That is a sensor-relative free-running clock, not UTC. Ouster supports externally referenced PTP or pulse synchronization, but it must be configured and its lock measured. [Ouster time synchronization](https://static.ouster.dev/sensor-docs/image_route1/image_route3/time_sync/time-sync.html)

All raw MTi UTC validity flags are zero. Those calendar fields must not be used as an absolute time reference. Its native SampleTimeFine is still useful for relative timing. GNSS validity and sample tick semantics are specified separately in the [Xsens protocol documentation, pp. 47–54](https://www.xsens.com/hubfs/Downloads/Manuals/MT_Low-Level_Documentation.pdf).

PCAP timestamps are epoch-sized with microsecond resolution. All recorded network headers use localhost addresses and zero MAC addresses, consistent with reconstructed capture headers; the source code must establish where the capture timestamps were taken. Device timestamps remain useful, but host/device timestamp pairs alone cannot separate fixed transport latency from clock offset. Their short-term variation here is not an end-to-end accuracy measurement.

**3. The structured MTi export changes the meaning of some data.**

The diagnostic HDF5 read found 71 rows with all-zero acceleration and gyro. Of these, 66 could be matched to GNSS-only raw messages that contain no inertial measurement. At least those rows encode missing data as physical zero. A consumer must not integrate them as real IMU samples. Use separate streams or explicit presence masks/null values for heterogeneous Xbus outputs.

Pressure decoding is demonstrably wrong for a matching raw/structured sample: packet counter 20994 contains `0x0001872b`, which is unsigned integer 100,139; the HDF5 stores approximately `1.403246e-40`, exactly those same bits interpreted as float32. Correct the field-specific decoder. The same protocol defines pressure in Pa; it must be decoded according to its integer representation. [Xsens protocol documentation](https://www.xsens.com/hubfs/Downloads/Manuals/MT_Low-Level_Documentation.pdf)

The HDF5 has 172 missing packet counter values across one 1.66-second device-time gap that crosses the recording start boundary. There are no counter gaps within its in-session rows. This is not evidence of continuous in-session loss: it indicates inconsistent inclusion of pre-recording data. Conversely, 18 complete raw messages at the beginning/end are absent from the HDF5. Define the intended coverage and finalize all writers after queues drain.

**4. Completion status does not currently guarantee readable, consistent artifacts.**

The session says `COMPLETED`, but the original HDF5 fails normal read-only opening because its superblock still marks it open for writing. SWMR read also fails. An in-memory copy with that flag cleared allowed inspection of all datasets; this does not establish that the original file was cleanly finalized. A copy made while the writer was active could also explain the flag. The source file was left unchanged.

Close and flush the sensor files, reopen them independently, reconcile counts, and only then publish a completed manifest. Treat recovery as a separate operation: HDF5's own flag-clearing tool is explicitly not a general corruption repair tool. [HDF Group h5clear documentation](https://support.hdfgroup.org/documentation/hdf5/latest/_h5_t_o_o_l__c_r__u_g.html)

The events report 40 dropped LiDAR frames and 14 dropped radar frames, but the native LiDAR scans and radar structured frame IDs are consecutive within their respective recorded ranges. These counters may describe another consumer or a longer application lifetime. Record counter scope, interval, stage, reason, and units; do not interpret them as proven losses from the raw archive.

**5. Calibration is incomplete for external sensor fusion.**

`calibration/camera_imu.yaml` contains identity rotation, zero translation, and `status: not_calibrated`. `calibration/extrinsics/camera_to_mti.yaml` is empty. There are no supplied external LiDAR–IMU, camera–LiDAR, or GNSS antenna lever-arm calibrations.

Camera sidecars contain one intrinsic set and a baseline, with zero distortion and no declaration of raw versus rectified image geometry. That is insufficient as a standalone stereo calibration contract; do not assume zero distortion is wrong, since the values may describe rectified images. Export both cameras' relevant intrinsics, distortion model, stereo transform, rectification/projection matrices, and camera-to-internal-IMU calibration. Check what is embedded in SVO2 before concluding it is absent from the recording.

The HDF5 identifies NED, `quaternion_qLS`, and `wxyz`, which is helpful. Still define the quaternion mapping mathematically, sensor axes, gravity convention, and units for every field. Do not assume the native LiDAR frame equals its housing frame. Avoid Python-specific YAML tuple tags for portable calibration files.

**6. Multiple recordings need explicit session isolation.**

`raw/recording.svo2`, `raw/lidar.pcap`, and `raw/mti.xbus` have fixed names. They are safe if the application creates a new session root each time. Reusing this exact root risks replacement or mixing; overwrite behavior cannot be verified without recorder code. Daily HDF5 names and date-only Parquet partitions do not independently isolate sessions.

The session table has OPEN and COMPLETED rows for one ID. That can be valid event history, but consumers need revision ordering and one authoritative final state. `frames_end=495` appears to be a row count, whereas frame IDs end at 766; distinguish counts from IDs and define whether index endpoints are inclusive or exclusive. Vehicle, route, operator, weather, notes, and calibration ID are all empty.

**Recommended structure for repeated recordings**

Keep your current formats inside one immutable directory per session. A proposed layout is:

```text
MPK_dataset/
  dataset.json                         # dataset schema and conventions
  sessions/
    20260911T112729Z_<unique-id>/
      session.json                     # authoritative finalized manifest
      config/                          # effective settings, versions, environment
      calibration/                     # immutable snapshot, IDs and hashes
        frames.yaml
        stereo.yaml
        imu.yaml
        extrinsics.yaml
      raw/
        zed/chunk_000000.svo2
        lidar/chunk_000000.pcap
        lidar/sensor_metadata.json
        mti/chunk_000000.xbus
        gnss/receiver_native.bin        # if supported by the actual receiver
        gnss/corrections.bin            # native correction stream, when needed
        radar/                         # optional
      measurements/
        imu/part_000000.parquet         # or HDF5, with equivalent schema
        gnss/part_000000.parquet
      index/
        camera.parquet
        lidar.parquet
        raw_message_locations.parquet
      timing/
        clock_observations.parquet
        clock_models.json
        sync_health.parquet
      quality/
        events.parquet
        summary.json
        files.sha256
      derived/
        <processing-run-id>/           # outputs, matching index, run provenance
```

This is a proposed design, not a restructuring already performed. The existing raw/metadata separation can be retained; the essential changes are session isolation, explicit semantics, indexes, calibration, and validation. At the current short-trial settings, the dominant raw files suggest roughly 35 MB/s or 125 GB/hour before derived outputs. Treat this only as a preliminary capacity estimate; stream windows differ and scene content changes camera size. Use bounded chunks, benchmark sustained writes with headroom, and test long runs. Batch radar outputs instead of creating indefinitely many tiny files.

**Minimum timing and replay contract**

Every measurement should carry or resolve to the following fields:

| Field | Purpose |
|---|---|
| `session_id`, `stream_id`, `sequence`, `clock_segment_id` | Unique ownership, ordering, and reset/wrap boundaries |
| `device_time_raw`, `device_time_unit`, `device_clock_id` | Preserve the original measurement clock and its resolution |
| `host_rx_monotonic_ns`, `host_rx_realtime_ns`, `host_boot_id` | Preserve receipt timing and the host clock relationship |
| `measurement_time_ns`, `common_clock_id`, `clock_model_id` | Corrected acquisition time; keep null when not established |
| `time_uncertainty_ns`, `time_reference` | Estimated timing uncertainty and event meaning: exposure, column, sample, etc. |
| `payload_file`, `payload_locator`, `frame_id` | Exact SVO position, packet offset, or HDF5 row; distinguish spatial frame ID from sample sequence |
| `valid_fields`, `quality_flags`, `calibration_id` | Prevent absent, invalid, or uncalibrated data being treated as usable |

Use signed integer nanoseconds for absolute/common time. Use original integer ticks where that is the source representation. Do not route epoch nanoseconds through float64 or JavaScript Number. Serialize large JSON timestamps as decimal strings when consumers cannot guarantee 64-bit integer preservation. Store units, UTC/GPS/TAI conversion policy, leap-second handling, coordinate reference system, and uncertainty definitions explicitly.

A clock model can use `t_common = t0_common + scale * (t_device - t0_device)` within a defined segment. Store the anchors, scale/drift, valid interval, estimation method, latency assumptions, residuals, and uncertainty. Clock resets require new segments. Never infer precision from the number of timestamp digits.

Prefer one validated GNSS-disciplined time source, with PTP or PPS distributed to devices that actually support it. For this ZED 2 USB camera, software timestamp alignment is the relevant path: USB ZED cameras do not expose the GMSL hardware-trigger mechanism. Synchronizing the host helps but does not independently prove exposure-level alignment. Record ZED IMAGE timestamps, exposure settings, and host observations; measure residual delay under representative load. [Stereolabs camera synchronization](https://www.stereolabs.com/docs/development/zed-sdk/modules/camera/multi-camera)

Keep the ZED internal IMU and external MTi as distinct streams. SVO2 supports native-rate onboard sensor data, making it a sensible archive, but verify actual extraction using the recorded SDK version. The current `sdk_version: unknown` should be replaced with the effective version. Derived depth/poses also need SDK/model/settings provenance; if exact original outputs matter, save those outputs separately. [Stereolabs SVO2 recording](https://docs.stereolabs.com/docs/development/zed-sdk/modules/camera/recording)

For replay, merge streams by corrected acquisition time using stable tie ordering and exact payload locators. Preserve all valid IMU measurements between observations; interpolate/preintegrate as the algorithm requires. Deskew LiDAR using its measurement timing and the estimated trajectory. Camera-nearest-neighbor tables are useful derived products, but should not become the only surviving data or replace full-rate IMU integration. Record signed matching residuals, unmatched reasons, and maximum permitted interpolation gaps.

Use one replay scheduler or one exported bag timeline, with a single simulated-clock authority. Respect backpressure for offline evaluation, publish original measurement times, and record any dropped samples explicitly. SVO playback can skip frames in real-time mode, so select a mode that processes every frame when measuring algorithm quality. [Stereolabs replay parameters](https://docs.stereolabs.com/docs/integrations/ros-2/zed-stereo-node)

**Accuracy budget and algorithm suitability**

Current association tolerances are 20 ms camera–IMU, 50 ms camera–LiDAR, and 200 ms camera–GNSS. These are search windows, not established synchronization accuracy. At 10 m/s, timing errors of 20, 50, and 200 ms correspond to 0.2, 0.5, and 2 metres of translation. These are illustrative calculations, not measured errors in this stationary-looking trial.

Choose the timing target from the motion and spatial error allocation: approximately `timing_error <= allowed_position_error / (speed + range * angular_speed)`, with angular speed in radians/s. For example, 2 cm at 10 m/s gives 2 ms for translation alone; 20 m range and 30 degrees/s rotation tighten that illustrative budget to about 1 ms. Calibration, measurement noise, exposure, and algorithm error still consume separate parts of the budget. Sub-millisecond timing is an engineering target to test, not a capability demonstrated here.

| Representative processing family | Consequence for this dataset |
|---|---|
| LIO-SAM | Preserve ring/channel and relative point time, correct IMU orientation/extrinsics, and provide a suitably fast IMU. The original implementation recommends at least 200 Hz; this raw MTi stream is approximately 100 Hz. [Project requirements](https://github.com/TixiaoShan/LIO-SAM) |
| FAST-LIO | Preserve per-point timing and its units, synchronized IMU data, and the defined LiDAR-to-IMU transform. Zero-filled IMU rows and receipt-time batching are inappropriate inputs. [Project requirements](https://github.com/hku-mars/FAST_LIO) |
| Stereo/VIO and visual–LiDAR fusion | Require correct image geometry, physical frames, and reliable camera/IMU timing; embedded ZED data may provide a useful path once SDK decoding is verified. Validate the exact chosen implementation separately. |
| GNSS-aided mapping or RTK evaluation | Require valid fixes, status/covariance, datum/height convention, and antenna lever arms. This trial has no usable fixes. If RTK is used in the estimator, it is not automatically independent ground truth. |
| GNSS post-processing | If PPK or tightly coupled GNSS is required, log supported raw observations and navigation data, corrections/base reference and provenance. NMEA fixes alone do not provide those observables. Scope this to the actual receiver and selected algorithms. |

**Acceptance checks before a larger collection campaign**

1. Record at least three start/stop sessions in the same application run and another after reboot. Verify unique roots, no overwrites, exact session membership, documented pre-roll, and valid clock segments.
2. Fix field-level MTi decoding and validity handling. Reconstruct structured records from raw messages and compare counters, timestamps, and values. Never pad missing inertial fields with unmarked zeros.
3. Demonstrate clean close/reopen of all containers, including SVO2 through the ZED SDK. Verify counts, final records, indexes, and hashes. Test interruption recovery on a separate test recording.
4. Capture an outdoor GNSS test with recorded fix transitions, satellites, correction receipt/age, covariance, and verified RTK state. Represent no-fix and float periods explicitly.
5. Establish and validate a complete calibration chain connecting the external IMU, LiDAR, both cameras, vehicle frame, and GNSS antenna. Store physical units, transform direction, method, date, and residuals.
6. Measure time offset, drift, and jitter over a long run with translation/rotation, load changes, and shared observable events. This 17-second trial does not establish long-term drift or dynamic alignment accuracy.
7. Decode LiDAR columns and full-rate inertial samples; compare saved versus decoded counts. Validate camera SVO position/timestamp mapping independently of application frame counters.
8. Replay twice at different speeds, including slower than real time. Confirm identical input samples, order, and timestamps at algorithm entry. Bit-identical algorithm outputs may require additional controls for threading/GPU nondeterminism.

**Deliverables and remaining limits**

`audit_metrics.json` contains measured counts, schemas, timing statistics, decoder evidence, and coverage checks. `observed_file_hashes.json` contains hashes generated during this assessment; they are not acquisition-time integrity evidence. `inspect_trial.py` records the audit method and does not repair recording files. Its diagnostic HDF5 path uses the installed Windows HDF5 library and should not be treated as a production recovery tool.

The most valuable next work is to fix the recorder's timestamp schema, MTi decoding, finalization, and session isolation, then perform a calibrated moving test with valid RTK and verified SDK replay. The native formats can remain.
