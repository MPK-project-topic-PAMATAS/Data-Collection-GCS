# IMU Coordinate Frame Conventions

## MTi-8 (XSens)

Source: `mti8/frame.py:9`, `mti8/mtdata2_decoder.py:94-95`

### Quaternion
- **Component order:** (x, y, z, w) — XSens Hamilton convention
- **Byte order:** big-endian, decoded as `struct.unpack(">4f", raw[:16])`
- **Direction:** Rotation from **body frame → navigation frame**
- **Normalization:** Always unit quaternion (|q| = 1.0)

### Body Frame (sensor-fixed, right-handed ENU)
- **X:** Right (when sensor label faces up, connector faces forward)
- **Y:** Forward
- **Z:** Up (out of sensor label face)

### Navigation Frame
- **ENU** (East-North-Up) by default in MT Manager
- Local-level frame, magnetometer-aided heading
- Gravity acts along **-Z** in navigation frame

### Euler Convention
- **Order:** Roll (X) → Pitch (Y) → Yaw (Z)
- **Convention:** Tait-Bryan angles, intrinsic rotations
- Stored in `MTiFrame.euler` as `(roll, pitch, yaw)` in degrees
- Derived from quaternion by `dashboard/main_window.py:273-284`

### Accelerometer
- **XDI ID:** 0x4020 (`DI_ACCELERATION`)
- **Components:** (x, y, z) in m/s², body frame
- **Gravity:** Stationary ≈ [0, 0, +9.81] (Z-up body frame)

### Gyroscope
- **XDI ID:** 0x8020 (`DI_RATE_OF_TURN`)
- **Components:** (x, y, z) in rad/s, body frame

---

## ZED (Stereo Labs)

Source: `camera/zed_camera.py:231-261`, `config/settings.py:10`

### Quaternion
- **Component order:** (x, y, z, w) — from `imu.get_pose().get_orientation().get()`
- **Direction:** Rotation from **body frame → world frame**
- **World frame:** VIO-initialized reference frame (ZED starts at identity)

### Coordinate System (from settings.py)
```
sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP
```
- **X:** Right
- **Y:** Forward
- **Z:** Up

### Camera Frame (raw, before coordinate_system transform)
- **X:** Right
- **Y:** Down
- **Z:** Forward (into scene)

**Note:** The `coordinate_system` parameter affects depth/measurement data, NOT the IMU orientation quaternion. The IMU quaternion is always in the ZED VIO body frame.

### Body Frame (IMU / VIO, right-handed)
- **X:** Right
- **Y:** Forward
- **Z:** Up

### Euler Convention
- Derived from quaternion by `dashboard/main_window.py:273-284` (same function as MTi)
- **Order:** Roll (X) → Pitch (Y) → Yaw (Z)
- **Convention:** Tait-Bryan angles

### Accelerometer
- From `imu.get_linear_acceleration()` — linear acceleration (gravity removed by SDK)
- **Components:** (x, y, z) in m/s², body frame
- **Gravity:** Stationary ≈ [0, 0, +9.81] (if gravity not removed) or ≈ [0, 0, 0] (if removed)

**CRITICAL:** ZED's `get_linear_acceleration()` may or may not include gravity depending on SDK version. The raw accelerometer includes gravity. Verify from data.

### Gyroscope
- From `imu.get_angular_velocity()`
- **Components:** (x, y, z) in rad/s, body frame

---

## Summary Comparison

| Property | MTi-8 | ZED |
|----------|-------|-----|
| Quaternion order | (x, y, z, w) | (x, y, z, w) |
| Direction | body → nav | body → world |
| Body X | Right | Right |
| Body Y | Forward | Forward |
| Body Z | Up | Up |
| Handedness | Right | Right |
| Nav/world frame | ENU (magnetometer-aided) | VIO-initialized (no mag) |
| Gravity in body | [0, 0, +9.81] | [0, 0, +9.81] |
| Euler convention | Tait-Bryan (RPY) | Tait-Bryan (RPY) |

### Key Insight
Both sensors use the **same body frame convention** (X-right, Y-forward, Z-up, right-handed). The quaternion ordering is identical. The **disagreement** therefore comes from:

1. **World frame definition:** MTi uses magnetometer-aided ENU, ZED uses VIO-initialized frame
2. **Physical mounting offset:** Even "aligned" sensors have residual rotation between them
3. **Heading reference:** MTi has magnetic heading, ZED has visual heading — these diverge

The relative rotation R_rel = R_zed^T @ R_mti should be **approximately constant** if both sensors are rigidly mounted and stationary, because the body frames are (nearly) aligned.
