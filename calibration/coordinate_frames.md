# Coordinate Frames — ZED 2 + MTi-8

## ZED 2 Camera Optical Frame

The rectified camera frame used by the ZED SDK:

```
        Y (down)
        |
        |
        +------ X (right)
       /
      /
     Z (forward, into scene)
```

- X: right
- Y: down
- Z: forward (optical axis)

This is the frame in which `zed_intrinsics.yaml` is expressed.

## MTi-8 Body Frame

Default Xsens sensor body frame (label facing up):

```
        Z (up)
        |
        |
        +------ Y (left)
       /
      /
     X (forward)
```

- X: forward (out the top of the sensor, along the long axis)
- Y: left
- Z: up

The quaternion from the MTi-8 represents rotation from this body frame to the Earth reference frame (NED: North-East-Down).

## Transform to Estimate

```
T_camera ← imu

[ R | t ]
[ 0 | 1 ]
```

- **R**: 3×3 rotation that aligns MTi-8 body axes with camera optical axes
- **t**: 3×1 translation from IMU center to camera optical center (in camera frame)

After calibration, a point `p_imu` in the MTi-8 frame maps to the camera frame as:

```
p_camera = R @ p_imu + t
```

## Axis Mapping Sanity Check

When the MTi-8 is mounted flat (label up) below the camera:

| Camera axis | MTi-8 axis | Expected rotation |
| ----------- | ---------- | ----------------- |
| X (right)   | -Y (right) | ~90° around Z     |
| Y (down)    | -Z (down)  | ~90° around X     |
| Z (forward) | X (forward)| ~0°               |

The actual rotation from calibration should be close to this if the mounting is roughly as expected.
