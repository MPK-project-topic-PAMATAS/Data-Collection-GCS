# Rig Measurements — ZED 2 + MTi-8

## Sensor Specifications

| Sensor | ZED 2 | MTi-8 |
|--------|-------|-------|
| Serial | 24043017 | MTi-8-2A8G4 |
| Resolution | 1280×720 | N/A |
| FPS | 30 | 200 Hz |
| Baseline | 119.6 mm | N/A |
| Coordinate frame | X-right, Y-down, Z-forward | X-forward, Y-left, Z-up |

## Physical Setup

```
Front View

    +---------------------+
    |                     |
    |       ZED 2         |
    |                     |
    +---------------------+
              |
              |  d_height
              |
         +---------+
         |  MTi-8  |
         +---------+


Top View

         ZED 2
    +---------------+
    |   [L]   [R]   |
    +---------------+
         |
         |  d_forward
         |
       MTi-8
```

## Measurements

| Parameter          | Value  | Unit | Verified |
| ------------------ | ------ | ---- | -------- |
| d_forward          |        | mm   | [ ]      |
| d_height           |        | mm   | [ ]      |
| d_lateral          |        | mm   | [ ]      |

* **d_forward**: Distance from IMU center to camera optical center along the camera Z axis (forward). Positive = IMU is behind the camera.
* **d_height**: Distance from IMU center to camera optical center along the camera Y axis (down). Positive = IMU is below the camera.
* **d_lateral**: Distance from IMU center to camera optical center along the camera X axis (right). Positive = IMU is to the right of the camera center.

## MTi-8 Mounting Orientation

- [ ] Flat (label facing up)
- [ ] Upside down (label facing down)
- [ ] Rotated (describe below)

Rotation about vertical axis (if any): ______ degrees

## Mounting Method

- [ ] Rigid bracket (aluminum/steel)
- [ ] 3D printed mount
- [ ] Zip ties (not recommended for calibration)
- [ ] Other: _______________

**Critical:** The relative pose between ZED 2 and MTi-8 must never change after calibration. Use rigid mounting. Avoid flexible brackets, loose screws, or anything that allows movement.

## Calibration Target

- [ ] AprilGrid 6×6 printed
- [ ] Tag size verified: 80 mm (measured with ruler)
- [ ] Board is flat (no curling)
- [ ] Mounted on rigid surface

## Calibration Recording Details

| Parameter | Value |
|-----------|-------|
| Duration | 3 minutes |
| Motion | roll, pitch, yaw, translation, figure-eight |
| Distance to target | 0.5–3 m |
| Date | |
| Operator | |

## Notes
