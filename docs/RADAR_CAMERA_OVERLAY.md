# Radar-on-Camera Overlay — Extrinsic

`radar_camera_demo.py` and `dashboard/radar_panel.py`'s `RadarCameraOverlay`
project AWR1843 radar points onto the ZED camera image using the camera's
own intrinsics (`fx`, `fy`, `cx`, `cy` from `camera.calib`, populated by
`ZEDCamera.open()`), via the shared `radar/projection.py`:

```
dx, dy, dz = config.settings.RADAR_TO_CAMERA_OFFSET_M   # radar->camera lever arm, radar frame
Xc, Yc, Zc = (p.x - dx), -(p.z - dz), (p.y - dy)         # -> pinhole (x=right, y=down, z=forward)
u = cx + fx * Xc / Zc
v = cy + fy * Yc / Zc
```

`RADAR_TO_CAMERA_OFFSET_M` defaults to `(0.0, 0.0, 0.0)` — the radar and
camera treated as **co-located** — which is a fine approximation at a
meter or more, but the overlay visibly drifts the closer an object gets,
since the real mounting gap (centimeters) becomes proportionally larger
relative to distance. There is still no measured camera↔radar extrinsic
recorded anywhere in this repo — only `calibration/extrinsics/camera_to_mti.yaml`,
`camera_to_vehicle.yaml`, and `gnss_to_vehicle.yaml` exist, and none of
those three are actually read by any code (`validate_dataset.py` and
`dataset/writer.py` only check/copy them as placeholders) — so measuring
and setting `RADAR_TO_CAMERA_OFFSET_M` is the only way to make this
overlay more accurate than co-location right now.

## How to measure it

With calipers or a ruler, measure from the AWR1843's **antenna array
phase center** — the patch-antenna region on the BOOST board (the
tan/gold section near the top of the PCB, *not* the board edge, a mounting
screw, or the SMA/connector end) — to the ZED's **left** lens optical
center (the left aperture; `camera.calib`'s `fx/fy/cx/cy` are the left
eye's, not the stereo midpoint). Express the result in the radar's own
frame (x=right, y=forward, z=up), meters:

- `dx`: camera left-lens position minus radar position, left/right (+ if
  the camera lens is to the radar's right)
- `dy`: same, forward/back (+ if the camera lens is further from the
  scene than the radar, i.e. set back)
- `dz`: same, up/down (+ if the camera lens is higher than the radar)

Then set `RADAR_TO_CAMERA_OFFSET_M = (dx, dy, dz)` in `config/settings.py`.
Re-measure and update it after every physical remount — it's only valid
for the exact mounting geometry it was measured on.

## Reference: how radar-lab/ti_mmwave_rospkg does it

Looked at `https://github.com/radar-lab/ti_mmwave_rospkg` (a local clone
already exists on this machine under
`/media/vgtu/ssd11/mmwave_ws/src/mmwave_ti_ros`, specifically
`ros1_driver/src/ti_mmwave_rospkg/launch/camera_overlay.launch`). Their
"camera overlay" isn't custom projection code — it's RViz's built-in
`Camera` display doing the projection automatically, driven by a ROS `tf`
static transform that someone measured by hand on their specific rig:

```xml
<node pkg="tf" type="static_transform_publisher" name="camera_baselink"
      args="-2 0 .95 -1.5707963267948966 0 -1.5707963267948966 ti_mmwave_pcl usb_webcam 100"/>
```

That's `x=-2m, y=0, z=0.95m` translation plus a rotation, from the radar's
frame to the camera's frame. RViz combines that transform with the
camera's intrinsics (a ROS `camera_info` YAML — same idea as our
`camera.calib`) to project the radar point cloud into the live image.

This confirms our projection math is the same fundamental approach they
use. The only thing missing here is the measured offset they have and we
don't.

## To do later

1. ~~Measure the actual mounting offset~~ Done:
   `RADAR_TO_CAMERA_OFFSET_M = (-0.055, 0.004, -0.015)` in `config/settings.py`
   (5.5cm lateral, 0.4cm depth, 1.5cm vertical, from ruler/sketch
   measurements). `dy`'s sign is an assumption (camera lens forward of the
   radar PCB, going by the earlier rig photo) rather than an independent
   measurement — its magnitude (0.3-0.5cm) is small next to dx/dz either
   way, but re-check with calipers if close-range precision matters more
   later.
2. ~~Factor the projection into one shared function.~~ Done:
   `radar/projection.py`'s `project_point()`, used by both
   `radar_camera_demo.py` and `dashboard/radar_panel.py`.
3. If the two sensors also aren't boresight-aligned (camera pointed a
   meaningfully different direction than the radar), a rotation would be
   needed too, the same way `ti_mmwave_rospkg`'s static transform has
   one — but for this rig they're mounted parallel, so translation alone
   is expected to be enough.
