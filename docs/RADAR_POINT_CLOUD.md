# Radar Point Cloud — Object Size, Clustering, and Elevation

Reference notes for later work on the AWR1843BOOST radar pipeline
(`radar/`, `radar_camera_demo.py`, `dashboard/radar_panel.py`).

## Object width/height are not available from the sensor

The AWR1843's `MMWDEMO_OUTPUT_MSG_DETECTED_POINTS` TLV (decoded in
`radar/parser.py`, `_TLV_DETECTED_POINTS`) gives each detected point's
range, Doppler velocity, and 3D coordinates (x, y, z) — see
`radar/frame.py`'s `RadarPoint`. Each point is one radar reflection
(scatter point) off some part of an object's surface, not a full object
contour, so **no width/height/bounding-box comes from the sensor itself**.

A single physical object typically returns **multiple points per frame**
(one per reflecting surface patch), varying with the object's reflectivity
and distance — this is why `RadarFrame.points` is a list, not a single
point per object.

To estimate object size/shape, this would need to happen at the
application layer, on the host, as a post-processing step over one
frame's point cloud:
- Cluster points into groups likely belonging to the same object (e.g.
  DBSCAN or K-means on x/y/z, possibly using Doppler velocity as an
  additional clustering dimension to separate objects moving differently
  even if spatially close).
- Estimate each cluster's approximate extent (bounding box or convex
  hull) from its member points.

Not implemented anywhere in this repo yet — `radar_camera_demo.py` and
`dashboard/radar_panel.py`'s `RadarCameraOverlay` currently draw every
point individually, with no clustering/grouping step.

More detail: the mmWave SDK's own module documentation
(`mmwave_sdk_module_documentation.html`, shipped in the SDK's `docs/`
folder — locally at
`/media/vgtu/ssd11/vgtu/ti/mmwave_sdk_03_04_00_03/` on this machine).

## Elevation (3TX+4RX) is real, confirmed

Confirmed (independently, twice — once by live testing here, once by an
external source): the AWR1843 has 3 TX antennas + 4 RX antennas, and one
TX antenna (TX1) sits physically offset **vertically** from the other two
(TX0, TX2, which share the same vertical position and are used together
for higher-resolution azimuth). Enabling all 3 TX antennas (not just
TX0+TX2) is what's needed for the radar to estimate elevation (z) at all
— see `docs/RADAR_CAMERA_OVERLAY.md` isn't the right place for this, but
related: this project's radar was briefly switched to a 3TX config to get
real z values (confirmed working — z varied meaningfully instead of being
hard-zeroed), then reverted because:
1. This exact firmware build could not combine the 3TX chirp table with
   the range-Doppler heatmap output (`guiMonitor`'s heatmap flag) —
   enabling both crashed `sensorStart` with a firmware exception and
   wedged the CLI, needing a physical unplug/replug to recover.
2. With heatmap disabled and 3TX enabled, detections were noticeably
   noisier/less consistent than the original 2TX config on a static
   scene (points/frame varied 1–9 for a scene where nothing moved,
   closest-point x/y/z each had ~0.5 m std-dev) — most likely because
   `cfarCfg`'s detection thresholds were never retuned for the 3TX
   profile's different noise floor, not because 3TX elevation is
   inherently unusable.

Net: elevation is achievable on this hardware and the antenna
config to get it is already known-good (see git history on
`radar/receiver.py` around that experiment, or the backed-up commands if
still recoverable from history) — the remaining work if revisited is
**retuning `cfarCfg`** (and possibly `profileCfg`/`frameCfg` timing) for
the 3TX profile's noise floor, ideally validated against a static-scene
jitter measurement like the one done during that experiment, before
trusting the elevation data for anything beyond a demo. That work was not
finished — reverted to the original 2TX+heatmap config, current state.

### Official confirmation: TI EVM user's guide

`SPRUIM4B` (Rev. B), "xWR1843 Evaluation Module (xWR1843BOOST) Single-Chip
mmWave Sensing Solution User's Guide" — local copy at
`/home/vgtu/Downloads/spruim4b.pdf`, section 2.7 "Antenna" (page 11):

> "The BoosterPack includes onboard-etched antennas for the four
> receivers and three transmitters that enable tracking multiple objects
> with their distance and angle information. **This antenna design
> enables estimation of distance and elevation angle that enables object
> detection in a three-dimensional plane.**"

Figure 10 in that section (a photo of the actual antenna PCB) shows the 4
RX antennas in one row and the 3 TX antennas split across **two
different rows** — one TX antenna at the same row as the RX array, the
other two together in a row offset above it (labeled with `λ`/`λ/2`
spacing). That vertical split between TX antenna rows is what makes
elevation estimation possible — matching what this project found
empirically: the original 2TX config (only the two co-planar TX
antennas) always gave z=0; enabling all three TX gave real, varying z.

The exact silkscreen-label-to-SDK-antenna-index mapping (which physical
TX is SDK `TX0`/`TX1`/`TX2` in `channelCfg`/`chirpCfg`) isn't confidently
resolved from the photo alone, but doesn't need to be — the working 3TX
chirp config from the earlier live experiment (channelCfg txChannelEn=7,
three chirps enabling TX0/TX2/TX1 in that order, matching the verified
`1843_3d.cfg` reference from `docs/RADAR_CAMERA_OVERLAY.md`) already
empirically confirmed which combination works on this exact board.

Also relevant from the same guide, section 2.7: antenna peak gain is
>10.5 dBi across 76–81 GHz; horizontal 3dB-beamwidth ≈ ±28°, elevation
3dB-beamwidth ≈ ±14° (at 78 GHz) — elevation's much narrower beamwidth is
consistent with elevation position estimates being noisier/less reliable
than azimuth, independent of the CFAR-tuning issue found during the live
3TX experiment above.

The xWR1843BOOST schematic/assembly package (TI doc `sprr370d`, RevC/RevD
— local copy at `/media/vgtu/ssd1/vgtu/Downloads/sprr370d/`) confirms the
same board electrically (cover sheet block diagram shows "PCB ANTENNAS:
4RX 3TX" wired directly to the AWR1843 chip, matching this project's
actual radar), but schematics only show circuit connections, not physical
antenna placement — the antenna row/offset geometry needed the EVM user's
guide's photo (above), not the schematic.
