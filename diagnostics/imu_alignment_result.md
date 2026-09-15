# IMU Alignment Result

Poses: LEVEL, ROLL_POS, ROLL_NEG, PITCH_POS, PITCH_NEG, YAW_POS, YAW_NEG

## Mean Orientation per Pose

| Pose | MTi R | MTi P | MTi Y | ZED R | ZED P | ZED Y |
|------|-------|-------|-------|-------|-------|-------|
| LEVEL | +122.80 | -3.81 | +2.92 | -1.95 | +0.14 | -0.07 |
| ROLL_POS | +121.16 | +1.65 | -28.28 | -2.20 | -31.74 | +3.08 |
| ROLL_NEG | +109.30 | -4.82 | +29.71 | -2.44 | +26.19 | -3.34 |
| PITCH_POS | +140.41 | -32.00 | +2.67 | -29.68 | -0.11 | +0.73 |
| PITCH_NEG | +155.39 | +36.01 | +6.47 | +37.75 | +1.88 | -5.35 |
| YAW_POS | -166.77 | +3.13 | +3.34 | +3.97 | +1.24 | -42.65 |
| YAW_NEG | +122.50 | -1.29 | +6.87 | +0.12 | +5.06 | +41.63 |

## Estimated X (MTi→ZED body frame)

```
  [ +0.190719   -0.981614   -0.007808]
  [ +0.121087   +0.015631   +0.992519]
  [ -0.974148   -0.190237   +0.121842]
```

- Quaternion (x,y,z,w): [-0.51313875, +0.41924649, +0.47840686, +0.57623596]
- Euler (RPY): roll=-57.3615° pitch=+76.9436° yaw=+32.4115°
- Rotation angle: 109.6275°
- det(X): 1.00000000

## Residuals per Pose

| Pose | Angle | Roll | Pitch | Yaw |
|------|-------|------|-------|-----|
| ROLL_POS | +51.5193° | +35.7850° | +39.8971° | +12.4091° |
| ROLL_NEG | +46.5814° | -31.8711° | -35.9474° | +11.7799° |
| PITCH_POS | +32.4688° | +9.7867° | +28.5598° | -9.9301° |
| PITCH_NEG | +33.6223° | -0.2041° | -31.4132° | -12.0812° |
| YAW_POS | +28.9366° | +9.0198° | +10.0432° | -24.8766° |
| YAW_NEG | +43.3333° | -0.5028° | -7.9078° | -42.6023° |

## Summary

- RMS residual: 40.2572°
- Max residual: 51.5193°
- Per-axis max |residual|: Roll=35.7850° Pitch=39.8971° Yaw=42.6023°
- Verdict: **FAIL**
- Max residual 51.52° is too large. Check sensor mounting, quaternion conventions, or data quality.
