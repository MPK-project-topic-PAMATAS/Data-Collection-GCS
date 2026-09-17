# OUSTER LIDAR DASHBOARD

Real-time visualization application for Ouster LiDAR sensor data.

## Quick Start

### Live Sensor
```bash
./launch_dashboard.sh --live 169.254.197.185
```

Or directly:
```bash
python3 dashboard.py --live 169.254.197.185
```

### PCAP Recording
```bash
./launch_dashboard.sh --pcap data.pcap --metadata config/sensor.json
```

Or directly:
```bash
python3 dashboard.py --pcap data.pcap --metadata config/sensor.json
```

## Features

### Display Modes
- **2D Range Image** - Shows beam channels vs azimuth with color-coded range
- **3D Point Cloud** - Interactive 3D visualization of LiDAR points

### Channel Switching
- **RANGE** - Distance measurements (primary mode)
- **SIGNAL** - Signal intensity
- **REFLECTIVITY** - Material reflectance

Switch channels using:
- Click the buttons at the bottom
- Press keys 1, 2, or 3 on keyboard

### Status Bar
Real-time information:
- Scan ID
- Active channel
- Point count
- Valid measurement percentage
- Display FPS
- Current time

### Interactive Controls

**Mouse:**
- Drag to rotate 3D view
- Scroll to zoom in/out
- Right-click for menu

**Keyboard:**
- `1` - RANGE channel
- `2` - SIGNAL channel
- `3` - REFLECTIVITY channel
- `R` - Reset 3D camera view
- `Q` / `ESC` - Exit dashboard

### Buttons
- **RANGE** - Switch to range visualization
- **SIGNAL** - Switch to signal visualization
- **REFLECTIVITY** - Switch to reflectivity visualization
- **RESET VIEW** - Reset 3D camera to default angle
- **EXIT** - Close dashboard

## Requirements

Python 3.6+ with:
```bash
pip install matplotlib numpy
```

Or install automatically:
```bash
./launch_dashboard.sh --live 192.168.1.100
```

## Architecture

```
Ouster Sensor
     ↓
C++ Backend (ouster_diaognoise)
     ↓
Data Acquisition
     ↓
Dashboard.py
     ↓
Matplotlib Visualization
```

The dashboard:
1. Launches the diagnostic C++ backend
2. Receives real-time LiDAR data
3. Displays 2D and 3D visualizations
4. Updates continuously from live sensor

## Data Flow

1. **Acquisition**: C++ backend receives UDP packets from Ouster sensor
2. **Processing**: Packets are reconstructed into complete scans
3. **Channel Extraction**: RANGE, SIGNAL, REFLECTIVITY channels extracted
4. **Visualization**: Python dashboard renders both 2D and 3D views
5. **Display**: Updated every ~200ms from latest sensor data

## Troubleshooting

### No data showing
- Check sensor IP is correct: `ping 169.254.197.185`
- Verify sensor is powered on and connected
- Check firewall isn't blocking UDP ports 7502/7503

### Slow FPS
- Close other applications
- Reduce number of displayed points (built into dash board)
- Use PCAP mode instead of live for testing

### Connection timeout
- Ensure sensor is on same network
- Check USB/Ethernet connection
- Try default IP: `169.254.197.185`

## Tips

1. **Rotate 3D View**: Click and drag in the 3D plot area
2. **Zoom**: Scroll wheel in 3D plot area  
3. **Reset View**: Press `R` or click "RESET VIEW"
4. **Switch Channels**: Press `1`, `2`, `3` or click buttons
5. **Full Screen**: Double-click window title bar

## Performance

- **Update Rate**: ~5 Hz (200ms per frame)
- **Point Cloud**: 33,000+ points per scan
- **Memory**: ~200-300 MB while running
- **CPU**: Minimal (visualization thread only)

## What You're Seeing

The dashboard shows **real, live LiDAR data**:

- **2D Image**: Range values color-coded from black (invalid) to white (far)
- **3D Cloud**: Each point is a valid LiDAR measurement
- **Channel Data**: Raw sensor outputs with no simulation

Objects in the scene appear as:
- **Near objects**: Bright colors (short range)
- **Far objects**: Dark colors (long range)
- **Invalid**: Black (no return signal)

## Architecture Note

This dashboard is built on top of the `ouster_diaognoise` diagnostic tool, which handles:
- Sensor connection
- Metadata retrieval  
- Packet acquisition
- Scan reconstruction
- Channel extraction

The dashboard provides the visual interface for this data.

## Files

- `dashboard.py` - Main dashboard application
- `launch_dashboard.sh` - Convenient launcher script
- `DASHBOARD.md` - This documentation
- `ouster_diaognoise/` - Diagnostic backend (C++)

## Future Enhancements

Potential additions:
- Point filtering and clustering
- Measurement recording
- Motion visualization
- Intensity/reflectivity statistics
- Real-time point cloud export
- Multi-sensor support
