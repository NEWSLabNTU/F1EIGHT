# Sensor Configuration Guide

This guide covers sensor configuration options for AutoSDV.

## Sensor Suites (Recommended)

Predefined sensor suites provide convenient configurations for common hardware combinations:

```bash
# Robin-W LiDAR + ZED X Mini + ZED IMU (integrated)
ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=robin_zed

# Robin-W LiDAR + ZED X Mini + MPU9250 IMU
ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=robin_zed_mpu

# Velodyne VLP-32C + ZED X Mini + ZED IMU (integrated)
ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=vlp32c_zed

# Velodyne VLP-32C + ZED X Mini + MPU9250 IMU
ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=vlp32c_zed_mpu

# Velodyne VLP-32C + ZED IMU only (no camera streams, lightweight)
ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=vlp32c_zed_imu

# Blickfeld Cube1 + USB Cameras + MPU9250 IMU
ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=cube1_usb

# Custom configuration (use individual parameters)
ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=custom lidar_model:=robin-w camera_model:=zedxm imu_source:=mpu9250
```

### Suite Configuration Matrix

| Suite | LiDAR | Camera | IMU | GNSS | ZED OD |
|-------|-------|--------|-----|------|--------|
| robin_zed | Robin-W | ZED X Mini | ZED built-in | u-blox | Yes |
| robin_zed_mpu | Robin-W | ZED X Mini | MPU9250 | u-blox | Yes |
| vlp32c_zed | Velodyne 32C | ZED X Mini | ZED built-in | u-blox | Yes |
| vlp32c_zed_mpu | Velodyne 32C | ZED X Mini | MPU9250 | u-blox | Yes |
| vlp32c_zed_imu | Velodyne 32C | None | ZED built-in | u-blox | No |
| cube1_usb | Cube1 | USB cameras | MPU9250 | u-blox | No |
| custom | (manual) | (manual) | (manual) | (manual) | (manual) |

### Overriding Suite Defaults

```bash
# Use robin_zed suite but disable GNSS for indoor testing
ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=robin_zed use_gnss:=false

# Use vlp32c_zed suite but switch to Septentrio GNSS
ros2 launch autosdv_launch autosdv.launch.yaml sensor_suite:=vlp32c_zed gnss_receiver:=septentrio
```

## Individual Sensor Parameters

When using `sensor_suite:=custom` (default), configure sensors individually:

### IMU Sources

```bash
# MPU9250 external I2C IMU (default for custom suite)
ros2 launch autosdv_launch autosdv.launch.yaml imu_source:=mpu9250

# ZED camera built-in IMU (factory-calibrated, more stable)
ros2 launch autosdv_launch autosdv.launch.yaml imu_source:=zed camera_model:=zedxm
```

**Note:** When using ZED IMU (`imu_source:=zed`), the ZED camera must be used (`camera_model:=zedxm`). The system automatically relays IMU data from the camera node to avoid launching duplicate ZED drivers.

### LiDAR Models

```bash
just launch "lidar_model:=robin-w"   # Robin-W Solid-State (default)
just launch "lidar_model:=vlp32c"    # Velodyne VLP-32C
just launch "lidar_model:=vlp16"     # Velodyne VLP-16
just launch "lidar_model:=cube1"     # Blickfeld Cube1
```

The VLP-16 driver publishes in the `vlp16` frame. The sensor kit defines a
fixed transform from `sensor_kit_base_link` to `vlp16` in
`src/sensor_kit/autosdv_sensor_kit_launch/autosdv_sensor_kit_description/urdf/sensor_kit.xacro`.
For the default vehicle, adjust the `sensor_kit_base_link.vlp16` entry in
`src/param/autoware_individual_params/individual_params/config/default/autosdv_sensor_kit/sensor_kit_calibration.yaml`.
Translation (`x`, `y`, `z`) is in metres; rotation (`roll`, `pitch`, `yaw`) is
in radians. The current values copy the VLP-32C pose: zero translation, roll,
and pitch, with yaw `-0.2210`. Replace these initial values with the measured
VLP-16 mounting pose.

The description package's `config/sensor_kit_calibration.yaml` also contains
a VLP-16 entry, with an all-zero pose. Normal vehicle launch loads the
`individual_params` calibration from `config/<vehicle_id>/<sensor_model>/`,
so edit that vehicle's configuration and restart the launch to apply it.

### Camera Models

```bash
just launch "camera_model:=zedxm"    # ZED stereo camera (default)
just launch "camera_model:=usb"      # USB cameras
just launch "camera_model:=none"     # No camera
```

### GNSS Receivers

```bash
just launch "gnss_receiver:=garmin"      # Garmin (default)
just launch "gnss_receiver:=ublox"       # u-blox
just launch "gnss_receiver:=septentrio"  # Septentrio
```

## NTRIP/RTK Configuration (u-blox only)

AutoSDV supports RTK positioning for centimeter-level accuracy using NTRIP.

### Hardware Requirements

- u-blox ZED-F9R GNSS receiver (e.g., SimpleRTK2B Fusion board)
- Connected via USB (udev rules create `/dev/ublox-gps` symlink)
- Clear sky view for optimal satellite reception

### NTRIP Service

**Default**: e-GNSS Taiwan VRS (Virtual Reference Station)
- Server: 210.241.63.193:81
- Mountpoint: Taiwan
- Credentials: `src/sensor_kit/autosdv_sensor_kit_launch/launch/ntrip.launch.xml`

### Enable NTRIP

```bash
# Basic RTK setup with u-blox + NTRIP
just launch "gnss_receiver:=ublox use_ntrip:=true"

# Full outdoor autonomous setup with RTK
just launch "gnss_receiver:=ublox use_ntrip:=true lidar_model:=robin-w camera_model:=zedxm"
```

### How NTRIP Works

1. NTRIP client connects to e-GNSS Taiwan VRS server
2. Client subscribes to `/sensing/gnss/ublox/nmea_sentence` (rover position)
3. Server sends RTCM corrections via `/sensing/gnss/ntrip/rtcm` topic
4. u-blox driver applies corrections to achieve RTK fix
5. Output: cm-level accuracy on `/sensing/gnss/ublox/nav_sat_fix`

### Monitor RTK Status

```bash
ros2 topic hz /sensing/gnss/ntrip/rtcm              # RTCM corrections
ros2 topic echo /sensing/gnss/ublox/nmea_sentence   # NMEA sentences
ros2 topic echo /sensing/gnss/ublox/nav_sat_fix     # RTK fix quality
ros2 topic echo /sensing/gnss/ublox/rxmrtcm         # RTCM reception status
```

### Configuration Files

- NTRIP client: `src/sensor_kit/autosdv_sensor_kit_launch/launch/ntrip.launch.xml`
- ZED-F9R config: `src/sensor_kit/autosdv_sensor_kit_launch/config/zed_f9r_rover.yaml`

### Troubleshooting

- **No RTCM data**: Check internet connection and e-GNSS credentials
- **Poor accuracy**: Ensure clear sky view (avoid buildings/trees)
- **Recommended location**: Open areas like NTU College of Social Sciences

## Indoor Operation (No GPS)

For indoor testing without GNSS:

```bash
just launch "use_gnss:=false"
```

When running indoors:
1. The system uses NDT localization instead of GNSS
2. Use RViz's "2D Pose Estimate" tool to set initial vehicle position
3. Click and drag on the map to set pose and orientation
4. The `/initialpose` topic receives the manual pose input

## Localization Sources

### CUDA NDT Scan Matching (Default)

```bash
just launch  # cuda_ndt is the default pose_source
# Or explicitly: just launch "pose_source:=cuda_ndt"
# Fallback to Autoware NDT: just launch "pose_source:=ndt"
```

- CUDA-accelerated NDT (1.3-1.6x faster, 57% less CPU on Jetson)
- Tuned for VLP-32C LiDAR on COSS map (NTU Campus)
- Resolution: 2.0m, Score threshold: 2.2, Voxel size: 0.5m, Points: 3000
- See `docs/research/localization/ndt_parameter_tuning_coss_map.md` for tuning details
