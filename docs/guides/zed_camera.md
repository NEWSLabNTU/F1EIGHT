# ZED Camera Integration

This guide covers ZED camera setup, configuration, and troubleshooting.

## Version Information

| Component | Version |
|-----------|---------|
| ZED SDK | 5.4.1 |
| ZED Link Duo Driver | 1.3.2 for L4T 36.3.0 |
| ZED ROS2 Wrapper | 5.4.1 (our `ntust-workshop` branch, rebased onto `v5.4.1`) |
| Supported models | ZED, ZED M, ZED 2, ZED 2i, ZED X, ZED X Mini, ZED X Nano |

Source: `src/sensor_component/external/zed-ros2-wrapper/`, versions in
`versions.yaml` under `zed:`.

**The SDK and the wrapper are one version, not two.** `zed_components` compiles
against the installed SDK's headers, so a wrapper built for 5.1 against an SDK
at 5.4 is a build or a runtime failure, not a degraded mode. Bump both or
neither.

**The SDK is installed by hand.** Stereolabs publishes no apt repository, so
`setup.sh`'s `zed-sdk` step checks the installed version and prints the download
for this machine -- again, highlighted, at the end of the run -- rather than
pretending to install it:

```bash
./setup.sh --run --only zed-sdk --yes    # what is installed, and what to fetch
```

| machine | installer |
|---|---|
| amd64, Ubuntu 22.04, CUDA 12 | <https://download.stereolabs.com/zedsdk/5.4/cu12/ubuntu22> |
| Jetson, L4T 36.4 (JetPack 6.0/6.1) | <https://download.stereolabs.com/zedsdk/5.4/l4t36.4/jetsons> |
| Jetson, L4T 36.5 | <https://download.stereolabs.com/zedsdk/5.4/l4t36.5/jetsons> |

Those are redirects Stereolabs keeps stable; each resolves to a CDN file whose
name carries the patch version, so bookmark the redirect and not the file.

**On amd64 the installer brings TensorRT 10.9**
(`ZED_SDK_Ubuntu22_cuda12.8_tensorrt10.9_v5.4.1`), while Autoware's perception
engines require 10.8 exactly. Both can be installed: the `tensorrt-runtime`
setup step puts Autoware's TensorRT in a private prefix that
`scripts/trt-runtime-env.sh` places ahead of the system one for AutoSDV
processes only. This is why that step exists rather than downgrading the system
libraries -- doing that would break the ZED SDK. See
`versions.yaml` (`nvidia_amd64.tensorrt_engine_abi`) and
`docs/roadmap/11-engine-file-delivery.md`.

**`zed_msgs` comes from apt** (`ros-humble-zed-msgs`), which currently publishes
5.3.0 while the wrapper is 5.4.1. The workspace builds against it; if a future
wrapper needs a message this package does not have, that is the first place to
look.

**Without the SDK, nothing else breaks.** `zed_components` reports
`Skipping zed_components: missing the ZED SDK` and returns, `zed_wrapper` still
builds (it carries the URDF the simulation TF path xacros), and the rest of the
workspace is unaffected.

## Launch File Namespace Handling

**Important**: The ZED Python launch file (`zed_camera.launch.py`) does NOT respect XML `<push-ros-namespace>` directives.

**Solution**: Load the ZED composable node directly via XML's `<load_composable_node>`:

```xml
<!-- Container with ABSOLUTE namespace -->
<node_container pkg="rclcpp_components" exec="component_container_isolated"
                name="zed_container" namespace="/sensing/camera/$(var camera_name)">
  <param name="use_multi_threaded_executor" value="true"/>
</node_container>

<!-- Load composable node -->
<load_composable_node target="/sensing/camera/$(var camera_name)/zed_container">
  <composable_node pkg="zed_components" plugin="stereolabs::ZedCamera"
                   name="$(var camera_name)" namespace="camera">
    <param from="$(find-pkg-share zed_wrapper)/config/common_stereo.yaml"/>
    <param from="$(find-pkg-share zed_wrapper)/config/$(var camera_model).yaml"/>
  </composable_node>
</load_composable_node>
```

This creates:
- Container at `/sensing/camera/zedxm/zed_container`
- ZED node at `/sensing/camera/zedxm`
- Topics at `/sensing/camera/zedxm/<topic>`

## Topic Structure

| Topic | Purpose |
|-------|---------|
| `/sensing/camera/zedxm` | ZED node |
| `/sensing/camera/zedxm/obj_det/objects` | ZED detected objects |
| `/perception/object_recognition/detection/camera_objects` | Autoware format objects |
| `/sensing/camera/zedxm/point_cloud/cloud_registered` | Colored point cloud |
| `/sensing/camera/zedxm/imu/data` | IMU data |

## Object Detection

Two operation modes:
1. **Normal mode** (default): ZED publishes colored point cloud
2. **Object detection mode**: ZED performs detection, converts to Autoware format

### Enable Object Detection

```bash
just launch "enable_zed_object_detection:=true"
```

### Configuration Files

- Launch: `autosdv_sensor_kit_launch/launch/zed_with_object_detection.launch.xml`
- Config: `autosdv_sensor_kit_launch/config/zed_object_detection.yaml`

## Troubleshooting

### Hangs at "=== CAMERA OPENING ===" (empty or dead DISPLAY)

An empty `DISPLAY=` or a `DISPLAY` with no X server behind it (`DISPLAY=:0` when
the desktop is `:1`) makes Argus fail, and the ZED wrapper then waits forever at
`=== CAMERA OPENING ===`. Unset it or point it at a live server: `unset DISPLAY`,
or `export DISPLAY=:1` (check with `xdpyinfo`). `just lab2 survey` and
`just lab2 loc-check` test the connection, warn, and unset a dead `DISPLAY` for
the launch.

### Hardware Acceleration Required (VNC Sessions)

The ZED node requires OpenGL hardware acceleration. Standard VNC will cause failures.

**Solution**: Use TurboVNC with VirtualGL:

```bash
VGL_DISPLAY=:1 /opt/TurboVNC/bin/vncserver :2 -vgl
```

Systemd service available:
- File: `~/.config/systemd/user/turbovnc.service`
- Commands: `systemctl --user {start|stop|status|restart} turbovnc`

### Camera Freeze / Stream Failed to Start

Errors like:
```
(Argus) Error Timeout: ...
[WARN] Error opening camera: CAMERA STREAM FAILED TO START
```

Or in logs (`journalctl -u zed_x_daemon`):
```
[ZED-X Daemon] Received invalid message: "ZEDX#0#0#FROZEN"
```

**Recovery**:
```bash
sudo service zed_x_daemon restart
sleep 25  # Wait for driver reconfiguration
```

**Note**: Restarting `nvargus-daemon` alone is NOT sufficient.

### IMU reaches the topic but not `/sensing/imu/imu_data` (QoS)

Symptom: `/sensing/camera/zedxm/imu/data` runs at 100 Hz, `/sensing/imu/imu_data`
is empty, `imu_corrector` warns about incompatible QoS, and `gyro_odometer`
publishes no twist (the EKF then runs on its other inputs alone).

Cause: the wrapper (`zed-ros2-wrapper` humble-v5.0.0) hard-codes BEST_EFFORT,
and the stock `imu_corrector` subscribes RELIABLE; the two never match.

Fix (in the sensor kit, applies to `imu_source:=zed` on every path): both ZED
launches (`zed_with_object_detection.launch.xml`, `zed_imu_only.launch.xml`)
load `config/zed_imu_qos.yaml`, which sets
`qos_overrides./sensing/camera/zedxm/imu/data.publisher.reliability: reliable`.
The wrapper creates its publishers with `QosOverridingOptions::with_default_policies()`,
so the override key is the fully resolved topic name. Check on the car:

```bash
ros2 topic info -v /sensing/camera/zedxm/imu/data | grep -A6 'Publisher'   # Reliability: RELIABLE
ros2 topic hz /sensing/imu/imu_data                                         # ~100 Hz
ros2 topic hz /localization/twist_estimator/twist_with_covariance          # ~20 Hz
```

### Check Service Logs

```bash
journalctl -u zed_x_daemon --since "30 minutes ago"     # ZED X daemon
journalctl -u nvargus-daemon --since "30 minutes ago"   # NVIDIA Argus
journalctl -k | grep -i "zed\|gmsl\|ar0234"            # Kernel camera errors
```

## Known Issues

- Detection box positions may not perfectly align with point cloud coordinates (coordinate transformation issue)
