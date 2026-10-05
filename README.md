# ArduPilot Gazebo Plugin

[![ubuntu-build](https://github.com/ArduPilot/ardupilot_gazebo/actions/workflows/ubuntu-build.yml/badge.svg)](https://github.com/ArduPilot/ardupilot_gazebo/actions/workflows/ubuntu-build.yml)
[![ccplint](https://github.com/ArduPilot/ardupilot_gazebo/actions/workflows/ccplint.yml/badge.svg)](https://github.com/ArduPilot/ardupilot_gazebo/actions/workflows/ccplint.yml)
[![cppcheck](https://github.com/ArduPilot/ardupilot_gazebo/actions/workflows/ccpcheck.yml/badge.svg)](https://github.com/ArduPilot/ardupilot_gazebo/actions/workflows/ccpcheck.yml)

This is the official ArduPilot plugin for [Gazebo](https://gazebosim.org/home).
It replaces the previous
[`ardupilot_gazebo`](https://github.com/khancyr/ardupilot_gazebo)
plugin and provides support for the recent releases of the Gazebo simulator:
[Gazebo Garden](https://gazebosim.org/docs/garden/install), [Gazebo Harmonic (LTS)](https://gazebosim.org/docs/harmonic/install), [Gazebo Ionic](https://gazebosim.org/docs/ionic/install) and [Gazebo Jetty (LTS)](https://gazebosim.org/docs/jetty/install).

It also adds the following features:

- More flexible data exchange between SITL and Gazebo using JSON.
- Additional sensors supported.
- True simulation lockstepping. It is now possible to use GDB to stop
  the Gazebo time for debugging.
- Improved 3D rendering using the `ogre2` rendering engine.

The project comprises a Gazebo plugin to connect to ArduPilot SITL
(Software In The Loop) and some example models and worlds.

## Prerequisites

Gazebo Garden or Harmonic is supported on Ubuntu 22.04 (Jammy).
Harmonic is recommended.
If you are running Ubuntu as a virtual machine you will need at least
Ubuntu 20.04 in order to have the OpenGL support required for the
`ogre2` render engine. Gazebo and ArduPilot SITL will also run on macOS
(Big Sur, Monterey and Venturua; Intel and M1 devices).

Follow the instructions for a binary install of
[Gazebo Garden](https://gazebosim.org/docs/garden/install) or [Gazebo Harmonic](https://gazebosim.org/docs/harmonic/install) or [Gazebo Ionic](https://gazebosim.org/docs/ionic/install)
and verify that Gazebo is running correctly.

Set up an [ArduPilot development environment](https://ardupilot.org/dev/index.html).
In the following it is assumed that you are able to run ArduPilot SITL using
the [MAVProxy GCS](https://ardupilot.org/mavproxy/index.html).

## Installation

Install additional dependencies:

### Ubuntu

#### Garden (apt)

Manual - Gazebo Garden Dependencies:

```bash
sudo apt update
sudo apt install libgz-sim7-dev rapidjson-dev
sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
```

#### Harmonic (apt)

Manual - Gazebo Harmonic Dependencies:

```bash
sudo apt update
sudo apt install libgz-sim8-dev rapidjson-dev
sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl
```

#### Rosdep

Use rosdep with
[osrf's rosdep rules](https://github.com/osrf/osrf-rosdep?tab=readme-ov-file#1-use-rosdep-to-resolve-gazebo-libraries)
to manage all dependencies. This is driven off of the environment variable `GZ_VERSION`.

```bash
export GZ_VERSION=harmonic # or garden or ionic
sudo bash -c 'wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list -O /etc/ros/rosdep/sources.list.d/00-gazebo.list'
rosdep update
rosdep resolve gz-harmonic # or gz-garden or gz-ionic
# Navigate to your ROS workspace before the next command.
rosdep install --from-paths src --ignore-src -y
```

### macOS

```bash
brew update
brew install rapidjson
brew install opencv gstreamer
```

Ensure the `GZ_VERSION` environment variable is set to either
`garden` or `harmonic` or `ionic`.

Clone the repo and build:

```bash
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

## Configure

Set the Gazebo environment variables in your `.bashrc` or `.zshrc` or in 
the terminal used to run Gazebo.

#### Terminal

Assuming that you have cloned the repository to `$HOME/ardupilot_gazebo`:

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
```

#### .bashrc or .zshrc

Assuming that you have cloned the repository to `$HOME/ardupilot_gazebo`:

```bash
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
```

Reload your terminal with `source ~/.bashrc` (or `source ~/.zshrc` on macOS).

## Usage

### Argos RGB and thermal sensors

The Argos gimbal contains the existing independently zoomable RGB camera and
an independent grayscale `thermal_camera` sensor. The thermal sensor produces
640x512 `L_INT8` frames at 30 Hz, with a 1.0 rad horizontal field of view.
Its pose is offset 3 cm along the pitch link's positive Y axis, with the same
optical orientation as the RGB camera. It has no radiometric-temperature
model, RGB zoom control, streaming, or capture pipeline. The Argos camera
controller reports a thermal frame summary every 30 received frames.

### 1. Iris quad-copter

#### Run Gazebo

```bash
gz sim -v4 -r iris_runway.sdf
```

The `-v4` parameter is not mandatory, it shows additional information and is
useful for troubleshooting.

#### Run ArduPilot SITL

To run an ArduPilot simulation with Gazebo, the frame should have `gazebo-`
in it and have `JSON` as model. Other commandline parameters are the same
as usual on SITL.

```bash
sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console
```

#### Arm and takeoff

```bash
STABILIZE> mode guided
GUIDED> arm throttle
GUIDED> takeoff 5
```

### 2. Zephyr delta wing  

The Zephyr delta wing is positioned on the runway for vertical take-off. 

#### Run Gazebo

```bash
gz sim -v4 -r zephyr_runway.sdf
```

#### Run ArduPilot SITL

```bash
sim_vehicle.py -v ArduPlane -f gazebo-zephyr --model JSON --map --console
```

#### Arm, takeoff and circle

```bash
MANUAL> mode fbwa
FBWA> arm throttle
FBWA> rc 3 1800
FBWA> mode circle
```

#### Increase the simulation speed

The `zephyr_runway.sdf` world has a `<physics>` element configured to run
faster than real time: 

```xml
<physics name="1ms" type="ignore">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>-1.0</real_time_factor>
</physics>
```

To see the effect of the speed-up set the param `SIM_SPEEDUP` to a value
greater than one:

```bash
MANUAL> param set SIM_SPEEDUP 10
```

### 3. Streaming camera video

Images from camera sensors may be streamed with GStreamer using
the `GstCameraPlugin` sensor plugin. The example gimbal models include the
plugin element:

```xml
<plugin name="GstCameraPlugin"
    filename="GstCameraPlugin">
  <udp_host>127.0.0.1</udp_host>
  <udp_port>5600</udp_port>
  <use_basic_pipeline>true</use_basic_pipeline>
  <use_cuda>false</use_cuda>
</plugin>
```

The `<image_topic>` and `<enable_topic>` parameters are deduced from the
topic name for the camera sensor, but may be overriden if required.

The `gimbal.sdf` world includes a 3 degrees of freedom gimbal with a
zoomable camera. To start streaming:

```bash
gz topic -t /world/gimbal/model/mount/model/gimbal/link/pitch_link/sensor/camera/image/enable_streaming -m gz.msgs.Boolean -p "data: 1"
```

Display the streamed video:

```bash
gst-launch-1.0 -v udpsrc port=5600 caps='application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264' ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink sync=false
```

View the streamed camera frames in [QGC](http://qgroundcontrol.com/):

`Open QGC > Application Settings > Video Settings > Select UDP h.264 Video Stream & use port 5600`

![qgc_video_settings](https://github.com/user-attachments/assets/61fa4c2a-37e2-47cf-abcf-9f110d9c2015)


### 4. Using 3d Gimbal

The Iris model is equipped with a 3d gimbal and camera that can be controlled directly in MAVProxy using the RC overrides.

#### Run Gazebo

```bash
gz sim -v4 -r iris_runway.sdf
```

#### Run ArduPilot SITL with a specified parameter file

```bash
cd ardupilot

sim_vehicle.py -D -v ArduCopter -f JSON --add-param-file=$HOME/ardupilot_gazebo/config/gazebo-iris-gimbal.parm --console --map
```

Control action for gimbal over RC channel:

| Action | Channel | RC Low | RC High |
| ------------- | ------------- | ------------- | ------------- |
| Roll | RC6 | Roll Left | Roll Right |
| Pitch | RC7 | Pitch Down | Pitch Up |
| Yaw | RC8 | Yaw Left | Yaw Right |

Example usage:

`rc 6 1100` - Gimbal rolls left

`rc 7 1900` - Gimbal pitch upwards

`rc 8 1500` - Gimbal yaw neutral

### 5. Argos Iris camera and zoom bridge

The Argos example is isolated from the reference `iris_runway` and
`gimbal_small_3d` models:

- World: `worlds/argos/argos_iris_runway.sdf`
- Vehicle: `models/argos_iris`
- Gimbal and RGB camera: `models/argos_gimbal`

The model retains the `base_link -> yaw_link -> roll_link -> pitch_link`
gimbal chain. The gimbal include pose in the vehicle model is
`0 -0.01 -0.124923 90 0 90` (degrees) relative to the vehicle; the gimbal's
own model pose is `0 0 0.18 0 0 0`, and `gimbal_joint` constrains its
`base_link` to the vehicle. The camera sensor is mounted on `pitch_link`;
its sensor pose remains `0 0 0 -1.57 -1.57 0` and its Gazebo frame ID remains
`pitch_link`. The sensor data is therefore expressed in `pitch_link`; the
camera optical axes are obtained from that link through the sensor pose. No
separate optical-frame link is introduced. These frames are preserved for
future camera/gimbal state reporting.

The SDF rotations make the sensor optical axis point forward at zero pitch.
The existing Argos ArduPilot servo mapping translates mount pitch to the
inverse pitch-joint angle, with the pitch joint axis along local +X. Therefore,
`MNT1_NEUTRAL_Y=0` points forward, while `MNT1_RETRACT_Y=-90` points down.
Select the startup pose using `MNT1_DEFLT_MODE=1` (neutral, forward) or
`MNT1_DEFLT_MODE=0` (retracted, down). Both poses fit within the existing
Argos pitch limits. Keep `MNT1_TYPE=1` for the Gazebo servo-driven gimbal;
camera discovery is separate and uses `CAM1_TYPE=6` on `SERIAL5_PROTOCOL=2`.
ArduPilot's Servo mount backend applies the neutral/retract angle vectors to
servo outputs 9-11; the Argos `ArduPilotPlugin` maps those outputs to the
Gazebo joints. `SIM_AVT_CM62` remains the separate MAVLink camera/gimbal
peripheral for component discovery and camera commands; no extra Argos
startup-orientation setting is involved.

The RGB camera is configured at 1920x1080, 30 Hz and 2.0 rad horizontal
field of view. With square pixels, its initial pinhole intrinsics derive
from that configuration:

```text
fx = fy = width / (2 * tan(HFOV / 2)) ~= 616.41
cx = width / 2 = 960
cy = height / 2 = 540
```

All five distortion coefficients are zero in the SDF and can be changed there.
No separate intrinsic values are hard-coded.
On the validation host, Gazebo published 1920x1080 images and camera info
with `fx=616.4089`, `fy=616.4089`, `cx=960`, `cy=540`, and zero distortion.
The sensor timestamps advanced by approximately 33 ms in simulation time.
The headless render on that host ran at approximately 10 wall-clock frames/s
with a real-time factor near 0.30; wall-clock throughput depends on rendering
performance.

The Argos sensor uses the existing `GstCameraPlugin` basic pipeline to stream
the rendered camera images (including sensor-level HFOV zoom) as RTP/H.264 to
`127.0.0.1:5600`. Streaming starts automatically on the first camera frame;
the plugin does not perform a separate GStreamer crop or zoom.

For the normal Argos startup, add these values to
`config/argos-iris-gimbal.parm` (keep the existing `MNT1_TYPE=1` and
`SERVO9/10/11_FUNCTION` assignments). `SERIAL5_PROTOCOL=2` is essential:
SITL defaults `SERIAL5_PROTOCOL` to `-1` (None), so the simulated peripheral
will not exchange MAVLink until this is set:

```text
SERIAL5_PROTOCOL  2
CAM1_TYPE         6
MNT1_NEUTRAL_X     0
MNT1_NEUTRAL_Y     0
MNT1_NEUTRAL_Z     0
MNT1_RETRACT_X     0
MNT1_RETRACT_Y   -90
MNT1_RETRACT_Z     0
MNT1_DEFLT_MODE    1
```

The existing pitch limits (`MNT1_PITCH_MIN=-135`,
`MNT1_PITCH_MAX=45`) already allow both startup orientations and should be
left unchanged. With `CAM1_COMPID` at its default value of `0`, ArduPilot uses
camera component ID 100.
For a downward startup, change only `MNT1_DEFLT_MODE` to `0`; the neutral and
retract angles can both remain configured so either position is available.
The `gazebo-iris` frame appends the simulated camera/gimbal peripheral argument
automatically. Start SITL and Gazebo in separate terminals:

```bash
cd ~/workspace/nxtqube/ardupilot
Tools/autotest/sim_vehicle.py \
  -v ArduCopter \
  -f gazebo-iris \
  --add-param-file /home/harshal/workspace/nxtqube/ardupilot_gazebo/config/argos-iris-gimbal.parm \
  -N -M \
  --model JSON
```

```bash
cd ~/workspace/nxtqube/ardupilot_gazebo/worlds
gz sim -v4 -r argos/argos_iris_runway.sdf
```

#### Build and run

From the repository root:

```bash
cmake --preset argos-jetty
cmake --build --preset argos-jetty --parallel 4
cmake --install build --prefix "$PWD/build/install"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build/install/lib/ardupilot_gazebo:${GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$PWD/models:$PWD/worlds:${GZ_SIM_RESOURCE_PATH}"
gz sim -v4 -r worlds/argos/argos_iris_runway.sdf
```

#### Zoom control endpoint

ArduPilot remains responsible for MAVLink parsing. `MAV_CMD_SET_CAMERA_ZOOM`
with range zoom is represented at the Gazebo integration boundary as a
`gz.msgs.Double` percentage on `/argos/camera/zoom/percent`, in the configured
range 0-100. The Argos controller linearly maps that percentage to the
configured Gazebo zoom-factor range, currently 1-125, and publishes a
`gz.msgs.Double` to
`/model/argos_gimbal/sensor/rgb_camera/zoom/cmd_zoom`.
The controller's `max_zoom` and the `CameraZoomPlugin`'s `max_zoom` must match.
This mapping is an Argos configuration choice, not a MAVLink zoom-unit
definition; update the SDF parameters together when calibrating a different
camera range.

`CameraZoomPlugin` interprets the published value as a multiplicative zoom
factor. Its current implementation clamps the factor to `[1, max_zoom]` and
sets the target horizontal FOV to `2.0 / factor`; focal length is then
calculated from the camera sensor width. At 0%, the target is 2.0 rad; at
50%, this configured linear factor mapping requests 63x and about 0.03175 rad;
at 100%, it requests 125x and 0.016 rad. The topic callback is asynchronous:
the plugin applies the new focal length/FOV on a subsequent simulation
`PreUpdate`, with the configured slew rate limiting its transition.

Test the bridge while the world is running:

```bash
gz topic -t /argos/camera/zoom/percent -m gz.msgs.Double -p 'data: 50'
gz topic -t /argos/camera/zoom/percent -m gz.msgs.Double -p 'data: 0'
gz topic -t /argos/camera/zoom/percent -m gz.msgs.Double -p 'data: 100'
```

The output zoom topic is `/model/argos_gimbal/sensor/rgb_camera/zoom/cmd_zoom`.
The RGB image and camera-info topics are discoverable with `gz topic -l`.
The controller also accepts a localhost UDP datagram containing the numeric
percentage on port 9010. The corresponding ArduPilot-side adapter belongs in
the ArduPilot repository: `SIM_MAVLinkCamV2` parses the MAVLink camera command
and sends the percentage to this socket. Gazebo does not implement a second
MAVLink parser. The controller forwards the datagram to the same percentage
topic used by direct Gazebo tests.

The `MountAVTCM62Dual` autotest exercises camera discovery and range-zoom
commands through `SIM_MAVLinkCamV2`. The live Argos run also verified component
100 discovery, `CAMERA_INFORMATION` with `HAS_BASIC_ZOOM`, and 0%, 50%, and
100% MAVLink zoom reaching the controller and changing the sensor HFOV to
2.0, approximately 0.031746, and 0.016 rad. The 1920x1080 RTP/H.264 feed
decoded during that run.

#### Still image and video capture

`MAV_CMD_IMAGE_START_CAPTURE` remains handled by ArduPilot's camera backend;
repeated-image intervals are scheduled there as individual captures, and
`MAV_CMD_IMAGE_STOP_CAPTURE` stops future interval requests. Each successful
camera command saves the next actual Gazebo RGB sensor frame as a JPEG under
`/tmp/argos-camera/`, writes a matching `.jpg.json` sidecar, and emits
`CAMERA_IMAGE_CAPTURED` with the successful image index, UTC timestamp, camera
orientation, vehicle position, and local file URL. The sidecar records actual
sensor HFOV, resolution, measured gimbal joint positions, commanded zoom, and
the measured vehicle/camera attitude. Failed image writes are reported as
failed MAVLink commands and do not produce success metadata.

`MAV_CMD_VIDEO_START_CAPTURE` and `MAV_CMD_VIDEO_STOP_CAPTURE` control an
idempotent H.264/MP4 recording pipeline inside the existing
`GstCameraPlugin`. It consumes the same rendered RGB sensor frames as the
existing RTP/H.264 live stream; recording can start or stop without stopping
that live stream. Completed recordings are also written under
`/tmp/argos-camera/`.

## Models

In addition to the Iris and Zephyr models included here, a selection
of models configured use the ArduPilot Gazebo plugin is available in
[ArduPilot/SITL_Models](https://github.com/ArduPilot/SITL_Models). 
Click on the images to see further details.

<table>
<tr>
<td title="Alti Transition">
<a href="https://github.com/ArduPilot/SITL_Models/blob/master/Gazebo/docs/AltiTransition.md">
<img src="https://user-images.githubusercontent.com/24916364/150612555-958a64d4-c434-4f90-94bd-678e6b6011ec.png" width="100%" style="display: block;">
</a>
</td>
<td title="SkyCat TVBS">
<a href="https://github.com/ArduPilot/SITL_Models/blob/master/Gazebo/docs/SkyCatTVBS.md">
<img src="https://user-images.githubusercontent.com/24916364/145025150-4e7e48e1-3e83-4c83-be7b-b944db1d9152.png" width="100%" style="display: block;">
</a>
</td>
<td title="Skywalker X8">
<a href="https://github.com/ArduPilot/SITL_Models/blob/master/Gazebo/docs/SkywalkerX8.md">
<img src="https://user-images.githubusercontent.com/24916364/142733947-1a39e963-0aea-4b1b-a57b-85455b2278fe.png" width="100%" style="display: block;">
</a>
</td>
</tr>
<tr>
<td title="Quadruped">
<a href="https://github.com/ArduPilot/SITL_Models/blob/master/Gazebo/docs/Quadruped.md">
<img src="https://user-images.githubusercontent.com/24916364/144449710-5bab34b4-dabf-410f-b276-d290ddbb54b2.gif" width="100%" style="display: block;">
</a>
</td>
<td title="WildThumper">
<a href="https://github.com/ArduPilot/SITL_Models/blob/master/Gazebo/docs/WildThumper.md">
<img src="https://user-images.githubusercontent.com/24916364/144286154-231ac9b3-e54b-489f-b35e-bc2adb4b1aa0.png" width="100%" style="display: block;">
</a>
</td>
<td title="Rover Playpen">
<a href="https://github.com/ArduPilot/SITL_Models/blob/master/Gazebo/docs/RoverPlayPen.md">
<img src="https://user-images.githubusercontent.com/24916364/144513412-1b0661f1-fdf8-4aed-a745-e8bb73ffca91.jpg" width="100%" style="display: block;">
</a>
</td>
</tr>

</td>
</tr>
<tr>
<td title="Swan-K1">
<a href="https://github.com/ArduPilot/SITL_Models/blob/master/Gazebo/docs/Swan-K1.md">
<img src="https://user-images.githubusercontent.com/24916364/210408630-01e5f56d-57ba-430e-b04d-62cb8d232527.png" width="100%" style="display: block;">
</a>
</td>
<td title="Sawppy Rover">
<a href="https://github.com/ArduPilot/SITL_Models/blob/master/Gazebo/docs/Sawppy.md">
<img src="https://user-images.githubusercontent.com/24916364/210653579-e635ffc2-2962-4221-83a8-9622915a4121.png" width="100%" style="display: block;">
</a>
</td>
<td title="Hexapod Copter">
<a href="https://github.com/ArduPilot/SITL_Models/blob/master/Gazebo/docs/HexapodCopter.md">
<img src="https://user-images.githubusercontent.com/24916364/225340320-9aa31fe2-4602-4036-ba6b-491f72097c01.jpg" width="100%" style="display: block;">
</a>
</td>
</tr>

</table>

## Troubleshooting

For issues concerning installing and running Gazebo on your platform please
consult the Gazebo documentation for [troubleshooting frequent issues](https://gazebosim.org/docs/harmonic/troubleshooting#ubuntu).
