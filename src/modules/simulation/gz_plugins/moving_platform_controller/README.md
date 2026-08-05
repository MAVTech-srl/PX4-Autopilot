# Moving Platform Controller

This plugin controls a moving platform that emulates ships/trucks/etc. to
takeoff and land on. The platform moves at a constant mean velocity, with added
random fluctuations in velocity and angular velocity.

## Dependencies

This depends on the [moving platform world](https://github.com/PX4/PX4-gazebo-models/blob/moving_platform_world/worlds/moving_platform.sdf) in the `PX4-gazebo-models` repo, so ensure that the `Tools/simulation/gz` submodule is recent enough. That world contains the corresponding [moving platform model](https://github.com/PX4/PX4-gazebo-models/blob/moving_platform_world/models/moving_platform/model.sdf). Within that, we include this plugin.


## Usage & Configuration

Start by selecting the moving_platform world, which loads the plugin. We need to set the pose so the aircraft is above the platform, which is at a height of 2m.

```
PX4_GZ_MODEL_POSE=0,0,2.2 PX4_GZ_WORLD=moving_platform make px4_sitl gz_standard_vtol
```

The velocity (in m/s) can be set with the `PX4_GZ_PLATFORM_VEL` and `PX4_GZ_PLATFORM_HEADING_DEG` environment variables. By default it is 1 m/s in east direction. The heading is such that 0 represents east, 90 north, 180 west, and 270 south.

```
PX4_GZ_PLATFORM_VEL=5 PX4_GZ_PLATFORM_HEADING_DEG=135 PX4_GZ_MODEL_POSE=0,0,2.2,0,0,0 PX4_GZ_WORLD=moving_platform make px4_sitl gz_standard_vtol
```

A sinusoidal vertical bob (heave) can be added on top of the height setpoint with `PX4_GZ_PLATFORM_HEAVE_AMPL` (amplitude in m, default 0 = disabled) and `PX4_GZ_PLATFORM_HEAVE_PERIOD` (period in s, default 6). Both position and velocity of the wave are fed forward, so the platform tracks it cleanly instead of lagging behind.

```
PX4_GZ_PLATFORM_HEAVE_AMPL=0.3 PX4_GZ_PLATFORM_HEAVE_PERIOD=6 PX4_GZ_MODEL_POSE=0,0,2.2,0,0,0 PX4_GZ_WORLD=moving_platform make px4_sitl gz_standard_vtol
```

The platform also carries a low-pass filtered random-noise wrench (waves/road noise jitter, applied every step regardless of heave), scaled with `PX4_GZ_PLATFORM_NOISE_AMPL` (default **0 = disabled**; set to `1.0` to restore the original always-on jitter).

```
PX4_GZ_PLATFORM_NOISE_AMPL=1 PX4_GZ_MODEL_POSE=0,0,2.2,0,0,0 PX4_GZ_WORLD=moving_platform make px4_sitl gz_standard_vtol
```

If the vehicle spawns directly on the platform, it starts moving as soon as the vehicle model appears in the world -- generally well before you can complete pre-arm checks and arm, so GPS/position keeps drifting while disarmed and the vehicle refuses to take off. `PX4_GZ_PLATFORM_START_DELAY` (seconds, default 0 = original behaviour) keeps the platform stationary for that long *after* the vehicle spawns, giving you time to arm and take off with a stable fix before it starts moving.

```
PX4_GZ_PLATFORM_START_DELAY=20 PX4_GZ_MODEL_POSE=0,0,2.2,0,0,0 PX4_GZ_WORLD=moving_platform make px4_sitl gz_standard_vtol
```

For a controlled "moves a bit, then stops" test instead of moving forever, set `PX4_GZ_PLATFORM_TRAVEL_DISTANCE` (metres, default 0 = moves forever). The platform stops for good once it has covered that horizontal distance from wherever it was when it started moving (i.e. after any `PX4_GZ_PLATFORM_START_DELAY` has elapsed).

```
PX4_GZ_PLATFORM_TRAVEL_DISTANCE=5 PX4_GZ_PLATFORM_VEL=1 PX4_GZ_MODEL_POSE=0,0,2.2,0,0,0 PX4_GZ_WORLD=moving_platform make px4_sitl gz_standard_vtol
```

For manual control instead of a timer, set `PX4_GZ_PLATFORM_WAIT_FOR_TRIGGER=1` (default 0/off) -- the platform stays stationary indefinitely (ignoring `PX4_GZ_PLATFORM_START_DELAY`) until you publish a message on `/model/<model_name>/start_motion`.

```
PX4_GZ_PLATFORM_WAIT_FOR_TRIGGER=1 PX4_GZ_MODEL_POSE=0,0,2.2,0,0,0 PX4_GZ_WORLD=moving_platform make px4_sitl gz_standard_vtol
```

Then, whenever you want it to start moving:

```
gz topic -t /model/moving_platform_aruco/start_motion -m gz.msgs.Empty -p ""
```

(replace `moving_platform_aruco` with the actual model name if different -- the plugin logs the exact topic it's waiting on at startup)

To use the plugin with a *different* world or model, add the following to the model.sdf:

```
<?xml version="1.0" encoding="UTF-8"?>
<sdf version="1.9">
  <model name="flat_platform">

    <!-- the rest of your model -->

    <link name="platform_link">
      <!-- define the link representing the platform -->
    </link>

    <plugin
      filename="libMovingPlatformController.so"
      name="custom::MovingPlatformController">
      <link_name>platform_link</link_name>
    </plugin>

  </model>
</sdf>
```

## Limitations & Future Ideas

 - Apart from the velocity and heading, nothing is configurable: Noise amplitude and frequency spectrum, initial acceleration, feedback gains.
    - Feel free to change these ad-hoc in code, or open an issue / propose a PR for better config options.
 - This plugin does not communicate the state of the platform with PX4. If that is needed, there are a couple options:
    - Read the pose of the platform in `GZBridge::poseInfoCallback`
    - Add an IMU sensor to the platform link, listen to it and the existing NavSat sensor from `GZBridge`.
    - Add a custom gazebo transport message containing all needed info about the platform, populate and publish it from the plugin here, listen to that in `GZBridge`.
