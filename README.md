# driver_quad_encoder

## Overview

This package includes driver software for generic quadrature encoders connected to GPIO pins.

**Keywords:** quadrature encoder driver raspberry_pi

### License

The source code is released under a [MIT license](LICENSE).

**Author: Paul D'Angio<br />
Maintainer: Paul D'Angio, pcdangio@gmail.com**

The driver_quad_encoder package has been tested under [ROS] Melodic and Ubuntu 18.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics)
- [sensor_msgs_ext](https://github.com/pcdangio/ros-sensor_msgs_ext) (extension of sensor_msgs)
- [geometry_msgs_ext](https://github.com/pcdangio/ros-geometry_msgs_ext) (extension of geometry_msgs)
- [pigpio](http://abyz.me.uk/rpi/pigpio/) (Raspberry PI I/O)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

```bash
cd catkin_workspace/src
git clone https://github.com/pcdangio/ros-driver_quad_encoder.git driver_quad_encoder
cd ../
catkin_make
```

## Usage

Run any of the driver nodes with (where xxx is the driver type):

```bash
rosrun driver_quad_encoder driver_quad_encoder_xxx
```

For example, to run the node using a driver for a Raspberry Pi:

```bash
rosrun driver_quad_encoder driver_quad_encoder_rpi
```

## Nodes

### rpi_node

A Raspberry Pi driver for a quadrature encoder.  Ensure that the pigpio daemon is running before starting this node.


#### Published Topics
* **`axis_state`** ([sensor_msgs_ext](https://github.com/pcdangio/ros-sensor_msgs_ext/blob/master/msg/axis_state.msg))

        The current state of the encoder axis.

#### Services

* **`~/set_home`** ([sensor_msgs_ext/set_axis_home](https://github.com/pcdangio/ros-sensor_msgs_ext/blob/master/srv/set_axis_home.srv))

        Sets the home position of the axis to the current position.


#### Parameters

* **`~/gpio_pin_a`** (int, default: 0)

        The GPIO input pin connected to signal A of the encoder.

* **`~/gpio_pin_b`** (int, default: 1)

        The GPIO input pin connected to signal B of the encoder.

* **`~/ppr`** (int, default: 200)

        The pulses per revolution (PPR) of the encoder.

* **`~/publish_rate`** (double, default: 30)

        The rate (in Hz) to publish state and delta messages.


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/pcdangio/ros-driver_quad_encoder/issues).


[ROS]: http://www.ros.org