# Unity Simulation

Unity simulation environment for AGH Space Systems robotics projects.

## Prerequisites

This software must be installed on the host computer for the simulation to run:

- CMake
- C++ 20 compiler

Those requirements are imposed because Unity needs a custom native plugin that will get compiled on the first Colcon build.

## How RealSense Cameras are Simulated

In Unity each RealSense is a prefab composed of a single `GameObject` that contains a camera and the control script. The script renders the camera at a set rate and applies a shader that embeds the depth information in the alpha channel of the image. When a frame is available, it is read by the script onto the CPU, and from there [native C++ code](./unity_rs_publisher_plugin/) in invoked that sends the unmodified binary buffer over a Unix socket to the `unity_rs_publisher` ROS node. The node generates point clouds, camera information and publishes all this data.
