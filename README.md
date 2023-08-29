# Unity Simulation

Unity simulation environment for AGH Space Systems robotics projects.

## Prerequisites

This software must be installed on the host computer for the simulation to run:

- CMake
- C++ 20 compiler

Those requirements are imposed because Unity needs a custom native plugin that will get compiled on the first Colcon build.

## How RealSense Cameras are Simulated

In Unity each RealSense is a prefab composed of a single `GameObject` that contains a camera and the control script. The script renders the camera at a set rate and applies a shader that embeds the depth information in the alpha channel of the image. When a frame is available, it is read by the script onto the CPU, and from there [native C++ code](./realsense-native-plugin/) in invoked that sends the binary buffer over a custom WebSocket to the `realsense_bridge` node. The node generates point clouds, camera information and finally communicates with ROS network in order to publish the messages.

## Performance Issues

Currently there is no `realsense_bridge` node and the active code send data directly to ROS using rosbridge. This, however, only allows for extremely sparse point clouds and low quality images. It is not known yet if those performance issues occur because of the `libhv` websocket client or because of rosbridge which AFAIK is written in Python.
