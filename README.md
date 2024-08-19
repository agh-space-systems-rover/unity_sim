# Unity Simulation

Unity simulation environment for AGH Space Systems robotics projects.

![](./docs/cover.png)

## Table of Contents

- [Getting Started](#getting-started)
- [IMU Simulation](#imu-simulation)
- [GPS Simulation](#gps-simulation)
- [How RealSense Cameras are Simulated](#how-realsense-cameras-are-simulated)
- [RealSense Publisher Plugin Development](#realsense-publisher-plugin-development)
- [Creating a New Map](#creating-a-new-map)

## Getting Started

Firstly, please clone the repository to your ROS 2 workspace:
```bash
cd src
git clone git@github.com:agh-space-systems-rover/unity_sim.git
cd ..
colcon build
source install/setup.bash
```

Once the workspace is built, you need to find out which Unity version is required by this project:
```bash
ros2 run unity_sim unity_version
```
And this command will yield something akin to `Unity 2023.1.9f1`. (Exact version may differ!)

You need to install this version of Unity on your system. Start by installing Unity Hub using the [official instructions](https://docs.unity3d.com/hub/manual/InstallHub.html#install-hub-linux). (Users on Arch Linux can install `unityhub` from the AUR.) Once you have Unity Hub installed, it should prompt you to sign into your account in order to activate a personal license. When you are logged in, you can install the required version of Unity. Unity Hub only provides download links for the latest versions of Unity, so you will need to use the [Unity Archive](https://unity.com/releases/editor/archive) to find the version you need. From there you can click a button that will open Unity Hub and start the download. After the download is complete, it is best to test if your installation is able to run other demo projects, but you should be fine to close Unity Hub now and run the simulation:
```bash
ros2 launch unity_sim unity_sim.launch.py
```

On the first run, the project will take a while to start, because it needs to download some packages from the web and import all the assets. Subsequent runs however will be much faster. Unity might still take up to a dozen of seconds to start, so you can always use the above command to run the simulation in a new terminal window, while you continue to restart other nodes separately.

> [!IMPORTANT]
> If there are errors in the Unity Console that mention problems with the RealSense plugin, you may need to manually remove `UnityRSPublisherPlugin.so` file from the `unity_sim/Assets/Simulation/RealSense` directory and restart the simulation to trigger a re-build.
> See the [RealSense Publisher Plugin Development](#RealSense-Publisher-Plugin-Development) section for more information.

If you wish to upgrade the simulation to a newer version of Unity, please launch it using the Unity Hub. The project directory is located [here](./unity_project/unity_sim) and will need to be manually selected in the Unity Hub.

## IMU Simulation

The simulation provides a virtual IMU sensor. It is a standalone [C# script](./unity_sim/Assets/Simulation/IMU/IMU.cs) that can be attached to any GameObject of choice. The sensor works by comparing the subsequent positions of the GameObject between this and previous physics frame. In this way it can recover the velocity. Then the velocity from the previous frame is compared with the current velocity to compute the acceleration. Angular values are calculated in an analogous way. The IMU's topic and report frequency is configurable in the settings of the script component. Data is published to `/imu/data`.

## GPS Simulation

A simulated GPS unit is by default attached as a prefab to Kalman. It is a GameObject with an attached [C# script](./unity_project/unity_sim/Assets/Simulation/GPS/GPS.cs). The script searches for GPSBaseStation objects in the scene and uses them as reference points to calculate the GPS position of the GPS unit. The position is published to `/gps/fix`. You can put an instance of the GPSProbe prefab in your scene and use a button in its custom editor UI to log info about the base stations and the current GPS position of the probe. Additionally, the probe will log the yaw of the whole Unity world relative to true north. It can be copy-pasted right into IMU's Yaw Offset field to make the IMU properly point to the north.

## How RealSense Cameras are Simulated

In Unity each RealSense is a prefab composed of a single `GameObject` that contains a camera and the control script. The script renders the camera at a set rate and applies a shader that embeds the depth information in the alpha channel of the image. When a frame is available, it is read by the script onto the CPU, and from there [native C++ code](./unity_sim/unity_rs_publisher_plugin/) in invoked that sends the unmodified binary buffer over a Unix socket to the `unity_rs_publisher` ROS node. The node generates point clouds (this is currently disabled), image messages, camera info and publishes all this data. Importantly, the node subscribes to `{camera}/unity_rs_publisher/meta` topics to receive the cameras' configurations. Only then the node has all the required information to generate its own messages. 

## RealSense Publisher Plugin Development

This Unity native plugin is a rather simple piece of code that forwards live feed from simulated RealSense cameras, over sockets, to a ROS 2 publisher node.
It is distributed within this repository in binary form. The component is built for the AMD64 Linux platform. You **DO NOT NEED** to install the below components if you do not plan to develop the plugin and only want to use it.

This software must be installed on the host computer in order to build the plugin:
- CMake
- C++ 20 compiler

The plugin will only be recompiled if the simulation [starts](./unity_sim/unity_sim/__init__.py) and the [binary file](./unity_sim/Assets/Simulation/RealSense/UnityRSPublisherPlugin.so) is not found in the assets folder. That is why you need to manually remove it and restart the simulation to recompile the plugin.

## Creating a New Map

On European Rover Challenge you are usually given a 3D model of the terrain prior to the competition.
You can use it to create a new terrain in Unity:

1. Clean the imported 3D geometry.
2. Create a a custom shader that colour points based on their height (pass colour directly to the Surface output, do not use Principled/Diffuse).
3. Disable any tonemappers and post-processing effects.
4. Render the heightmap using an orthographic camera.
5. Save as 16 bit grayscale PNG.
6. Convert to RAW using GIMP.
7. Create a new scene in Unity and Save it to `Assets/Simulation/Scenes/NameOfMyNewScene.unity`.
8. Use Window -> Terrain Toolbox in Unity to import the heightmap and create a terrain.
9. Move the newly generated Assets/Terrain directory to `Assets/Simulation/Scenes/NameOfMyNewScene` folder.
10. Optionally copy over global post processing volume and a skybox from another scene.

Now you have the terrain in Unity.
Later, you may paint textures on top of it, but that is not necessary for the simulation.

Moving on, you can add in the robot and geographic reference points:

1. Instantiate Kalman Prefab in the scene.
2. Now you can drive on your terrain.
3. Instantiate FollowCamera Prefab.
4. Configure the FollowCamera Component on FollowCamera object to follow the Kalman object (to adjust the look-at pos, add an Empty GameObject as a child of Kalman and follow that).
5. Now your view follows the robot.
6. Add at least 3 GPSBaseStation prefabs. Configure each one with the assumed lat/long of the base station.
7. Use GPSProbe to find the world's yaw relative to true north and put that value as IMU's yaw offset.
8. Now `/imu/data` and `/gps/fix` messages will be published with the correct data. You should be able to see Kalman's rotation and position on the map in Ground Station.
