# Unity Simulation

Unity simulation environment for AGH Space Systems robotics projects.

## Giant Frame Time Spikes Every Few Seconds

RealSense cameras have to send massive amounts of data over rosbridge. Since rosbridge does not support fragmentation (it simply crashes), Unity sends each frame as a single message. This results in large allocations within the Websockets library and subsequently Garbage Collector has a lot of work to do.
Workaround: Disable RealSense cameras or decrease their resolution (or framerate).
Solution: Find a way to enable fragmentation in rosbridge.
