# Camera dependencies installation
to make use of the camera is necessary to manually install the camera dependencies, taking into account you have already installed the Orbecc SDK (in case you cloned this repo, you got it). Start executing on yor bash terminal: 

```bash
  # assume you have sourced ROS environment, same blow
  sudo apt install libgflags-dev nlohmann-json3-dev
  sudo apt install ros-$ROS_DISTRO-image-transport 
  sudo apt install ros-$ROS_DISTRO-image-publisher 
  sudo apt install ros-$ROS_DISTRO-camera-info-manager
  sudo apt install ros-$ROS_DISTRO-diagnostic-updater
  sudo apt install ros-$ROS_DISTRO-diagnostic-msgs
  sudo apt install ros-$ROS_DISTRO-statistics-msgs
  sudo apt install ros-$ROS_DISTRO-backward-ros libdw-dev

```
in our specific case, the distro is Humble:

```bash
  sudo apt install libgflags-dev nlohmann-json3-dev
  sudo apt install ros-humble-image-transport 
  sudo apt install ros-humble-image-publisher 
  sudo apt install ros-humble-camera-info-manager
  sudo apt install ros-humble-diagnostic-updater
  sudo apt install ros-humble-diagnostic-msgs
  sudo apt install ros-humble-statistics-msgs
  sudo apt install ros-humble-backward-ros libdw-dev  
```
### Getting started with the Camera

  ```bash
  cd ~/ros2_ws/
  # build release, Default is Debug
  colcon build --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release
  ```

 The camera we have needs to be configured in a new launch file since it's not part of this package. To do that, we need to go to the `orbbec_camera` package and open Visual Studio.

```bash
  cd ~/ros2_ws/src/OrbbecSDK_ROS2/orbbec_camera
  code .
```
Here we will duplicate the launch script that is most similar to our camera, `astra_pro2.launch.py`, to change as few things as possible and rename it to astra\_pro\_plus.launch.py.
  <p align="center">
    <img src="resources/image (1).png" alt="Project Logo" width="250"/>
  </p>
Now in this file we need to make the necessary changes so that the parameters in the code match what the camera requires:

* Change the **color** camera configuration:
```bash
DeclareLaunchArgument('color_width', default_value='640'),
DeclareLaunchArgument('color_height', default_value='480'),
DeclareLaunchArgument('color_fps', default_value='30'), #estaba en 10
DeclareLaunchArgument('color_format', default_value='RGB888'), #estaba en UYVY
```
The camara accepts the following config
<p align="center">
    <img src="resources/image (2).png" alt="Project Logo" width="250"/>
  </p>

we change the depth config
```bash
DeclareLaunchArgument('depth_width', default_value='640'),
DeclareLaunchArgument('depth_height', default_value='480'),
DeclareLaunchArgument('depth_fps', default_value='30'), #estaba en 10
DeclareLaunchArgument('depth_format', default_value='Y11'),
```
we change ir config 

```bash
DeclareLaunchArgument('ir_width', default_value='640'),
DeclareLaunchArgument('ir_height', default_value='480'),
DeclareLaunchArgument('ir_fps', default_value='30'), #estaba en 10
DeclareLaunchArgument('ir_format', default_value='Y10'),
```
we finally compile and source 

```bash

cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

```
now we are ready to launch

```bash
ros2 launch orbbec_camera astra_pro_plus.launch.py 
```