### Graphic Card Configuration (for Vive & Kinect Camera)


#### Kinect Configuration

In order to run Kinect Camera, two essential packages have to be installed: `libfreenect2` and `OpenNI2`.

##### `libfreenect2` Installation

In prior for `libfreenect2` to work, another package of `libturbojpeg` has to be installed as prerequisite:

```bash
$ apt-get install libturbojpeg
```
Then, go to the website of `libfreenect2` and start installation by clone the repository into your local directory:


https://github.com/OpenKinect/libfreenect2

Then start building procedures:

```bash
$ mkdir build && cd build
$ cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2 -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-8.0
$ make clean
$ make
$ make install
```
> **Note**: crucial stuff is to let the computer look for `/usr/local/cuda-8.0`

> If you encounter the installation error of: 
> 
> ```
> /home/rh3014/libfreenect2/src/cuda_kde_depth_packet_processor.cu:39:25: fatal error: helper_math.h: No such file or directory
> ```
> Try edit the `CMakeLists.txt` file (might not be working) and add the following two lines into it, force the program to overwrite:
> 
> ```
> CXXFLAGS="-fPIC"
> CFLAGS="-fPIC"
>```

If the build execution doesn't work properly, try `cmake` it again and force it to look for the directory of Cuda:

```bash
$ cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/freenect2 -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-8.0
```
> **Note**: remember to remove the CMakeCache.txt file everytime you re-cmake it
>
>```
>$ rm CMakeCache.txt
>```

After that, execute the `make` command:

```bash
$ make -j
$ make -j install
```
If the build succeeded, something analagous to the following should be seen as response in the terminal:

```
-- Install configuration: "RelWithDebInfo"
-- Installing: /home/rh3014/freenect2/lib/libfreenect2.so.0.2.0
-- Installing: /home/rh3014/freenect2/lib/libfreenect2.so.0.2
-- Installing: /home/rh3014/freenect2/lib/libfreenect2.so
-- Installing: /home/rh3014/freenect2/include/libfreenect2
-- Installing: /home/rh3014/freenect2/include/libfreenect2/logger.h
-- Installing: /home/rh3014/freenect2/include/libfreenect2/packet_pipeline.h
-- Installing: /home/rh3014/freenect2/include/libfreenect2/frame_listener.hpp
-- Installing: /home/rh3014/freenect2/include/libfreenect2/frame_listener_impl.h
-- Installing: /home/rh3014/freenect2/include/libfreenect2/registration.h
-- Installing: /home/rh3014/freenect2/include/libfreenect2/libfreenect2.hpp
-- Up-to-date: /home/rh3014/freenect2/include/libfreenect2
-- Installing: /home/rh3014/freenect2/include/libfreenect2/config.h
-- Installing: /home/rh3014/freenect2/include/libfreenect2/export.h
-- Installing: /home/rh3014/freenect2/lib/cmake/freenect2/freenect2Config.cmake
-- Installing: /home/rh3014/freenect2/lib/cmake/freenect2/freenect2ConfigVersion.cmake
-- Installing: /home/rh3014/freenect2/lib/pkgconfig/freenect2.pc
-- Installing: /home/rh3014/freenect2/lib/OpenNI2/Drivers/libfreenect2-openni2.so.0
-- Installing: /home/rh3014/freenect2/lib/OpenNI2/Drivers/libfreenect2-openni2.so
```
> **Note**: One benefit installing packages into the directory of `/usr/local` is that other programs may look to `/usr/local` by default, thus the package might be potentially used for other purposes as well.

Eventually, install `OpenNI2` and use `roslaunch` command to launch the program: 

```bash
$ sudo apt-get install ros-kinetic-openni2-launch
$ roslaunch openni2_launch openni2.launch
```

Navigate to the directory where you intend to 


#### Vive Configuration

Firstly, check the Nvdia version on the perception host computer by:

```bash
$ nvidia-smi
```

<span style="color:red"> 
***check the usage of the following line***
</span>

```bash
$ apt-get install dnsmasq
```



#### (Optional) might not work on all computers

Graphic card installation for the Vive

```bash
sudo apt-add-repository ppa:graphics-drivers/ppa
```

#### WebCam Configuration

Other than using the Kinect Sensor, an plausible althernative would be using two Web Cameras to stream through into two screens in the Vive. In order to proceed via this path, the `usb_cam` packages need to be compiled in to the catkin workspace.

Firstly, download `usb_cam` folder from box into `src` directory, then execute:

```bash
$ catkin_make
```
for compiling the ros packages in the workspace. 

After a while, something similar to these following lines should be seen as response:

```
Scanning dependencies of target usb_cam
[ 25%] Building CXX object usb_cam/CMakeFiles/usb_cam.dir/src/usb_cam.cpp.o
[ 50%] Linking CXX shared library /home/rh3014/catkin_ws/devel/lib/libusb_cam.so
[ 50%] Built target usb_cam
Scanning dependencies of target usb_cam_node
[ 75%] Building CXX object usb_cam/CMakeFiles/usb_cam_node.dir/nodes/usb_cam_node.cpp.o
[100%] Linking CXX executable /home/rh3014/catkin_ws/devel/lib/usb_cam/usb_cam_node
[100%] Built target usb_cam_node
```
which means it has been successfully compiled.

> **Note**: since the path of the camera has not been added in `.bashrc`, it is needed to `source catkin_ws/devel/setup.bash` in order to make the camera been seen by other programs/

After that, run:

```bash
roslaunch usb_cam camera.launch
```
to check the stream reponse from the camera.

> **Note** since two cameras has been added to the system, check the `camera.launch` file, the following two lines are defining the camera topic:
> 
> ```
> <remap from="image" to="/camera1/usb_cam1/image_raw"/>
> ...
> <remap from="image" to="/camera2/usb_cam2/image_raw"/>
> ```
> in this case, `camera1` and `camera2`.
> 
> If no camera plugged in, the built in camera will be recognised as one of the cameras. In addition, if these two lines of configuration hasn't been added, the default camera will be recognised as `camera0`.




