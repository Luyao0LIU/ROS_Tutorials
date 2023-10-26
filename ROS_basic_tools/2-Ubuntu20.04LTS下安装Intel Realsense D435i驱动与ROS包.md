# Release安装
<br>
Referce:<br>
https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages
<br>
https://github.com/IntelRealSense/realsense-ros
<br>
https://github.com/amov-lab/Prometheus
<br>
realsense D435i gazebo slam(px4)仿真
https://blog.csdn.net/weixin_41469272/article/details/117919845
<br>
https://gitee.com/nie_xun/realsense_ros_gazebo
<br>
Ubuntu20.04LTS下安装Intel Realsense D435i驱动与ROS包https://blog.csdn.net/wanghq2013/article/details/123325671
<br>



 
## 方法1
### Installing the packages:
Register the server's public key:
```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
```
Make sure apt HTTPS support is installed: sudo apt-get install apt-transport-https

Add the server to the list of repositories:

```bash
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
```
Install the libraries (see section below if upgrading packages):
```bash
sudo apt-get install librealsense2-dkms
sudo apt-get install librealsense2-utils
```
The above two lines will deploy librealsense2 udev rules, build and activate kernel modules, runtime library and executable demos and tools.

Optionally install the developer and debug packages:
```bash
sudo apt-get install librealsense2-dev
sudo apt-get install librealsense2-dbg
```
With dev package installed, you can compile an application with librealsense `using g++ -std=c++11 filename.cpp -lrealsense2` or an IDE of your choice.

Reconnect the Intel RealSense depth camera and run: `realsense-viewer` to verify the installation.

Verify that the kernel is updated :
`modinfo uvcvideo | grep "version:"` should include `realsense` string

Upgrading the Packages:
Refresh the local packages cache by invoking:
```python
sudo apt-get update
```

Upgrade all the installed packages, including `librealsense` with:
```bash
sudo apt-get upgrade
```

To upgrade selected packages only a more granular approach can be applied:
```bash
sudo apt-get --only-upgrade install <package1 package2 ...>
```
E.g:
```bash
sudo apt-get --only-upgrade install  librealsense2-utils librealsense2-dkms
```

## 方法2
（源代码）：
下载librealsense `git clone https://github.com/IntelRealSense/librealsense.git`，然后进入该目录，运行下列指令安装和编译依赖项：
```bash
sudo apt-get install libudev-dev pkg-config libgtk-3-dev
sudo apt-get install libusb-1.0-0-dev pkg-config
sudo apt-get install libglfw3-dev
sudo apt-get install libssl-dev

sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger 
mkdir build
cd build
cmake ../ -DBUILD_EXAMPLES=true
make
sudo make install
```
测试安装结果
```bash
realsense-viewer 
```

## ROS包安装
方式一（apt）：
安装ROS版本的realsense2_camera

```bash
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera
sudo apt-get install ros-$ROS_DISTRO-realsense2-description
```
方式二（源代码）：
进入catkin_ws工作空间的src目录，下载realsense和ddynamic_reconfigure，然后用ROS的catkin_make指令编译。
```bash
cd ~/catkin_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git
git clone https://github.com/pal-robotics/ddynamic_reconfigure.git
cd ~/catkin_ws && catkin_make
```
安装rgbd-launch
rgbd_launch是一组打开RGBD设备，并load 所有nodelets转化 raw depth/RGB/IR 流到深度图(depth image), 视差图(disparity image)和点云(point clouds)的launch文件集。
```bash
sudo apt-get install ros-noetic-rgbd-launch
```
测试编译结果：
```bash
roslaunch realsense2_camera demo_pointcloud.launch 
```
![image](https://github.com/Luyao0LIU/ROS_Tutorials/assets/128677149/adb7c6f0-ccee-4cb8-a53f-39fa4578d9d8)






