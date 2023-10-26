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







