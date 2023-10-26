# 在gazebo中添加相机、激光雷达等传感器的仿真插件

参考：
gazebo插件与传感器的添加https://www.guyuehome.com/36462
<br>
https://classic.gazebosim.org/tutorials?tut=ros_gzplugins&cat=connect_ros#OpenniKinect
<br>
https://github.com/arnabsinha/d435i_gazebo
<br>

### D435i传感器插件获取
realsense D435i gazebo slam仿真
包含realsense T265 D435i的urdf和sdf文件、realsense_gazebo_plugin包及realsense 模型文件使用示例。

下载realsense 仿真模型
[catkin_ws]表示自定义的工作目录

```bash
mkdir -p [catkin_ws]/src
cd [catkin_ws]/src
git clone https://gitee.com/nie_xun/realsense_ros_gazebo.git
cd [catkin_ws]
catkin_make
source devel/setup.sh
```
**运行D435仿真环境测试**
D435
`roslaunch realsense_ros_gazebo simulation_sdf.launch`
运行结果：
![image](https://github.com/Luyao0LIU/ROS_Tutorials/assets/128677149/a038a823-8050-4a97-9b3c-5466e4b3cd6d)

**D435i**
相比D435多一个camera/imu topic
`roslaunch realsense_ros_gazebo simulation_D435i_sdf.launch`
![image](https://github.com/Luyao0LIU/ROS_Tutorials/assets/128677149/bda6b192-9c4c-4758-bbd1-51a1ca0e838b)







