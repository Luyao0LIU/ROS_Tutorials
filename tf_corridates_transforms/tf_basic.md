# TF corridates transform
This file introducts how to use tf to transform among frames such as odom, base_link, camera_link and so on.

## (1) Odometry的发布和发布odom到base_link的tf变换
https://blog.csdn.net/sinat_16643223/article/details/113919768
<br>
https://blog.csdn.net/qq_43481884/article/details/105429655

### 各种frame之间的关系介绍

1. ROS中base_link, odom, fixed_frame, target_frame和虚拟大地图map的关系
一般在urdf文件中都要定义base_link，它代表了机器人的主干，其它所有的frame都是相对于base_link定义并粘在一起的。它们一起相对于大地图map移动，让机器人移动就是向tf发布 geometry_msgs::TransformStamped 消息通知ros base_linke相对于map的tf转换关系。先看一下这几个概念在ros中的定义：

2. map
map是虚拟世界中的固定frame, 它的Z轴指向正上方，也就是天空。一个时间点上移动机器人的姿态相对于map不应该出现明显的漂移。如果一个map是不连续稳定的那就意味着机器人的位置在任何一个时间点上都会是变化的。
一般激光地位仪等设备会连续的计算map的位置因而造成它的不稳定性，这也使它不能成为一个好的参照体frame.

3. odom
odom是一个很好的固定世界参照frame.机器人的姿态相对odom而言是随时间是经常可变，所以在长远中它不是一个好的全局参照系。但在某一时间点而言它相对机器人的位置是不变的。
典型的 odom frame 是通过运动源来计算出来的, 例如轮子运动，视觉偏移等.odom frame 的准确性使它在局部参照系中是很有用的。但是不适合作全局参照frame.

4. base_link
base_link参照系紧紧粘在移动机器人基座上的任何一个位置和角度。

5. 各个Frames的关系
frame之间是按树状结构组织的。所以每个frame只有一个父节点和任意多个子节点。 上述几个frame的关系:
map --> odom --> base_link

6. Frame Authorities
odom到base_link的坐标转换是从运动源计算出来广播的。
map到base_link的坐标转换是被定位模块计算出来的. 但定位模块并不发布map到base_link的转换. 相反它先接受从odom到base_link的转换, 再计算并广播map到odom的位置转换关系。
fixed_frame: RViz中认定的大世界就是fixed_frame;
target_frame: Rviz中视觉跟踪的frame是 target_frame;

···
<launch>  
    <arg name="model" />  
    <arg name="gui" default="False" />  
    <param name="robot_description" textfile="$(arg model)" />  
    <param name="use_gui" value="$(arg gui)"/>  
    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />  
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />  
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find sp1s)/urdf.rviz" required="true" />  
</launch>
···

“robot_description” 参数定义了urdf文件的路径，它被 robot_state_publisher节点使用。该节点解析urdf文件后将各个frame的状态发布给tf. 因此在rviz里面就看到各个frame(link)之间的tf转换显示OK.否则会显示warning.
"joint_state_publisher"节点获取urdf里面定义的rotate link并发布坐标转换给tf.否则会显示warning. 注意:“joint_state_publisher” 是python写的，只支持ascii编码，不支持Unicode.






### 如何使用tf包来设置相对静态和动态frames之间的坐标系的变换
导航包使用tf来确定机器人在世界中的位置，并将传感器数据与静态地图相关联。然而，tf不提供关于机器人的速度的任何信息。因此，导航包要求任何odometry源通过ROS发布包含速度信息的transform和nav_msgs/Odometry消息。



## (2) 两个相对位置不变的坐标系之间可以采用如下方式进行变换
将以下代码加入到.launch文件中，args="x y z r p y patent_frame child_frame ms"， xyz表示子坐标系在父坐标系为参考下的坐标。
‘<!-- 坐标系之间关系设置 -->
<node pkg="tf" type="static_transform_publisher" name="tf_map_odom" args="0 0 0 0 0 0 map odom 100"/>
<node pkg="tf" type="static_transform_publisher" name="tf_base_camera" args="0 0 0 0 0 0 base_link camera_link 100"/>’



## Reference
http://wiki.ros.org/tf2
 <br>
https://wiki.ros.org/navigation/Tutorials/RobotSetup/TF
 <br>
 
 <br>
https://rospypi.github.io/simple/
 <br>
https://rospypi.github.io/simple/
