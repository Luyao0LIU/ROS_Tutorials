# TF corridates transform
This file introducts how to use tf to transform among frames such as odom, base_link, camera_link and so on.

## (1) Odometry的发布和发布odom到base_link的tf变换
https://blog.csdn.net/sinat_16643223/article/details/113919768
<br>
https://blog.csdn.net/qq_43481884/article/details/105429655
<br>
还可以参考经典的小乌龟例程来学习tf坐标变换
<br>
https://blog.csdn.net/qq_42731062/article/details/125926225
<br>
https://zhuanlan.zhihu.com/p/395314257
<br>
http://wiki.ros.org/tf/Tutorials/Time%20travel%20with%20tf%20%28Python%29
<br>
https://www.guyuehome.com/35890
<br>


**RVIZ坐标系**

X轴--红色 Y轴---绿色 Z轴---蓝色
YAW(偏航角)绕Z轴旋转
PITCH(俯仰角)绕Y轴旋转
ROLL(滚转角)绕X轴旋转
符合右手坐标系原则


### 坐标变换简介
  坐标转换是空间实体的位置描述，是从一种坐标系统变换到另一种坐标系统的过程。通过建立两个坐标系统之间一一对应关系来实现。通常坐标转换有平移、缩放、旋转三个方面的转换。

![image](https://github.com/Luyao0LIU/ROS_Tutorials/assets/128677149/40f2dbe4-2dbd-4d10-8191-1a761a2f5342)

A、B两个坐标系A坐标系下的位姿可以通过平移和旋转变换成B坐标系下的位姿这里的平移和旋转可以通过4×4的变换矩阵来描述。

**TF坐标变换如何实现？**
监听TF变换：接收并缓存系统中发布的所有坐标变换数据并从中查询所需要的坐标变换关系。

广播TF变换：向系统中广播坐标系之间的坐标变换关系。系统中可能会存在多个不同部分的TF变换广播每个广播都可以直接将坐标变换关系插入TF树中不需要再进行同步。


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

```xml
<launch>  
    <arg name="model" />  
    <arg name="gui" default="False" />  
    <param name="robot_description" textfile="$(arg model)" />  
    <param name="use_gui" value="$(arg gui)"/>  
    <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" />  
    <node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />  
    <node name="rviz" pkg="rviz" type="rviz" args="-d $(find sp1s)/urdf.rviz" required="true" />  
</launch>
```

“robot_description” 参数定义了urdf文件的路径，它被 robot_state_publisher节点使用。该节点解析urdf文件后将各个frame的状态发布给tf. 因此在rviz里面就看到各个frame(link)之间的tf转换显示OK.否则会显示warning.
"joint_state_publisher"节点获取urdf里面定义的rotate link并发布坐标转换给tf.否则会显示warning. 注意:“joint_state_publisher” 是python写的，只支持ascii编码，不支持Unicode.

### TF工具 如何查看当前各个frames之间的转换关系

可以使用ROS官方自带的工具，运行如下命令

**tf_monitor**

tf_monitor工具的功能是打印TF树中所有坐标系的发布状态

`tf_monitor <source_frame> <target_target>`

tf_monitor工具查看TF树中所有坐标系的发布状态

**tf_echo**

tf_echo工具的功能是查看指定坐标系之间的变换关系

`tf_echo <source_frame> <target_frame>`

**static_transform_publisher**

static_transform_publisher工具的功能是发布两个坐标系之间的静态坐标变换这两个坐标系不发生相对位置变化

`static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period_in_ms`

`static_tra nsform_publisher x y z qx qy qz qw frame_id child_frame_id period_in_ms`

该命令不仅可以在终端中使用还可以在launch文件中使用

```xml
<launch>
  <node pk g="tf" type="static_transform_publisher" name="link1_broadcaster"args="1 0 0 0 0 0 1 link1_parent link1 100" />
</launch>
```

**view_frames**

view_frames是可视化的调试工具可以生成pdf文件显示整棵TF树的信息。

```xml
rosrun tf2_tools view_frames.py
```

使用如下命令或者使用pdf阅读器查看生成的pdf文件

```xml
evince frames.pdf
```

![image](https://github.com/Luyao0LIU/ROS_Tutorials/assets/128677149/6b56a77e-9e82-4d99-929d-78c49f69c3e8)



### 如何使用tf包来设置相对静态和动态frames之间的坐标系的变换
导航包使用tf来确定机器人在世界中的位置，并将传感器数据与静态地图相关联。然而，tf不提供关于机器人的速度的任何信息。因此，导航包要求任何odometry源通过ROS发布包含速度信息的transform和nav_msgs/Odometry消息。



## (2) 两个相对位置不变的坐标系之间可以采用如下方式进行变换
**方法1：**
将以下代码加入到.launch文件中，`args="x y z r p y patent_frame child_frame ms"`， xyz表示子坐标系在父坐标系为参考下的坐标。
```xml
<!-- 坐标系之间关系设置 -->
<node pkg="tf" type="static_transform_publisher" name="tf_map_odom" args="0 0 0 0 0 0 map odom 100"/>
<node pkg="tf" type="static_transform_publisher" name="tf_base_camera" args="0 0 0 0 0 0 base_link camera_link 100"/>
```

**方法2：**
有时候后静态转换是固定不变的，我们可以在程序开始运行的时候发布这个静态的转换，例如从标准的相机系到机体系的转换。我们根本就没有必要再写上面这样一段程序，ros为我们提供了现成的包。可以在命令行或者launch文件中实现。终端方式后面讲。
格式如下：

`static_transform_publisher x y z yaw pitch roll frame_id child_frame_id`

还可以通过四元数的方式来实现。

`static_transform_publisher x y z qx qy qz qw frame_id child_frame_id`

在相应的launch文件夹内新建一个名为`static_transforms.launch`的launch文件，然后写如下面的内容保存退出。

```xml
<launch>
    <node pkg="tf2_ros" type="static_transform_publisher" name="link1_broadcaster" 
    args="1 0 0 0 0 0 1 link1_parent link1" />
</launch>
```
**方法3：**
利用终端的方式实现
因为`launch`文件不会管各个节点的启动顺序，所以我们可以利用终端的方式来运行。
首先打开一个终端运行

```xml
roscore
```
然后再开一个终端运行静态的坐标转换，格式和launch文件一样。

```xml
rosrun tf2_ros static_transform_publisher 0 0 1 0 0 0 link1_parent link1
```

或者
用tf包工具发布global fixed frame到topic所在坐标系的tf关系，例如：

```xml
rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 map xxx 100 ;//将xxx映射为map
```

# (3) 在python文件中定义动态的tf坐标变换

参考：
https://zhuanlan.zhihu.com/p/395314257
<br>


```python

# NOTE: This experiment is vel test.
import rospy, tf2_ros
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped, Vector3, TransformStamped
from mavros_msgs.msg import State, OverrideRCIn, PositionTarget, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import cos, sin, pi
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge, CvBridgeError
import cv2 ,math, message_filters
import pandas as pd
import open3d as o3d
import numpy as np
import scipy.io

# File Path
read_path='../datasets/export_uav_data_0_05.csv'
# read_path='../datasets/takeoff_uav_data1.csv'
uav_data=pd.read_csv(read_path)
intems=np.array([[337.2084410968044, 0.0, 320.5], [0.0, 337.2084410968044, 240.5], [0.0, 0.0, 1.0]])

# Global variables
current_odom=Odometry()
current_points=PointCloud2()

# TF Transform
tf_odom_base_link=TransformStamped()
tf_odom_camera_link=TransformStamped()


# Setpoint Position
desired_pose = PoseStamped()
desired_pos_target=PositionTarget()

# Callbacks
def odom_callback(msg):
    global current_odom
    current_odom = msg

def depth_callback(msg):
    global current_depth, depth_control,pointxyz
    dp_img = bridge.imgmsg_to_cv2(msg, '32FC1')
    if start_dep and depth_control: # start depth image
        # print("*****", dp_img.shape) # (480, 640) -- H,W
        # save_depth4image(dp_img, '../image/depth/2.png')
        # save_depth4csv(dp_img, '../image/depth_csv/2.csv') # cost a lot of time
        pointxyz=depth2xyz(dp_img, intems, save_point_coor=False,flatten=True)
        # print("*****cxyz_type_shape:", type(cxyz), cxyz.shape) # <class 'numpy.ndarray'> (307200, 3)
        # *****cxyz_type_shape: <class 'numpy.ndarray'> (480, 640, 3)
        # save_pointcloud2ply(xyz, save_path='../pointcloud/1.ply')
        # pointcloud_visual(xyz)
        depth_control=2
        # print("*****CTRL + C*****")

# Main function
def main():
    rospy.init_node('lly_pub_pointcloud_py')

    # make a video_object and init the video object
    global count,bridge, start_dep, depth_control
    start_dep=0
    depth_control=1
    count = 0
    bridge = CvBridge()
    br = tf2_ros.TransformBroadcaster()
    rospy.Subscriber('/d435/depth/image_rect_raw', Image, depth_callback)
    rospy.Subscriber('/mavros/local_position/odom', Odometry, odom_callback)

    # rospy.spin()
    print("************stil run*************")

    # Subscribers
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
    rospy.Subscriber('/mavros/local_position/velocity_local', TwistStamped, velocity_callback)
    rospy.Subscriber('/mavros/state', State, state_callback)

    # Publishers
    position_target_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
    position_acc_pub=rospy.Publisher('/mavros/setpoint_accel/accel', Vector3Stamped, queue_size=10)
    attitude_pub=rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
    pc_pub=rospy.Publisher('pointcloud_pub',PointCloud2, queue_size=10)

    # Service clients
    set_mode_client = rospy.ServiceProxy('/mavros/set_mode', SetMode)
    arming_client = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)

    # Rectangle parameters
    altitude = 1.5  # Altitude
    world_corridinate_x=8.5
    world_corridinate_y=-1.0
    world_corridinate_z=0.0

    rate = rospy.Rate(20)  # Update rate
    # Waypoints (rectangle vertices)
    # waypoints = [[-8.5, 1.0], [-8.0, 1.0], [-7.0, 1.0], [-6.0, 1.0], [-5.0, 1.0], [-4.0, 1.0], 
    # [-3.0, 1.0], [-2.0, 1.0]]
    waypoints = [[-8.5, 1.0]]

    # SET MESSAGE
    desired_pose.pose.position.x = 0.0
    desired_pose.pose.position.y = 0.0
    desired_pose.pose.position.z = altitude


    # Wait for connection
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(desired_pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True


    last_req = rospy.Time.now()
    # Main loop
    index = 0  # Waypoint index
    uav_index=0 # uav data index
    while not rospy.is_shutdown():
        if(index<len(waypoints)):
            waypoint_x, waypoint_y = waypoints[index]
            desired_pose.pose.position.x = waypoint_x+world_corridinate_x
            desired_pose.pose.position.y = waypoint_y+world_corridinate_y
            desired_pose.pose.position.z = altitude+world_corridinate_z
            # New
            desired_pos_target.position.x=desired_pose.pose.position.x
            desired_pos_target.position.y=desired_pose.pose.position.y
            desired_pos_target.position.z=desired_pose.pose.position.z
            desired_pos_target.coordinate_frame=PositionTarget.FRAME_LOCAL_NED
            desired_pos_target.type_mask=PositionTarget.IGNORE_VX|PositionTarget.IGNORE_VY|PositionTarget.IGNORE_VZ|PositionTarget.IGNORE_AFX|PositionTarget.IGNORE_AFY|PositionTarget.IGNORE_AFZ|PositionTarget.IGNORE_YAW|PositionTarget.IGNORE_YAW_RATE
        else:
            desired_pose.pose.position.x = -8.5+world_corridinate_x
            desired_pose.pose.position.y = 1.0+world_corridinate_y
            desired_pose.pose.position.z = altitude+world_corridinate_z
            index=uav_data.shape[0]+1
            # pass

        # TF corridates transform
        tf_odom_base_link.header.frame_id="odom"
        tf_odom_base_link.child_frame_id="base_link"
        tf_odom_base_link.header.stamp=rospy.Time.now()
        tf_odom_base_link.transform.translation.x=current_odom.pose.pose.position.x
        tf_odom_base_link.transform.translation.y=current_odom.pose.pose.position.y
        tf_odom_base_link.transform.translation.z=current_odom.pose.pose.position.z
        tf_odom_base_link.transform.rotation.x=current_odom.pose.pose.orientation.x
        tf_odom_base_link.transform.rotation.y=current_odom.pose.pose.orientation.y
        tf_odom_base_link.transform.rotation.z=current_odom.pose.pose.orientation.z
        tf_odom_base_link.transform.rotation.w=current_odom.pose.pose.orientation.w


        if depth_control==2:
            pub_current_points=pointcloud_process(current_points,pointtime=rospy.Time.now(), 
                                              cxyz=pointxyz)

        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")

                last_req = rospy.Time.now()

        distance_to_waypoint = ((current_pose.pose.position.x - (waypoint_x+world_corridinate_x)) ** 2 +
                                (current_pose.pose.position.y - (waypoint_y+world_corridinate_y)) ** 2+
                                (current_pose.pose.position.z - (altitude+world_corridinate_z)) ** 2) ** 0.5
        # print("++++++++++++++++",distance_to_waypoint)
        if distance_to_waypoint < 0.1:
            index += 1  # Move to the next waypoint

        if(index>=len(waypoints)):
            if(uav_index<uav_data.shape[0]):
                start_dep=1
            else: 
                pass
        else:
            pass

        # Publish various topics; Note: the order of publish topics is important.
        position_target_pub.publish(desired_pos_target)
        br.sendTransform(tf_odom_base_link)
        # br_camera.sendTransform(tf_odom_camera_link)
        if depth_control==2:
            pc_pub.publish(pub_current_points)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
```





## Reference
http://wiki.ros.org/tf2
 <br>
https://wiki.ros.org/navigation/Tutorials/RobotSetup/TF
 <br>
 
 <br>
https://rospypi.github.io/simple/
 <br>
https://rospypi.github.io/simple/
