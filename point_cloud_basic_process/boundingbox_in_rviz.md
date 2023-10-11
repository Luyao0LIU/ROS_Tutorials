# 点云边界框在rviz中显示

参考：Open3D点云基本变换：https://zhuanlan.zhihu.com/p/462296813
<br>
http://www.open3d.org/docs/release/python_api/open3d.geometry.OrientedBoundingBox.html#open3d.geometry.OrientedBoundingBox
<br>
Open3D坐标变换：https://blog.csdn.net/u014072827/article/details/112221093
<br>
https://blog.csdn.net/io569417668/article/details/106421206/
<br>
https://blog.csdn.net/QLeelq/article/details/122136481
<br>
Rviz可视化工具Marker示例：目标检测画矩形框<br>
https://blog.csdn.net/random_repick/article/details/110955439?spm=1001.2101.3001.6650.1&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-1-110955439-blog-116454690.235%5Ev38%5Epc_relevant_anti_t3&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-1-110955439-blog-116454690.235%5Ev38%5Epc_relevant_anti_t3&utm_relevant_index=2
<br>
https://blog.csdn.net/Fang_cheng_/article/details/116454690
<br>
https://zhuanlan.zhihu.com/p/432881120
<br>
https://blog.csdn.net/mingshili/article/details/124730585
<br>

为了方便调用写了一个函数`display_bbox_rviz_xyz`， 包含两种显示方式`CUBE`和`LINE_LIST`，根据输入的`bboxes_xyz`来进行决定，如果是边界框的8的顶点的坐标则显示线条，如果是边界狂的中心点坐标和长宽高和四元数则显示方块。

```python
def display_bbox_rviz_xyz(pointtime, current_marker_xyz, bboxes_xyz, use_bbox8points=False):
    '''
      7 -------- 4
     /|         /|
    6 -------- 5 .
    | |        | |
    . 3 -------- 0
    |/         |/
    2 -------- 1
    Referce: https://blog.csdn.net/mingshili/article/details/124730585
             https://zhuanlan.zhihu.com/p/432881120
             https://wiki.ros.org/rviz/DisplayTypes/Marker#Line_Strip_.28LINE_STRIP.3D4.29
    current_marker_xyz: MarkerArray()
    bboxes_xyz: (n,8,3)-> n is the number of bboxes. 
    '''
    lines = [[0, 1], [1, 2], [2, 3], [3, 0], [4, 5], [5, 6],
         [6, 7], [7, 4], [0, 4], [1, 5], [2, 6], [3, 7]] # 定义连接哪两个点之间的线
    current_marker_xyz.markers.clear() # 清除上一帧的边界框
    bboxes_xyz=np.expand_dims(bboxes_xyz, axis=0) # 增加一个维度，以后n个检测框是去掉这句
    for i, my_box in enumerate(bboxes_xyz):
        this_marker=Marker()
        this_marker.header.stamp=pointtime
        this_marker.header.frame_id = 'camera_link'
        this_marker.id = i  # must be unique
        this_marker.type = Marker.CUBE # CUBE：表示长方体显示；LINE_LIST：显示线条
        this_marker.action = Marker.ADD

        this_marker.color.r = 1
        this_marker.color.g = 0
        this_marker.color.b = 0
        this_marker.color.a = 0.5 # (0.0-1.0): 完全透明->完全不透明
        # this_marker.lifetime = rospy.Duration(0) # 设置多长时间后消失，0表示永不消失，前面有清除了，这里暂时不用
        # import pdb; pdb.set_trace()
        # print("*****bboxes_xyz:",i, my_box, bboxes_xyz)
        if use_bbox8points:
            this_marker.scale.x = 0.01 # controls the width of the line segments
            this_marker.points=[]
            detect_points_set = [Point(x=float(b[0]), y=float(b[1]), z=float(b[2])) for b in my_box[:8]]
            # this_marker.points.append(Point(x=float(my_box[0,0]), y=float(my_box[0,1]), z=float(my_box[0,2])))
            # this_marker.points += [Point(x=float(b[0]), y=float(b[1]), z=float(b[2])) for b in my_box[4:]]
            # this_marker.points.append(Point(x=float(my_box[4,0]), y=float(my_box[4,1]), z=float(my_box[4,2])))
            # temp = [5,1,2,6,7,3]
            # this_marker.points += [Point(x=float(my_box[i, 0]), y=float(my_box[i, 1]), z=float(my_box[i, 2])) for i in temp]
            # print("*****this_marker.points:", this_marker.points)
            for line in lines:
                this_marker.points.append(detect_points_set[line[0]])
                this_marker.points.append(detect_points_set[line[1]])
        else: # 采用中心点、长宽高、四元数
            # print("*****my_box: ",my_box)
            this_marker.pose.position.x=float(my_box[0])
            this_marker.pose.position.y=float(my_box[1])
            this_marker.pose.position.z=float(my_box[2])
            this_marker.pose.orientation.x=float(my_box[6])
            this_marker.pose.orientation.y=float(my_box[7])
            this_marker.pose.orientation.z=float(my_box[8])
            this_marker.pose.orientation.w=float(my_box[9])
            this_marker.scale.x=float(my_box[3])
            this_marker.scale.y=float(my_box[4])
            this_marker.scale.z=float(my_box[5])

        current_marker_xyz.markers.append(this_marker)
    return current_marker_xyz
```

在主文件中加入一下内容来发布话题。

```python
marker_pub = rospy.Publisher('detect_box3d', MarkerArray, queue_size=10)

pub_current_markerarray=display_bbox_rviz_xyz(pointtime=rospy.Time.now(),
                                                      current_marker_xyz=current_markerarray,
                                                      bboxes_xyz=bboxes)

marker_pub.publish(pub_current_markerarray)
```




