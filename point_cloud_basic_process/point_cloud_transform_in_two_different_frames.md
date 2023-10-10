# 坐标系的转换

### 前言

点云的坐标系：X(前)，Y(左)，Z(上)； <br>
相机的坐标系：X(右)，Y(下)，Z(前)； <br>
点云坐标系到相机坐标系(即X—>Z(0,0,1),Y—>-X(-1,0,0),Z—>-Y(0,-1,0)的旋转矩阵应该为： <br>

$$
\begin{pmatrix} 
0 & -1 & 0 \\ 
0 & 0 & -1 \\ 
1 & 0 & 0 \\ 
\end{pmatrix}
$$

相反，相机坐标系到点云坐标系(即X—>-Y(0,-1,0),Y—>-Z(0,0,-1),Z—>X(1,0,0)的旋转矩阵应该为： <br>

$$
\begin{pmatrix} 
0 & 0 & 1 \\ 
-1 & 0 & 0 \\ 
0 & -1 & 0 \\ 
\end{pmatrix}
$$

MATLAB 四元数转R旋转矩阵的转换顺序为w,x,y,z； 坐标系2到坐标系1的T转换公式为：$ T_{2- & 1} = T_{1- & 0}^{-1} * T_{2- & 0} $

### Open3D绘制3D坐标-可视化点云

参考：https://blog.csdn.net/guyuealian/article/details/102475850

```python
# -*-coding: utf-8 -*-
"""
    @Project: PyKinect2-OpenCV
    @File   : open3d_test.py
    @Author : panjq
    @E-mail : pan_jinquan@163.com
    @Date   : 2019-10-10 09:49:27
"""
import open3d
import numpy as np
import cv2
 
# 绘制open3d坐标系
axis_pcd = open3d.create_mesh_coordinate_frame(size=0.5, origin=[0, 0, 0])
# 在3D坐标上绘制点：坐标点[x,y,z]对应R，G，B颜色
points = np.array([[0.1, 0.1, 0.1], [1, 0, 0], [0, 1, 0], [0, 0, 1]])
colors = [[1, 1, 1], [1, 0, 0], [0, 1, 0], [0, 0, 1]]
 
test_pcd = open3d.geometry.PointCloud()  # 定义点云
 
# 方法1（非阻塞显示）
vis = open3d.Visualizer()
vis.create_window(window_name="Open3D1")
vis.get_render_option().point_size = 3
first_loop = True
# 先把点云对象添加给Visualizer
vis.add_geometry(axis_pcd)
vis.add_geometry(test_pcd)
while True:
    # 给点云添加显示的数据
    points -= 0.001
    test_pcd.points = open3d.utility.Vector3dVector(points)  # 定义点云坐标位置
    test_pcd.colors = open3d.Vector3dVector(colors)  # 定义点云的颜色
    # update_renderer显示当前的数据
    vis.update_geometry()
    vis.poll_events()
    vis.update_renderer()
    cv2.waitKey(100)
 
# 方法2（阻塞显示）：调用draw_geometries直接把需要显示点云数据
test_pcd.points = open3d.utility.Vector3dVector(points)  # 定义点云坐标位置
test_pcd.colors = open3d.Vector3dVector(colors)  # 定义点云的颜色
open3d.visualization.draw_geometries([test_pcd] + [axis_pcd], window_name="Open3D2")
```
### ROS中将点云从相机坐标系转到世界坐标系(已知二者TF)

**TF获得坐标系转换关系**
首先通过tf获得trans和quat。指以/world为参考系，从/world到/kinect2_ir_optical_frame需要的平移向量trans与旋转四元数quat。
注：得到的quat是xyzw的顺序，一般我们使用的是wxyz（用作转欧拉角等操作）。

![image](https://github.com/Luyao0LIU/ROS_Tutorials/assets/128677149/840e5ad9-6713-4dc2-ade5-b6a3622e8a4f)

```python
   listener = tf.TransformListener()
    get_transform = False
    while not get_transform:
        try:
            trans, quat = listener.lookupTransform('/world', '/kinect2_ir_optical_frame', rospy.Time(0))
            # print(trans, quat) ## quat xyzw
            # [0.8420404691331246, -0.04343000000000001, 1.5380334678129328], 
            # [-0.7071067251356604, 0.7071067247627317, -0.00028154403091668057, 0.00028248221651117067] xyzw
            # 转为wxyz形式
            quat_wxyz[0] = quat[3]
			quat_wxyz[1] = quat[0]
			quat_wxyz[2] = quat[1]
			quat_wxyz[3] = quat[2]

```

**点云坐标系转换**
这里我所使用的点云为Numpy格式，shape为（500，3）。转换的思路为 P’ = R×P+T，因为点云P相对为行向量，所以实际为P’=P×R+T（这里我理解的是，平时我们处理点旋转时，点一般为列向量，所以旋转矩阵左乘R×P，但这里为行向量，所以右乘）

`1.将四元数转换为旋转矩阵`
```python
def quat2rot(quat_):
	q = quat_
	n = np.dot(q, q)
	if n < np.finfo(q.dtype).eps:
		return np.identity(3)
	q = q * np.sqrt(2.0 / n)
	q = np.outer(q, q)
	rot_matrix = np.array(
	[[1.0 - q[2, 2] - q[3, 3], q[1, 2] + q[3, 0], q[1, 3] - q[2, 0]],
	 [q[1, 2] - q[3, 0], 1.0 - q[1, 1] - q[3, 3], q[2, 3] + q[1, 0]],
	 [q[1, 3] + q[2, 0], q[2, 3] - q[1, 0], 1.0 - q[1, 1] - q[2, 2]]],
	dtype=q.dtype)

	return rot_matrix
	# [[ 1.05656190e-09 -1.00000000e+00 -1.32679448e-06]
    # [-9.99999682e-01  1.76031933e-12 -7.97653505e-04]
    # [ 7.97653505e-04  1.32679490e-06 -9.99999682e-01]]

```

`2.点云变换`

```python
points_ = np.dot(points_, rotation_matrix) + np.array(cam_wor)
```

实验结果：

![image](https://github.com/Luyao0LIU/ROS_Tutorials/assets/128677149/fdd05f9e-5c24-4957-8bc3-a1f0ce7abeb6)






## 参考

https://blog.csdn.net/a17381562089/article/details/115127318
<br>
https://blog.csdn.net/tycoer/article/details/124509088?spm=1001.2101.3001.6650.1&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-1-124509088-blog-115127318.235%5Ev38%5Epc_relevant_anti_vip&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-1-124509088-blog-115127318.235%5Ev38%5Epc_relevant_anti_vip&utm_relevant_index=2





