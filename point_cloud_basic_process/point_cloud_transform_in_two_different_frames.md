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




