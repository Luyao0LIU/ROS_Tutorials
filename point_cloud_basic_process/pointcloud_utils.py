
import rospy, tf2_ros
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped, Vector3, TransformStamped
from mavros_msgs.msg import State, OverrideRCIn, PositionTarget, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import cos, sin, pi
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge, CvBridgeError
from pointcloud_utils import *
import cv2 ,math, message_filters
import pandas as pd
import open3d as o3d
import numpy as np
import scipy.io


def depth2xyz(depth_map,depth_cam_matrix,flatten=False,  save_point_coor=True, depth_scale=1):
    '''
    Referce: https://blog.csdn.net/tycoer/article/details/106761886
    '''
    fx,fy = depth_cam_matrix[0,0],depth_cam_matrix[1,1]
    cx,cy = depth_cam_matrix[0,2],depth_cam_matrix[1,2]
    h,w=np.mgrid[0:depth_map.shape[0],0:depth_map.shape[1]]
    z=depth_map/depth_scale
    x=(w-cx)*z/fx
    y=(h-cy)*z/fy
    xyz=np.dstack((x,y,z))
    if save_point_coor:
        save_point4csv(xyz,  '../image/point_csv/1.csv')
        # save_point4mat(xyz, '../image/point_mat/1.mat')
        save_point4npy(xyz,  '../image/point_npy/1.npy')
    if flatten==True:
        xyz=xyz.reshape(-1,3)
    #xyz=cv2.rgbd.depthTo3d(depth_map,depth_cam_matrix)
    return xyz


def save_pointcloud2ply(xyz, save_path):
    '''
    Referce: https://blog.csdn.net/qq_41452267/article/details/120803284
    '''
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    o3d.io.write_point_cloud(save_path, pcd)


def save_depth4image(img, img_save_path):
    scaled_img = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX)
    uint8_img = np.uint8(scaled_img)
    cv2.imwrite(img_save_path, uint8_img)


def save_depth4csv(img, csv_save_path):
    pd_data=pd.DataFrame(img)
    pd_data.to_csv(csv_save_path, index=False, header=False)


def save_point4csv(points, point_csv_save_path):
    np_points=np.empty([points.shape[0],points.shape[1]], dtype=object)
    for i in range(points.shape[0]):
        for j in range(points.shape[1]):
            np_points[i][j]=str(points[i][j])
    pd_data=pd.DataFrame(np_points)
    pd_data.to_csv(point_csv_save_path, index=False, header=False)


def save_point4mat(points, point_mat_save_path):
    '''
    Referce: https://www.codenong.com/53659234/
    '''
    print("*****points_shape:", points.shape, points[0][0], type(points[0][0]))
    np_points=np.empty([points.shape[0],points.shape[1]], dtype=object)
    for i in range(points.shape[0]):
        for j in range(points.shape[1]):
            np_points[i][j]=str(points[i][j])
    # print("*****np_points_shape:",np_points.shape)
    scipy.io.savemat(point_mat_save_path,  {'image_data': np_points} )
    print("*****End--save_point4mat*****")
    # import pdb; pdb.set_trace()


def save_point4npy(points, point_npy_save_path):
    np_points=np.array(points)
    np.save(point_npy_save_path, np_points)


def pointcloud_visual(raw_points):
    #创建点云对象
    pcd=o3d.open3d.geometry.PointCloud()
    #将点云数据转换为Open3d可以直接使用的数据类型
    pcd.points= o3d.open3d.utility.Vector3dVector(raw_points)
    #设置点的颜色为白色
    # pcd.paint_uniform_color([1,1,1])
    o3d.visualization.draw_geometries([pcd])

def Abnormal_points_filtering_xyz(xyz, z_threshold=10, save_points_path=None):
    '''
    Referce: https://blog.csdn.net/weixin_42532587/article/details/112282366
    '''
    # points = np.asarray(xyz) # xyz is <class 'numpy.ndarray'>
    pcd = pcd.select_by_index(np.where(points[:, 2] < z_threshold)[0])

    if save_points_path:
        assert type(save_points_path) is str, 'Error: save_points_path is the path to save processed points! '
        o3d.io.write_point_cloud(save_points_path, pcd, True)  # default is False to save as Binarty; True is saved as ASICC format. 
    o3d.visualization.draw_geometries([pcd])


def pointcloud_process(current_points, pointtime, cxyz):
    '''
    current_points: <sensor_msgs.msg.PointCloud2>
    pointtime: rospy.Time.now()
    '''
    # set point cloud from depth map
    current_points.header.stamp=pointtime
    current_points.header.frame_id='camera_link'
    if len(cxyz.shape) == 3:
        current_points.height = cxyz.shape[0]
        current_points.width = cxyz.shape[1]
    else:
        current_points.height = 1
        current_points.width = len(cxyz)
    points_fields=[
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]
    # PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1) # when use color
    current_points.fields=points_fields
    current_points.is_bigendian = False
    current_points.point_step = 12
    # print("*****cxyz.shape[0]:",cxyz.shape[0])
    current_points.row_step = current_points.point_step * cxyz.shape[0]
    # print("*****current_points.row_step:", current_points.row_step)
    current_points.is_dense=True
    current_points.data=np.asarray(cxyz, np.float32).tostring() # cxyz.tostring() is error.
    # or ""=np.asarray(point_data, dtype=np.float32).tobytes()
    # return
    return current_points



