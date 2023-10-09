### (1) 点云(PointCloud2)的发送与接收--python

点云通常是通过深度相机（RGB-D）或激光雷达（lidar）等传感器产生，是空间3D点的集合。点云一般应用在需要知道深度信息（或者说是距离）的场景，例如自动驾驶时需要获取前车的距离。普通的的双目视觉算法也可以计算出深度，但相对复杂且计算量大，通过增加深度传感器即可实时获取深度信息，提高系统的实时性。

运行环境

Ubuntu 20.04

ROS Noetic

`点云发送`

```python
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import numpy as np

def talker():

    pub = rospy.Publisher('pointcloud_topic', PointCloud2, queue_size=5)
    rospy.init_node('pointcloud_publisher_node', anonymous=True)
    rate = rospy.Rate(1)

    points=np.array([[225.0, -71.0, 819.8],[237.0, -24.0, 816.0],[254.0, -82.0, 772.3]])

    while not rospy.is_shutdown():

        msg = PointCloud2()
        msg.header.stamp = rospy.Time().now()
        msg.header.frame_id = "livox_frame"

        if len(points.shape) == 3:
            msg.height = points.shape[1]
            msg.width = points.shape[0]
        else:
            msg.height = 1
            msg.width = len(points)

        msg.fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1)]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * points.shape[0]
        msg.is_dense = False
        msg.data = np.asarray(points, np.float32).tostring()

        pub.publish(msg)
        print("published...")
        rate.sleep()

if __name__ == '__main__':     

    talker()
```

`点云接收`

```python
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

class PointCloudSubscriber(object):
    def __init__(self) -> None:
        self.sub = rospy.Subscriber("pointcloud_topic",
                                     PointCloud2,
                                     self.callback, queue_size=5)
    def callback(self, msg):
        assert isinstance(msg, PointCloud2)

        # gen=point_cloud2.read_points(msg,field_names=("x","y","z"))
        points = point_cloud2.read_points_list(
            msg, field_names=("x", "y", "z"))
        print(points)


if __name__ =='__main__':
    rospy.init_node("pointcloud_subscriber")
    PointCloudSubscriber()
    rospy.spin()
```

## C++完整版本

```c++
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

class pointcloud_pub_sub
{
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    std::string pointcloud_topic = "cloudpoint_topic";

    void initializePublishers();
    void initializeSubscribers();
    void callback(const sensor_msgs::PointCloud2ConstPtr &cloud);

public:
    pointcloud_pub_sub(ros::NodeHandle *node);
    ~pointcloud_pub_sub();
    void publish();
};

pointcloud_pub_sub::pointcloud_pub_sub(ros::NodeHandle *node) : nh(*node)
{
    initializePublishers();
    initializeSubscribers();
}

pointcloud_pub_sub::~pointcloud_pub_sub()
{
}

void pointcloud_pub_sub::initializePublishers()
{
    pub = nh.advertise<sensor_msgs::PointCloud2>(pointcloud_topic, 10);
}

void pointcloud_pub_sub::initializeSubscribers()
{
    sub = nh.subscribe(pointcloud_topic, 10, &pointcloud_pub_sub::callback, this);
}

void pointcloud_pub_sub::callback(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud, *temp_cloud);
    // or
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

    //do stuff
    ROS_INFO("received %ld points", temp_cloud->points.size());

}

void pointcloud_pub_sub::publish()
{
    sensor_msgs::PointCloud2 msg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < 3; i++)
    {
        pcl::PointXYZ p(1.0 * i, 2.0 * i, 3.0 * i);
        cloud->push_back(p);
    }

    pcl::toROSMsg(*cloud, msg);
    // or
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*cloud, pcl_pc2);
    pcl_conversions::fromPCL(pcl_pc2, msg);

    // publish
    pub.publish(msg);
    ROS_INFO("published.");
}
```

总结

pcl库是一个强大的点云处理库，在c++中使用极其方便，而python版本中因为笔者没有安装python-pcl（也是为了追求多样性），代码看起来略显繁杂。

同时，c++版本中也展示了ros中sensor_msgs::PointCloud2格式与pcl中pcl::PointCloud、pcl::PointCloud的相互转换方法，请读者留意。

参考：https://blog.csdn.net/k_NGU_L/article/details/119541055






