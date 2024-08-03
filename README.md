# 2024年RoboMaster哨兵教程

​																																 

```yaml
author:  yangdong
```



## 一、ROS导航基础框架搭建



### 1.1.构建TF树坐标变换





### 1.2.搭建AMCL+move_base导航框架v1.0





### 1.3.加入imu_tools+robot_pose_ekf导航框架v2.0





### 1.4.融合odom去除雷达畸变构建点云数据-导航框架v3.0





## 二、双雷达

大致思路：

​	双雷达驱动发布雷达数据----->过滤车体自身遮挡数据----->2维雷达scan数据转3维点云数据----->运用点云库进行点云数据的融合----->点云数据转回雷达数据



### 2.1.驱动双雷达并发布坐标变换



### 2.2.雷达数据过滤：

**laser-filters安装**：

1.首先，打开终端并导航到您的ROS工作区（通常是`catkin_ws/src`目录）：

```bash
cd ~/catkin_ws/src
```

2. 使用Git克隆laser-filters软件包的仓库：

```bash
git clone https://github.com/ros-perception/laser_filters.git
```

3. 进入laser-filters目录：

```bash
cd laser_filters
```

4. 查看可用的版本列表：

```bash
git tag -l
```

5. 选择您想要安装的具体版本，并使用git检出该版本：

```bash
git checkout <版本号>
```

请将`<版本号>`替换为您想要安装的laser-filters版本号。

6. 返回到您的ROS工作区根目录并构建软件包：

```bash
cd ~/catkin_ws
catkin_make
```

7. 激活您的ROS工作区：

```bash
source ~/catkin_ws/devel/setup.bash
```

**功能包使用**

laser-filters有很多过滤方式，这里我们使用box过滤车体障碍信息



### 2.3.2D雷达信息转点云

对应的C++和Python代码如下：

C++:

```c++
#include"ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include"sensor_msgs/PointCloud.h"
#include"laser_geometry/laser_geometry.h"

laser_geometry::LaserProjection projector;
ros::Publisher scan_pub;

void ScanCallback(const sensor_msgs::LaserScan::ConstPtr &scan_in)
{
  	sensor_msgs::PointCloud cloud;
  	projector.projectLaser(*scan_in,cloud);
  	scan_pub.publish(cloud);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "laser_scan_to_pointcloud");
    ros::NodeHandle nh;
    scan_pub=nh.advertise<sensor_msgs::PointCloud>("/scan/point_cloud",1000);
	ros::Subscriber sub = nh.subscribe("/scan", 1000, ScanCallback);
 
	ros::spin();                                        
	return 0;
}
```

Pthon:

```python
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2 as pc2
from sensor_msgs.msg import LaserScan
from laser_geometry import LaserProjection

class Laser2PC():
    def __init__(self):
        self.laserProj = LaserProjection()
        self.pcPub = rospy.Publisher("/frontlaserPointCloud", pc2, queue_size=10) #转变后发		   布的点云话题
        self.laserSub = rospy.Subscriber("front_laser/scan", LaserScan, 						self.laserCallback) #接收到的雷达消息

    def laserCallback(self,data):
     
        cloud_out = self.laserProj.projectLaser(data)
     
        self.pcPub.publish(cloud_out)

if __name__ == '__main__':
    rospy.init_node("frontlaser2PointCloud")
    l2pc = Laser2PC()
    rospy.spin()
```



### 2.4.点云数据融合

这样ros里面建图导航都可以用了。

代码如下：

```c++
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <iostream>
 
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
class SubscribeAndPublish
{
private:
	ros::NodeHandle n;
	ros::Publisher pub;
  sensor_msgs::PointCloud2 laser_finalMsg;
  pcl::PointCloud<pcl::PointXYZI> front_back_cloud_pcl;
	message_filters::Subscriber<sensor_msgs::PointCloud2>* sub_front;
	message_filters::Subscriber<sensor_msgs::PointCloud2>* sub_back;
	
 
public:
    SubscribeAndPublish(){
		pub = n.advertise<sensor_msgs::PointCloud2>("/Addpoints2", 10); //定义两个点云数据融合后发布
		sub_front = new message_filters::Subscriber<sensor_msgs::PointCloud2>(n,  "/point_cloud1", 2000); //接收前雷达转换后的点云数据
		sub_back = new message_filters::Subscriber<sensor_msgs::PointCloud2>(n, "/point_cloud2", 2000);  //接收后雷达转换后的点云数据
		
		typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> syncPolicy;
		message_filters::Synchronizer<syncPolicy> sync(syncPolicy(1000), *sub_front, *sub_back);
		sync.registerCallback(boost::bind(&SubscribeAndPublish::callback, this,  _1, _2));
 
		ros::spin();
	}
    //回调函数
    void callback(const sensor_msgs::PointCloud2::ConstPtr& frontlaserPointCloud, const sensor_msgs::PointCloud2::ConstPtr& backlaserPointCloud){
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_frontPointCloud(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*frontlaserPointCloud, *pcl_frontPointCloud);  //将接收到的前雷达点云数据转换成pcl格式的点云数据
 
      pcl::PointCloud<pcl::PointXYZI>::Ptr front_calibration_cloud(new pcl::PointCloud<pcl::PointXYZI>()); //定义一个新的前雷达的点云数据
      Eigen::Matrix4f g_front_calibration_matrix = Eigen::Matrix4f::Identity();   //变换矩阵
				g_front_calibration_matrix(0, 0) = 1; 
				g_front_calibration_matrix(0, 1) = 0;
				g_front_calibration_matrix(0, 2) = 0;
				g_front_calibration_matrix(0, 3) = 0;
				g_front_calibration_matrix(1, 0) = 0; 
				g_front_calibration_matrix(1, 1) = 1;
				g_front_calibration_matrix(1, 2) = 0;
				g_front_calibration_matrix(1, 3) = 0;
				g_front_calibration_matrix(2, 0) = 0;
				g_front_calibration_matrix(2, 1) = 0;
				g_front_calibration_matrix(2, 2) = 1;
				g_front_calibration_matrix(2, 3) = 0;
                g_front_calibration_matrix(3, 0) = 0;
				g_front_calibration_matrix(3, 1) = 0;
				g_front_calibration_matrix(3, 2) = 0;
				g_front_calibration_matrix(3, 3) = 1;
      
 
      pcl::PointCloud<pcl::PointXYZI>::Ptr back_calibration_cloud(new pcl::PointCloud<pcl::PointXYZI>());  //定义一个新的后雷达的点云数据
      Eigen::Matrix4f g_back_calibration_matrix = Eigen::Matrix4f::Identity();    //后雷达的变换矩阵
	  float theta = M_PI; 
	  Eigen::Matrix4f transform_pitch;//俯仰
      Eigen::Matrix4f transform_roll;//翻转
      Eigen::Matrix4f transform_yaw;//偏航
	  			transform_pitch << 1, 0, 0, 0, \
                        0, cos(0), -sin(0), 0, \
                        0, sin (0), cos (0), 0, \
                        0, 0, 0, 1;
				transform_roll << cos (0), 0, sin(0), 0, \
                     0, 1, 0, 0, \
                     -sin (0), 0, cos (0), 0, \
                     0, 0, 0, 1;
				transform_yaw << cos (theta), -sin(theta), 0, 0, \
                     sin (theta), cos (theta), 0, 0, \
                     0, 0, 1, 0, \
                     0, 0, 0, 1;
				g_back_calibration_matrix = transform_yaw*transform_roll*transform_pitch;    //后雷达完整的变换矩阵
				g_back_calibration_matrix (0,3) = -0.5;  //x轴的偏移
   				g_back_calibration_matrix (1,3) = 0;  //y轴的偏移
   				g_back_calibration_matrix (2,3) = 0;  //z轴的偏移
				
      pcl::transformPointCloud(*pcl_frontPointCloud, *front_calibration_cloud, g_front_calibration_matrix);  //将*pcl_frontPointCloud变换为*front_calibration_cloud，变换矩阵为g_front_calibration_matrix。
      for (std::size_t i = 0; i < front_calibration_cloud->size(); ++i)
      {
           front_calibration_cloud->points[i].intensity = 64;
      }      
 
      //back 雷达
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_backPointCloud(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*backlaserPointCloud, *pcl_backPointCloud);
 
      
      pcl::transformPointCloud(*pcl_backPointCloud, *back_calibration_cloud, g_back_calibration_matrix);
      for (std::size_t i = 0; i < back_calibration_cloud->size(); ++i)
      {
           back_calibration_cloud->points[i].intensity = 128;
      }
 
      pcl::PointCloud<pcl::PointXYZI>::Ptr front_back_middle_calibration_cloud(new pcl::PointCloud<pcl::PointXYZI>());  //定义两个点云数据融合后的新点云数据
      *front_back_middle_calibration_cloud = *front_calibration_cloud + *back_calibration_cloud;  //两个点云数据融合
      front_back_cloud_pcl = *front_back_middle_calibration_cloud;
      pcl::toROSMsg(front_back_cloud_pcl, laser_finalMsg); //将pcl格式的点云数据转换成ros可以接收到的点云格式
      laser_finalMsg.header.frame_id = "laser"; //加入这句话
      pub.publish(laser_finalMsg);   //发布转换后的点云数据。
        }
 
};
 
int main(int argc, char** argv){
	ros::init(argc, argv, "AddpointCloud");
 
	SubscribeAndPublish sap;
    ros::spin();
	return 0;
}
```



## 三、通信部分

### 3.1 串口通信协议

向下位机发

底盘数据

| 帧头      | 帧号 | 数据1 | 数据2 | 数据3 | 校验位 |
| --------- | ---- | ----- | ----- | ----- | ------ |
| 0xAA 0x55 | 0x01 | x     | y     | yaw   | 0x66   |

云台数据

| 帧头      | 帧号 | 数据1 | 数据2 | 数据3   | 校验位 |
| --------- | ---- | ----- | ----- | ------- | ------ |
| 0xAA 0x44 | 0x02 | pitch | yaw   | command | 0x77   |

```
帧号决定数据类型：
	底盘数据：0x01
	云台数据：0x02
	
数据帧发送均位先发送高8位后发送低八位

```

### 3.2 TCP通信

TCP通信主要用于ROS与视觉进行通信：



## 四、后续任务



## 五、结语

