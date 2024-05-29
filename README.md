# PointCloud Crop
This package crops <sensor_msgs::PointCloud2> in range of xyz.

### Dependencies
- PCL

### Install & Build
``` shell
$ cd ~/{your workspace}/src
$ git clone https://github.com/mjlee111/pointcloud_crop.git
$ cd ..
$ catkin_make
```

### How to use
#### run package
``` shell
$ roslaunch pointcloud_crop crop.launch
```

#### parameters
- sub_topic : topic name to subscribe. default : /pointcloud
- pub_topic : topic name to publish after crop. default : /pointcloud_crop <sensor_msgs::PointCloud2>
- x_min : x minimum range to crop. default : -1.0 [m]
- x_max : x maximum range to crop. default : 1.0 [m]
- y_min : y minimum range to crop. default : -1.0 [m]
- y_max : y maximum range to crop. default : 1.0 [m]
- z_min : z minimum range to crop. default : -1.0 [m]
- z_max : z maximum range to crop. default : 1.0 [m]

