# PointCloud Crop
The pointcloud_crop package is designed to crop a <sensor_msgs::PointCloud2> within a specified xyz range.

## Dependencies
Point Cloud Library (PCL)

## Installation & Building
To install and build the package, follow these steps:

```shell
$ cd ~/{your workspace}/src
$ git clone https://github.com/mjlee111/pointcloud_crop.git
$ cd ..
$ catkin_make
```

## Usage
To use the package, follow these instructions:

### Running the Package
Launch the package using the following command:

``` shell
$ roslaunch pointcloud_crop crop.launch
``` 
### Parameters
The package supports the following parameters:

- **sub_topic**: The topic name to subscribe to.
  - Default: `/pointcloud`
  
- **pub_topic**: The topic name to publish the cropped point cloud to.
  - Default: `/pointcloud_crop`
  
- **x_min**: The minimum x-range for cropping.
  - Default: `-1.0` [m]
  
- **x_max**: The maximum x-range for cropping.
  - Default: `1.0` [m]
  
- **y_min**: The minimum y-range for cropping.
  - Default: `-1.0` [m]
  
- **y_max**: The maximum y-range for cropping.
  - Default: `1.0` [m]
  
- **z_min**: The minimum z-range for cropping.
  - Default: `-1.0` [m]
  
- **z_max**: The maximum z-range for cropping.
  - Default: `1.0` [m]


### Example launch file (crop.launch)
Below is an example of a crop.launch file, which you can use to run the package with custom parameters:

``` xml
<launch>
    <node name="pointcloud_crop" pkg="pointcloud_crop" type="pointcloud_crop_node" output="screen">
        <param name="sub_topic" value="/pointcloud" />
        <param name="pub_topic" value="/pointcloud_crop" />
        <param name="x_min" value="-1.0" />
        <param name="x_max" value="1.0" />
        <param name="y_min" value="-1.0" />
        <param name="y_max" value="1.0" />
        <param name="z_min" value="-1.0" />
        <param name="z_max" value="1.0" />
    </node>
</launch>
``` 
This package facilitates the cropping of point cloud data within a specified range, making it easier to focus on areas of interest in your point cloud datasets. Adjust the parameters as needed to suit your specific application requirements.