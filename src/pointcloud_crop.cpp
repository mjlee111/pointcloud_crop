#include "../include/pointcloud_crop/pointcloud_crop.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_crop_node");
  ros::NodeHandle nh;
  ROS_INFO("Starting Pointcloud Crop Node");

  std::string topic;
  ros::param::get("/pointcloud_crop_node/topic", topic);
  ROS_INFO("Subscribing Topic : %s", topic.c_str());
  ros::param::get("/pointcloud_crop_node/x_min", xyz[0]);
  ROS_INFO("X MIN : %f", xyz[0]);
  ros::param::get("/pointcloud_crop_node/x_max", xyz[1]);
  ROS_INFO("X MAX : %f", xyz[1]);
  ros::param::get("/pointcloud_crop_node/y_min", xyz[2]);
  ROS_INFO("Y MIN : %f", xyz[2]);
  ros::param::get("/pointcloud_crop_node/y_max", xyz[3]);
  ROS_INFO("Y MAX : %f", xyz[3]);
  ros::param::get("/pointcloud_crop_node/z_min", xyz[4]);
  ROS_INFO("Z MIN : %f", xyz[4]);
  ros::param::get("/pointcloud_crop_node/z_max", xyz[5]);
  ROS_INFO("Z MAX : %f", xyz[5]);

  point_sub = nh.subscribe<sensor_msgs::PointCloud2>("topic", 1, pointCloudCallback);
  point_pub = nh.advertise<sensor_msgs::PointCloud2>("cropped_point", 1);

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& point)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_passthrough(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*point, *input_cloud_passthrough);

  pcl::PassThrough<pcl::PointXYZ> pass_filter;
  pass_filter.setInputCloud(input_cloud_passthrough);
  pass_filter.setFilterFieldName("x");
  pass_filter.setFilterLimits(xyz[0], xyz[1]);
  pass_filter.setFilterLimitsNegative(false);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_x(new pcl::PointCloud<pcl::PointXYZ>);
  pass_filter.filter(*filtered_cloud_x);

  if (filtered_cloud_x->empty())
    return;

  pcl::PassThrough<pcl::PointXYZ> pass_filter2;
  pass_filter2.setInputCloud(filtered_cloud_x);
  pass_filter2.setFilterFieldName("y");
  pass_filter2.setFilterLimits(xyz[2], xyz[3]);
  pass_filter2.setFilterLimitsNegative(false);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_y(new pcl::PointCloud<pcl::PointXYZ>);
  pass_filter2.filter(*filtered_cloud_y);

  if (filtered_cloud_y->empty())
    return;

  pcl::PassThrough<pcl::PointXYZ> pass_filter3;
  pass_filter3.setInputCloud(filtered_cloud_y);
  pass_filter3.setFilterFieldName("z");
  pass_filter3.setFilterLimits(xyz[4], xyz[5]);
  pass_filter3.setFilterLimitsNegative(false);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
  pass_filter3.filter(*filtered_cloud_1);

  if (filtered_cloud_1->empty())
  {
    return;
  }

  point_pub.publish(filtered_cloud_1);
}
