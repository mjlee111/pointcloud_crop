#include "../include/pointcloud_crop/pointcloud_crop.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_crop_node");
  ros::NodeHandle nh;
  ROS_INFO("Starting Pointcloud Crop Node");

  std::string sub_topic;
  ros::param::get("/pointcloud_crop_node/sub_topic", sub_topic);
  ROS_INFO("Subscribing Topic : %s", sub_topic.c_str());
  std::string pub_topic;
  ros::param::get("/pointcloud_crop_node/pub_topic", pub_topic);
  ROS_INFO("Publishing Topic : %s", pub_topic.c_str());
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
  ros::param::get("/pointcloud_crop_node/marker", marker_status);
  ROS_INFO("Marker : %s", BOOL_TO_STRING(marker_status).c_str());

  point_sub = nh.subscribe<sensor_msgs::PointCloud2>(sub_topic, 1, pointCloudCallback);
  point_pub = nh.advertise<sensor_msgs::PointCloud2>("cropped_point", 1);
  marker_pub = nh.advertise<visualization_msgs::Marker>("crop_marker", 1);

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
  marker_msg.header.frame_id = point->header.frame_id;
  if (marker_status)
  {
    publishMarker();
  }

  pcl::PassThrough<pcl::PointXYZ> pass_filter;
  pass_filter.setInputCloud(input_cloud_passthrough);
  pass_filter.setFilterFieldName("x");
  pass_filter.setFilterLimits(xyz[0], xyz[1]);
  pass_filter.setFilterLimitsNegative(false);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_x(new pcl::PointCloud<pcl::PointXYZ>);
  pass_filter.filter(*filtered_cloud_x);

  if (filtered_cloud_x->empty())
  {
    ROS_ERROR("Pointcloud Out of Range X");
    return;
  }

  pcl::PassThrough<pcl::PointXYZ> pass_filter2;
  pass_filter2.setInputCloud(filtered_cloud_x);
  pass_filter2.setFilterFieldName("y");
  pass_filter2.setFilterLimits(xyz[2], xyz[3]);
  pass_filter2.setFilterLimitsNegative(false);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_y(new pcl::PointCloud<pcl::PointXYZ>);
  pass_filter2.filter(*filtered_cloud_y);

  if (filtered_cloud_y->empty())
  {
    ROS_ERROR("Pointcloud Out of Range Y");
    return;
  }

  pcl::PassThrough<pcl::PointXYZ> pass_filter3;
  pass_filter3.setInputCloud(filtered_cloud_y);
  pass_filter3.setFilterFieldName("z");
  pass_filter3.setFilterLimits(xyz[4], xyz[5]);
  pass_filter3.setFilterLimitsNegative(false);

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
  pass_filter3.filter(*filtered_cloud_1);

  if (filtered_cloud_1->empty())
  {
    ROS_ERROR("Pointcloud Out of Range Z");
    return;
  }

  point_pub.publish(filtered_cloud_1);
}

void publishMarker()
{
  marker_msg.header.stamp = ros::Time::now();
  marker_msg.ns = "basic_shapes";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::Marker::CUBE;
  marker_msg.action = visualization_msgs::Marker::ADD;
  marker_msg.pose.position.x = (xyz[0] + xyz[1]) / 2.0;
  marker_msg.pose.position.y = (xyz[2] + xyz[3]) / 2.0;
  marker_msg.pose.position.z = (xyz[4] + xyz[5]) / 2.0;
  marker_msg.pose.orientation.x = 0.0;
  marker_msg.pose.orientation.y = 0.0;
  marker_msg.pose.orientation.z = 0.0;
  marker_msg.pose.orientation.w = 1.0;
  marker_msg.scale.x = xyz[1] - xyz[0];
  marker_msg.scale.y = xyz[3] - xyz[2];
  marker_msg.scale.z = xyz[5] - xyz[4];
  marker_msg.color.r = 1.0f;
  marker_msg.color.g = 0.0f;
  marker_msg.color.b = 0.0f;
  marker_msg.color.a = 0.5;
  marker_msg.lifetime = ros::Duration();
  marker_pub.publish(marker_msg);
}