#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <visualization_msgs/Marker.h>
#define BOOL_TO_STRING(value)                                                                                          \
  ([&]() -> std::string {                                                                                              \
    std::ostringstream os;                                                                                             \
    os << std::boolalpha << (value);                                                                                   \
    return os.str();                                                                                                   \
  })()

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<PointT> PointCloudT;

ros::Subscriber point_sub;
ros::Publisher point_pub;
ros::Publisher marker_pub;

float xyz[6] = {
  0.0,
};

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& point);
void publishMarker();

visualization_msgs::Marker marker_msg;
bool marker_status = false;