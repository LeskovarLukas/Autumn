#include "ros/ros.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sstream>
#include "nav_msgs/Path.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include <iostream>
#include "PathPlanning.h"
#include <pcl/common/projection_matrix.h>

geometry_msgs::PoseStamped currentPose;
geometry_msgs::PointStamped goal;
pcl::PointCloud<pcl::PointXYZ> cloud;
bool activePathPlanning = false;
PathPlaning *pp = nullptr;
void pathcallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  geometry_msgs::PoseStamped poseData = *msg;
  currentPose = poseData;
  //std::cout << poseData.pose.position << std::endl;
}

void pointClickedcallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  geometry_msgs::PointStamped goalPoint = *msg;
  goal = goalPoint;
  //              ZED Position     GOAL Position   D    i
  pp->getPath(currentPose.pose, goalPoint.point, cloud, 12, 10);
}

void cloud2dcallback(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
  cloud = *temp_cloud;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "explorator");
  ros::NodeHandle n;
  //                      min max
  pp = new PathPlaning(n, 4, 12);
  std::cout << "subscribing to pose" << '\n';
  ros::Subscriber pathSub = n.subscribe("/zedi/zed_node/pose", 1, &pathcallback);
  std::cout << "subscribing to point" << '\n';
  ros::Subscriber goalSub = n.subscribe("/clicked_point", 1, &pointClickedcallback);
  std::cout << "subscribing to cloud" << '\n';
  ros::Subscriber cloudSub = n.subscribe("/zedi/cloud_map", 1, &cloud2dcallback);

  ros::spin();
  return 0;
}
