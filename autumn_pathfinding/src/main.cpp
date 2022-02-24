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
#include "spdlog/spdlog.h"

geometry_msgs::PoseStamped currentPose;
geometry_msgs::PointStamped goal;
pcl::PointCloud<pcl::PointXYZ> cloud;
bool activePathPlanning = false;
PathPlaning *pp = nullptr;
ros::NodeHandle* n;

void pathcallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  geometry_msgs::PoseStamped poseData = *msg;
  poseData.pose.position.x = 0;
  poseData.pose.position.y = 0;
  poseData.pose.position.z = 0;
  currentPose = poseData;
  //std::cout << poseData.pose.position << std::endl;
}

void functionTestCall(double radiusStart, double radiusStop, double stepSize, int it, std::ostringstream& buf){
  for(double i=radiusStart; i<=radiusStop; i+=stepSize){
    buf << "[i=1000, r=" << i << ", d=4, D=12];";
  }
  buf << "\n";
  for(int j=0; j<it; j++){
    for(double i=radiusStart; i<=radiusStop; i+=stepSize){
      pp = new PathPlaning(*n, i, i);
      buf << pp->getPath(currentPose.pose, goal.point, cloud, 12, 1000) << ";";
      delete pp;
    }
    buf << "\n";
  }
}

void pointClickedcallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  geometry_msgs::PointStamped goalPoint = *msg;
  goalPoint.point.x = -4;
  goalPoint.point.y = 0;
  goalPoint.point.z = 0;
  goal = goalPoint;
  //              ZED Position     GOAL Position   D    i
  std::ostringstream buf;
  functionTestCall(0, 3.2, 0.4, 2, buf);
  std::cout << buf.str() << '\n';
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
  n = new ros::NodeHandle();
  spdlog::set_level(spdlog::level::off);
  //                      min max
  //spdlog::info("subscribing to pose");
  //ros::Subscriber pathSub = n.subscribe("/zedi/zed_node/pose", 1, &pathcallback);
  //spdlog::info("subscribing to point");
  ros::Subscriber goalSub = n->subscribe("/clicked_point", 1, &pointClickedcallback);
  //spdlog::info("sub {}", goalSub.getNumPublishers());
  //std::cout << "subscribing to cloud" << '\n';
  //ros::Subscriber cloudSub = n.subscribe("/zedi/cloud_map", 1, &cloud2dcallback);

  ros::spin();
  return 0;
}
