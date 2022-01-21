#include "ros/ros.h"
#include <sstream>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include <iostream>
#include "PathPlanning.h"
#include "spdlog/spdlog.h"

geometry_msgs::PoseStamped currentPose;
nav_msgs::OccupancyGrid currentGrid;
geometry_msgs::PointStamped goal;
bool activePathPlanning = false;
PathPlaning *pp = nullptr;
ros::NodeHandle* n;

void gridcallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  nav_msgs::OccupancyGrid grid = *msg;
  currentGrid = grid;
  if (!activePathPlanning)
  {
    activePathPlanning = true;
    pp->getPath(currentGrid, currentPose.pose, goal.point, 12, 8000);
    activePathPlanning = false;
  }
}

void pathcallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  geometry_msgs::PoseStamped poseData = *msg;
  poseData.pose.position.x = -8;
  poseData.pose.position.y = 0;
  poseData.pose.position.z = 0;
  currentPose = poseData;
  //std::cout << poseData.pose.position << std::endl;
}

void pointClickedcallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  geometry_msgs::PointStamped goalPoint = *msg;
  goalPoint.point.x = 8;
  goalPoint.point.y = 0;
  goal = goalPoint;
  //          OccupancyGrid   ZED Position     GOAL Position   D    i
  std::ostringstream buf;
  buf << "[i=5000, r=10, d=8, D=12];[i=8000, r=20, d=8, D=12];[i=8000, r=40, d=8, D=12];[i=8000, r=80, d=8, D=12];[i=8000, r=160, d=8, D=12];\n";
  for(int i=0; i < 1; i++){
    pp = new PathPlaning(*n, 10, 10);
    buf << pp->getPath(currentGrid, currentPose.pose, goalPoint.point, 12, 5000) << ";";
    pp = new PathPlaning(*n, 20, 20);
    buf << pp->getPath(currentGrid, currentPose.pose, goalPoint.point, 12, 5000) << ";";
    pp = new PathPlaning(*n, 40, 40);
    buf << pp->getPath(currentGrid, currentPose.pose, goalPoint.point, 12, 5000) << ";";
    pp = new PathPlaning(*n, 80, 80);
    buf << pp->getPath(currentGrid, currentPose.pose, goalPoint.point, 12, 5000) << ";";
    pp = new PathPlaning(*n, 160, 160);
    buf << pp->getPath(currentGrid, currentPose.pose, goalPoint.point, 12, 5000) << ";\n";
  }
  std::cout << buf.str() << '\n';
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "explorator");
  spdlog::info("Listening");
  n = new ros::NodeHandle();
  nav_msgs::OccupancyGrid grid;
  spdlog::set_level(spdlog::level::off);
  grid.info.resolution = 0.05;
  grid.info.width = 100;
  grid.info.height = 100;
  geometry_msgs::PoseStamped poseData;
  poseData.pose.position.x = 0;
  poseData.pose.position.y = 0;
  poseData.pose.position.z = 0;
  currentPose = poseData;
  //                      min max
  pp = new PathPlaning(*n, 4, 12);
  //ros::Subscriber gridSub = n.subscribe("/zedi/map", 1, &gridcallback);
  //ros::Subscriber pathSub = n.subscribe("/zedi/zed_node/pose", 1, &pathcallback);
  ros::Subscriber goalSub = n->subscribe("/clicked_point", 1, &pointClickedcallback);

  ros::spin();
  return 0;
}
