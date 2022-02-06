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
    pp->getPath(currentGrid, currentPose.pose, goal.point, 12, 1000);
    activePathPlanning = false;
  }
}

void pathcallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  geometry_msgs::PoseStamped poseData = *msg;
  poseData.pose.position.x = 0;
  poseData.pose.position.y = 0;
  poseData.pose.position.z = 0;
  currentPose = poseData;
  //std::cout << poseData.pose.position << std::endl;
}

void functionTestCall(int radiusStart, int radiusStop, int stepSize, int it, std::ostringstream& buf){
  for(int i=radiusStart; i<=radiusStop; i+=stepSize){
    buf << "[i=1000, r=" << i << ", d=4, D=12];";
  }
  buf << "\n";
  for(int j=0; j<it; j++){
    for(int i=radiusStart; i<=radiusStop; i+=stepSize){
      pp = new PathPlaning(*n, i, i);
      buf << pp->getPath(currentGrid, currentPose.pose, goal.point, 120, 1000) << ";";
    }
    buf << "\n";
  }
}

void pointClickedcallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  geometry_msgs::PointStamped goalPoint = *msg;
  goalPoint.point.x = 4;
  goalPoint.point.y = 0;
  goal = goalPoint;
  //          OccupancyGrid   ZED Position     GOAL Position   D    i
  std::ostringstream buf;
  functionTestCall(0, 320, 40, 5, buf);
  std::cout << buf.str() << '\n';
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "explorator");
  //spdlog::info("Listening");
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
