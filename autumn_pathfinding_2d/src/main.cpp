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
  poseData.pose.position.x = -2;
  poseData.pose.position.y = 2;
  poseData.pose.position.z = 0;
  currentPose = poseData;
}

void functionTestCall(int valueStart, int valueStop, int stepSize, int it, std::ostringstream& buf, std::ostringstream& buf2){
  for(int i=valueStart; i<=valueStop; i+=stepSize){
    buf << "[i="<< i << ", r=0, d=4, D=1.2];";
    buf2 << "[i="<< i << ", r=0, d=4, D=1.2];";
  }
  buf << "\n";
  buf2 << "\n";
  pp = new PathPlaning(*n, 2, 2);
  for(int j=0; j<it; j++){
    for(int i=valueStart; i<=valueStop; i+=stepSize){
      std::pair<double, double> res = pp->getPath(currentGrid, currentPose.pose, goal.point, 4, i);
      spdlog::info("test {} {}", res.first, res.second);
      buf << res.first << ";";
      buf2 << res.second << ";";
    }
    buf << "\n";
    buf2 << "\n";
  }
}

void pointClickedcallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  geometry_msgs::PointStamped goalPoint = *msg;
  goal = goalPoint; 
  pp->getPath(currentGrid, currentPose.pose, goal.point, 10, 10000);
  //          OccupancyGrid   ZED Position     GOAL Position   D    i
  /*std::ostringstream buf;
  std::ostringstream buf2;
  functionTestCall(0, 10000, 500, 300, buf, buf2);*///Tests
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "explorator");
  //spdlog::info("Listening");
  n = new ros::NodeHandle();
  nav_msgs::OccupancyGrid grid;
  spdlog::set_level(spdlog::level::debug);
  //                      min max
  pp = new PathPlaning(*n, 4, 8);
  ros::Subscriber gridSub = n->subscribe("/zedi/map", 1, &gridcallback);
  ros::Subscriber pathSub = n->subscribe("/zedi/zed_node/pose", 1, &pathcallback);
  ros::Subscriber goalSub = n->subscribe("/clicked_point", 1, &pointClickedcallback);

  ros::spin();
  return 0;
}
