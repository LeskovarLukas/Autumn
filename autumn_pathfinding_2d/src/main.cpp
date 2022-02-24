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
  //std::cout << poseData.pose.position << std::endl;
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
  goalPoint.point.x = -1.8;
  goalPoint.point.y = 1.7;
  goal = goalPoint; 
  //          OccupancyGrid   ZED Position     GOAL Position   D    i
  std::ostringstream buf;
  std::ostringstream buf2;
  functionTestCall(500, 10000, 200, 500, buf, buf2);
  std::cout << buf.str() << '\n';
  std::cout << "----------------------------------------------------------------------" << std::endl;
  std::cout << buf2.str() << '\n';
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "explorator");
  //spdlog::info("Listening");
  n = new ros::NodeHandle();
  nav_msgs::OccupancyGrid grid;
  spdlog::set_level(spdlog::level::off);
  geometry_msgs::PoseStamped poseData;
  poseData.pose.position.x = 2.7;//2.6;
  poseData.pose.position.y = -2.1;//0.3;
  poseData.pose.position.z = 0;
  currentPose = poseData;
  //                      min max
  pp = new PathPlaning(*n, 4, 8);
  ros::Subscriber gridSub = n->subscribe("/map", 1, &gridcallback);
  //ros::Subscriber pathSub = n.subscribe("/zedi/zed_node/pose", 1, &pathcallback);
  ros::Subscriber goalSub = n->subscribe("/clicked_point", 1, &pointClickedcallback);

  ros::spin();
  return 0;
}
