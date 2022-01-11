#include "ros/ros.h"
#include <sstream>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include <iostream>
#include "PathPlanning.h"

geometry_msgs::PoseStamped currentPose;
nav_msgs::OccupancyGrid currentGrid;
geometry_msgs::PointStamped goal;
bool activePathPlanning = false;
PathPlaning *pp = nullptr;

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
  currentPose = poseData;
  //std::cout << poseData.pose.position << std::endl;
}

void pointClickedcallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
  geometry_msgs::PointStamped goalPoint = *msg;
  goal = goalPoint;
  //          OccupancyGrid   ZED Position     GOAL Position   D    i
  pp->getPath(currentGrid, currentPose.pose, goalPoint.point, 12, 8000);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "explorator");
  ros::NodeHandle n;
  //                      min max
  pp = new PathPlaning(n, 4, 12);
  ros::Subscriber gridSub = n.subscribe("/zedi/map", 1, &gridcallback);
  ros::Subscriber pathSub = n.subscribe("/zedi/zed_node/pose", 1, &pathcallback);
  ros::Subscriber goalSub = n.subscribe("/clicked_point", 1, &pointClickedcallback);

  ros::spin();
  return 0;
}
