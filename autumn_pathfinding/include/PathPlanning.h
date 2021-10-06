#ifndef PathPlaning_H
#define PathPlaning_H

#include "Point3D.h"

class PathPlaning
{
public:
  //Consturctor
  PathPlaning(ros::NodeHandle n, int rMin, int rMax);

  //Helper Methodes
  bool isInitialized();

  //PathPlaning Methods
  void getPath(nav_msgs::OccupancyGrid g, geometry_msgs::Pose p, geometry_msgs::Point goal, pcl::PointCloud<pcl::PointXYZ> c, int nodeSpacing, int iteratons);

private:
  nav_msgs::OccupancyGrid Grid;
  geometry_msgs::Pose Pose;
  Point3D goal;
  std::map<long, long> Tree;
  std::pair<int, int> centerDelta;
  int radiusCollisionMin;
  int radiusCollisionMax;
  //ROS Publisher
  ros::Publisher pubNewNode;
  ros::Publisher pubRandNode;
  ros::Publisher pubPath;
  Point3D generateXrand(double goalDistance);
  bool cellIsFree(float x, float y, float z);
  bool pathIsFree(Point3D node1, Point3D node2, int radius);
  Point3D getNearestNode(Point3D startNode, Point3D goalNode);
  Point3D generateNewNode(Point3D nearest, Point3D random, int d);
  double getPathLength(Point3D node);
  std::vector<Point3D> getNearestNeighbors(Point3D node, Point3D goalNode, double range);
  bool prevPathValid(Point3D newPosNode, Point3D goalNode, int nodeSpacing);

  //Helper Methodes
  geometry_msgs::PointStamped generatePoint(Point3D node);
  geometry_msgs::PoseStamped generatePose(Point3D node);
  nav_msgs::Path generatePath(Point3D goalNode);
  bool sphereIsFree(Point3D node, int radius);
  bool lineIsFree(int x1, int y1, int x2, int y2, int radius);
  int gridIndex(int x, int y);
  void setCenterDelta();
  long pairing(int x, int y);
  double nodeDistance(Point3D node1, Point3D node2);
};

#endif
