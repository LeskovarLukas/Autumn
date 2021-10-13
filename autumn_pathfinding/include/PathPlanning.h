#ifndef PathPlaning_H
#define PathPlaning_H

#include "Point3D.h"

class PathPlaning
{
public:
  //Consturctor
  PathPlaning(ros::NodeHandle n, int rMin, int rMax);

  //Helper Methodes
  bool isInitialized(pcl::PointCloud<pcl::PointXYZ> cloud, geometry_msgs::Pose pose, geometry_msgs::Point goal);

  //PathPlaning Methods
  void getPath(geometry_msgs::Pose p, geometry_msgs::Point point, pcl::PointCloud<pcl::PointXYZ> c, int nodeSpacing, int iteratons);

private:
  geometry_msgs::Pose Pose;
  Point3D goal;
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
  Point3D getNearestNode(Point3D startNode);
  Point3D generateNewNode(Point3D nearest, Point3D random, int d);
  double getPathLength(Point3D node);
  std::vector<Point3D> getNearestNeighbors(Point3D node, double range);
  bool prevPathValid(Point3D newPosNode, Point3D goalNode, int nodeSpacing);

  //Helper Methodes
  geometry_msgs::PointStamped generatePoint(Point3D node);
  geometry_msgs::PoseStamped generatePose(Point3D node);
  nav_msgs::Path generatePath(Point3D goalNode);
  bool sphereIsFree(Point3D node, int radius);
  bool lineIsFree(float x1, float y1, float z1, float x2, float y2, float z2, int radius);
  bool cellCombinationIsFree(float cord1, float cord2, float cord3);
  int gridIndex(int x, int y);
  long pairing(int x, int y);
  double nodeDistance(Point3D node1, Point3D node2);
};

#endif
