#ifndef PathPlaning_H
#define PathPlaning_H

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
  geometry_msgs::Point goal;
  pcl::PointCloud<pcl::PointXYZ> cloud;
  std::map<long, long> Tree;
  std::pair<int, int> centerDelta;
  int radiusCollisionMin;
  int radiusCollisionMax;
  double exponent;
  const int pairingAdditiv = 10000;
  //ROS Publisher
  ros::Publisher pubNewNode;
  ros::Publisher pubRandNode;
  ros::Publisher pubPath;
  std::pair<int, int> generateXrand(double goalDistance);
  bool cellIsFree(int x, int y);
  bool pathIsFree(long node1, long node2, int radius);
  long getNearestNode(int x, int y, long goalNode);
  long generateNewNode(long nearest, long random, int d);
  double getPathLength(long node);
  std::vector<long> getNearestNeighbors(long node, double range, long goalNode);
  bool prevPathValid(long newPosNode, long goalNode, int nodeSpacing);

  //Helper Methodes
  geometry_msgs::PointStamped generatePoint(long node);
  geometry_msgs::PoseStamped generatePose(long node);
  nav_msgs::Path generatePath(long goalNode);
  int gridIndex(int x, int y);
  void setCenterDelta();
  void ajustFloatingPoint();
  long pairing(int x, int y);
  double nodeDistance(long node1, long node2);
  std::pair<int, int> depairing(long z);
  //Mathemetical
  int gschSum(int w);
  int triangularRoot(int z);
};

#endif
