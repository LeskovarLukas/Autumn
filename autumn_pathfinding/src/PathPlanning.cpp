//ROS Includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//C++ Includes
#include <iostream>
#include <iterator>
#include <map>
#include <math.h>
#include <time.h>
//Custom Includes
#include "PathPlanning.h"

//Consturctor
PathPlaning::PathPlaning(ros::NodeHandle n, int rMin, int rMax)
{
  this->pubNewNode = n.advertise<geometry_msgs::PointStamped>("autumn_newNodes", 50);
  this->pubRandNode = n.advertise<geometry_msgs::PointStamped>("autumn_randNode", 10);
  this->pubPath = n.advertise<nav_msgs::Path>("autumn_path", 1);
  this->radiusCollisionMin = rMin;
  this->radiusCollisionMax = rMax;
  srand(time(0));
};

//PathPlaning Methods
void PathPlaning::getPath(nav_msgs::OccupancyGrid g, geometry_msgs::Pose p, geometry_msgs::Point point, pcl::PointCloud<pcl::PointXYZ> c, int nodeSpacing, int iteratons)
{
  this->cloud = c;
  this->Pose = p;
  this->Grid = g;
  Point3D goalPoint(point.x, point.y, point.z);
  this->goal = goalPoint;
  std::cout << "goal " << goal.x << " " << goal.y << " " << goal.z << '\n';
  ros::Rate pubRate(100);
  if (isInitialized())
  {
    //initialize goal node
    Point3D goalNode = this->goal;
    //NEED TO CHECK IF GOAL IS VALID!!
    Point3D startNode(Pose.position.x, Pose.position.y, Pose.position.z);
    //check if previous path is still valid && connects new position to best path node;
    if (prevPathValid(startNode, goalNode, nodeSpacing))
    {
      nav_msgs::Path path = generatePath(goalNode);
      pubPath.publish(path);
      ros::spinOnce();
      return;
    }
    Tree.clear();
    setCenterDelta();
    //check if goal is colliding
    if (!pathIsFree(goalNode, goalNode, radiusCollisionMax))
    {
      std::cout << "invalid goal!" << '\n';
      return;
    }
    Point3D::points.insert({goalNode, goalNode});
    //add x_init(zed position) to tree
    if (!pathIsFree(startNode, startNode, radiusCollisionMin))
    {
      std::cout << "colliding start position" << '\n';
    }
    Point3D::points.insert({startNode, startNode});
    //calculate direct distance start => goal
    double startGoalMinDistance = nodeDistance(goalNode, startNode);
    double minGoalPath = INT_MAX;
    for (int i = 0; i < iteratons; i++)
    {
      //generate random Node
      Point3D randNode = generateXrand(startGoalMinDistance);
      pubRandNode.publish(generatePoint(randNode));
      //find nearest node to x_rand
      Point3D nearestNode = getNearestNode(randNode, goalNode);
      //generate new node based of x_near in direction x_rand with distance d
      Point3D newNode = generateNewNode(nearestNode, randNode, nodeSpacing);
      if (Point3D::points.count(newNode) || !newNode.valid)
      { //Check if node is already in Tree
        continue;
      }
      if (startNode.equals(nearestNode) || pathIsFree(newNode, nearestNode, radiusCollisionMax))
      {
        //pubNewNode.publish(generatePoint(newNode));
        //ros::spinOnce();
        //Get neighboring nodes
        double range = nodeSpacing; //log(Tree.size()) / pow(Tree.size(), 1/nodeSpacing); //calculate search radius
        std::vector<Point3D> nearNeighbors = getNearestNeighbors(newNode, goalNode, range);
        //Find closest node to x_new
        Point3D minNeighbor = nearestNode;
        double nearestNodePathLength = getPathLength(nearestNode);
        if (nearestNodePathLength == -1)
        {
          continue;
        }
        double minDist = nearestNodePathLength + nodeDistance(nearestNode, newNode);
        for (Point3D node : nearNeighbors)
        {
          double pathLength = getPathLength(node);
          if (pathLength != -1 && pathLength + nodeDistance(node, newNode) < minDist && pathIsFree(node, newNode, radiusCollisionMax))
          {
            minNeighbor = node;
            minDist = getPathLength(node) + nodeDistance(node, newNode);
          }
        }
        Point3D::points.insert({newNode, minNeighbor});
        //Rewire neighbors to optimal pathSub
        for (Point3D node : nearNeighbors)
        {
          double pathLength = getPathLength(node);
          if (pathLength != -1 && getPathLength(newNode) + nodeDistance(node, newNode) < pathLength && pathIsFree(newNode, node, radiusCollisionMax))
          {
            Point3D::points[node] = newNode;
          }
        }
        //Check if goal is within reach
        double goalNewNodeDist = nodeDistance(newNode, goalNode);
        if (goalNewNodeDist <= nodeSpacing && getPathLength(newNode) + goalNewNodeDist < minGoalPath && pathIsFree(goalNode, newNode, radiusCollisionMax))
        {
          Point3D::points[goalNode] = newNode;
          minGoalPath = getPathLength(newNode) + goalNewNodeDist;
          //break;
        }
        //pubRate.sleep();
      }
    }
    nav_msgs::Path path = generatePath(goalNode);
    pubPath.publish(path);
    ros::spinOnce();
    std::cout << "ended" << '\n';
  }
  else
  {
    std::cout << "Topics haven't published yet!" << '\n';
  }
}

Point3D PathPlaning::generateXrand(double goalDistance)
{
  Point3D p((goalDistance * 1.5 + 60) * ((double)rand() / (RAND_MAX)),
            (goalDistance * 1.5 + 60) * ((double)rand() / (RAND_MAX)),
            (goalDistance * 1.5 + 60) * ((double)rand() / (RAND_MAX)));
  p.x = p.x * pow(-1, rand() % 2);
  p.y = p.y * pow(-1, rand() % 2);
  p.z = p.z * pow(-1, rand() % 2);
  return p;
}

bool PathPlaning::cellIsFree(int x, int y)
{
  int index = gridIndex(x, y);
  if (index >= Grid.info.width * Grid.info.height || index < 0)
  {
    return true;
  }
  return Grid.data[index] != 100;
}

bool PathPlaning::pathIsFree(Point3D node1, Point3D node2, int radius)
{
  std::pair<int, int> cords1 = depairing(node1);
  std::pair<int, int> cords2 = depairing(node2);
  for (int i = 0; i <= radius; i++)
  {
    for (int j = 0; j <= radius - i; j++)
    {
      if (!cellIsFree(cords1.first + i, cords1.second + j) || !cellIsFree(cords1.first - i, cords1.second + j) || !cellIsFree(cords1.first + i, cords1.second - j) || !cellIsFree(cords1.first - i, cords1.second - j))
      {
        return false;
      }
    }
  }
  double k = (double)(cords2.second - cords1.second) / (cords2.first - cords1.first == 0 ? 1 : cords2.first - cords1.first);
  int i = 0;
  if (abs(k) > 1)
  {
    while (i != (cords2.second - cords1.second))
    {
      int x = (int)(cords1.first + (int)(i / k));
      int y = cords1.second + i;
      for (int j = 1; j <= radius; j++)
      {
        if (!cellIsFree(x + j, y) || !cellIsFree(x - j, y) || !cellIsFree(x, y - j) || !cellIsFree(x, y + j))
        {
          return false;
        }
      }
      i += (cords2.second - cords1.second) / abs(cords2.second - cords1.second);
    }
  }
  else
  {
    while (i != (cords2.first - cords1.first))
    {
      int x = (cords1.first + i);
      int y = (int)(cords1.second + (int)(k * (i)));
      for (int j = 1; j <= radius; j++)
      {
        if (!cellIsFree(x + j, y) || !cellIsFree(x - j, y) || !cellIsFree(x, y - j) || !cellIsFree(x, y + j))
        {
          return false;
        }
      }
      i += (cords2.first - cords1.first) / abs(cords2.first - cords1.first);
    }
  }
  for (int i = 0; i <= radius; i++)
  {
    for (int j = 0; j <= radius - i; j++)
    {
      if (!cellIsFree(cords2.first + i, cords2.second + j) || !cellIsFree(cords2.first - i, cords2.second + j) || !cellIsFree(cords2.first + i, cords2.second - j) || !cellIsFree(cords2.first - i, cords2.second - j))
      {
        return false;
      }
    }
  }
  return true;
}

Point3D PathPlaning::getNearestNode(Point3D startNode, Point3D goalNode)
{
  std::pair<long, double> minDistance;
  minDistance.second = (double)INT_MAX;
  for (auto node : points)
  {
    if (node == goalNode)
    {
      continue;
    }
    Point3D tmpNode = node.first;
    double distance = nodeDistance(node.first, startNode);
    if (minDistance.second > distance)
    {
      minDistance.first = node.first;
      minDistance.second = distance;
    }
  }
  return minDistance.first;
}

Point3D PathPlaning::generateNewNode(Point3D nearest, Point3D random, int d)
{
  std::pair<int, int> nearestCords = depairing(nearest);
  std::pair<int, int> randomCords = depairing(random);
  int a = randomCords.first - nearestCords.first;
  int b = randomCords.second - nearestCords.second;
  if (a == 0 && b == 0)
  {
    return -1;
  }
  double c = sqrt(pow(a, 2) + pow(b, 2));
  int x = (int)(((double)a / c) * d);
  int y = (int)(((double)b / c) * d);
  return pairing(nearestCords.first + x, nearestCords.second + y);
}

double PathPlaning::getPathLength(Point3D node)
{
  double length = 0;
  do
  {
    length += nodeDistance(node, Tree[node]);
    if (node == Tree[node])
    {
      return -1;
    }
    node = Tree[node];
  } while (node != -1);
  return length;
}

std::vector<Point3D> PathPlaning::getNearestNeighbors(Point3D node, Point3D goalNode, double range)
{
  std::vector<long> neighbors;
  for (auto n : Tree)
  {
    if (nodeDistance(node, n.first) <= range)
    {
      if (n.first == goalNode)
      {
        continue;
      }
      neighbors.push_back(n.first);
    }
  }
  return neighbors;
}

bool PathPlaning::prevPathValid(Point3D newPosNode, Point3D goalNode, int nodeSpacing)
{
  if (Tree.count(goalNode) > 0)
  {
    Point3D node = goalNode;

    double minGoalPath = INT_MAX;
    Point3D minDistNode = nullptr;

    double pathLength = nodeDistance(goalNode, Tree[goalNode]);
    while (Tree[node] != -1)
    {
      //check for collisions and loops
      if (!pathIsFree(node, Tree[node], radiusCollisionMin) || node == Tree[node])
      {
        if (node != Tree[node])
        {
          //locate last node before collision and publish a path based on it
          long endNode = -1;
          while (Tree[node] != -1)
          {
            if (node == Tree[node])
            {
              break;
            }
            node = Tree[node];
            if (pathIsFree(node, Tree[node], radiusCollisionMin))
            {
              if (endNode == -1)
              {
                endNode = node;
              }
            }
            else if (Tree[node] != -1)
            {
              endNode = -1;
            }
          }
          if (endNode != -1)
          {
            nav_msgs::Path path = generatePath(endNode);
            pubPath.publish(path);
            ros::spinOnce();
          }
        }
        return false;
      }
      node = Tree[node];
      //smooth out path by checking if unneccecary nodes exist and rewire
      if (Tree.count(Tree[Tree[node]]) && nodeDistance(Tree[Tree[node]], node) < nodeSpacing)
      {
        Tree.erase(Tree[node]);
        Tree[node] = Tree[Tree[node]];
      }
      //check closest node to new position
      if (nodeDistance(node, newPosNode) + pathLength < minGoalPath && nodeDistance(node, newPosNode) < nodeSpacing && pathIsFree(node, newPosNode, radiusCollisionMin))
      {
        minGoalPath = nodeDistance(node, newPosNode) + pathLength;
        pathLength += nodeDistance(node, newPosNode);
        minDistNode = node;
      }
    }
    //check if path has been found
    if (minGoalPath != 0)
    {
      if (minDistNode == -1)
      {
        return false;
      }
      Tree[minDistNode] = newPosNode;
      Tree[newPosNode] = -1;
    }
    return true;
  }
  return false;
}

//Helper Methods
geometry_msgs::PointStamped PathPlaning::generatePoint(Point3D node)
{
  geometry_msgs::PointStamped msg;
  msg.header.seq = 0;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  std::pair<int, int> cords = depairing(node);
  msg.point.x = node.x;
  msg.point.y = node.y;
  msg.point.z = node.z;
  return msg;
}

geometry_msgs::PoseStamped PathPlaning::generatePose(Point3D node)
{
  geometry_msgs::PoseStamped msg;
  msg.header.seq = 0;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  std::pair<int, int> cords = depairing(node);
  msg.pose.position.x = node.x;
  msg.pose.position.y = node.y;
  msg.pose.position.z = node.z;
  msg.pose.orientation.x = 0;
  msg.pose.orientation.y = 0;
  msg.pose.orientation.z = 0;
  msg.pose.orientation.w = 0;
  return msg;
}

nav_msgs::Path PathPlaning::generatePath(Point3D goalNode)
{
  nav_msgs::Path path;
  path.header.seq = 0;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";
  Point3D node = goalNode;
  do
  {
    path.poses.push_back(generatePose(node));
    if (node == points[node])
    {
      break;
    }
    node = Point3D[node];
  } while (node != -1);
  return path;
}

int PathPlaning::gridIndex(int x, int y)
{
  int inverseDeltaX = (Grid.info.width + centerDelta.first);
  int inverseDeltaY = (Grid.info.height + centerDelta.second);
  if (x <= centerDelta.first || y <= centerDelta.second || x >= inverseDeltaX || y >= inverseDeltaY)
  {
    return -1;
  }
  int rowPos = (centerDelta.first + x * -1);
  int row = (centerDelta.second - y) + 2;
  int tmp = Grid.info.width;
  return ((row * tmp) + rowPos) * -1;
}

void PathPlaning::setCenterDelta()
{
  int originY = (int)(Grid.info.origin.position.y / Grid.info.resolution);
  int originX = (int)(Grid.info.origin.position.x / Grid.info.resolution);
  int deltaY = 0 - originY * -1;
  int deltaX = 0 - originX * -1;
  this->centerDelta = std::pair<int, int>(deltaX, deltaY);
}

bool PathPlaning::isInitialized()
{
  nav_msgs::OccupancyGrid defaultGrid;
  geometry_msgs::Pose defaultPose;
  geometry_msgs::Point defaultGoal;
  return !(this->Grid == defaultGrid) && !(this->Pose == defaultPose);
}

long PathPlaning::pairing(int x, int y)
{
  x = x + this->pairingAdditiv;
  y = y + this->pairingAdditiv;
  long z = (0.5 * (x + y) * (x + y + 1) + y);
  return z;
}

double PathPlaning::nodeDistance(Point3D node1, Point3D node2)
{
  std::pair<int, int> cords1 = depairing(node1);
  std::pair<int, int> cords2 = depairing(node2);
  int a = abs(cords1.first - cords2.first);
  int b = abs(cords1.second - cords2.second);
  double c = sqrt(pow(a, 2) + pow(b, 2));
  return c;
}

std::pair<int, int> PathPlaning::depairing(long z)
{
  int y = (int)(z - gschSum(triangularRoot(z)));
  int x = (int)(triangularRoot(z) - y);
  x = x - this->pairingAdditiv;
  y = y - this->pairingAdditiv;

  return std::pair<int, int>(x, y);
}

//Mathemetical
int PathPlaning::gschSum(int w)
{
  int f = (w * (w + 1)) / 2;
  return f;
}

int PathPlaning::triangularRoot(int z)
{
  int q = (int)((sqrt(8 * z + 1) - 1) / 2);
  return q;
}
