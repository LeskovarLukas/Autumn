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
#include "Point3D.h"

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
  std::cout << "start" << std::endl;
  this->Pose = p;
  this->Grid = g;
  Point3D goalPoint(point.x, point.y, point.z);
  this->goal = goalPoint;
  std::cout << "goal " << goal.point.x << " " << goal.point.y << " " << goal.point.z << '\n';
  ros::Rate pubRate(100);
  if (isInitialized())
  {
    Point3D::addPoints(c);
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
  p.point.x = p.point.x * pow(-1, rand() % 2);
  p.point.y = p.point.y * pow(-1, rand() % 2);
  p.point.z = p.point.z * pow(-1, rand() % 2);
  return p;
}

bool PathPlaning::cellIsFree(float x, float y, float z)
{
  PointVls pv(x, y, z);
  if(Point3D::pointCldDic.count(hashPoint(pv))){
    return false;
  }
  return true;
}

bool PathPlaning::pathIsFree(Point3D node1, Point3D node2, int radius)
{
  if(!sphereIsFree(node1, radius) || !sphereIsFree(node2, radius)){
    return false;
  }
  float k1 = ;
  int i = 0;
  if (abs(k) > 1 && !lineIsFree(node1.x, node1.y, node2.x, node2.y, radius))
  {
    return false;
  }
  else if(!lineIsFree(node1.y, node1.x, node2.y, node2.x, radius))
  {
    return false;
  }
  return true;
}

Point3D PathPlaning::getNearestNode(Point3D startNode, Point3D goalNode)
{
  std::pair<Point3D, double> minDistance;
  minDistance.second = (double)INT_MAX;
  for (auto point : Point3D::points)
  {
    Point3D node = point.first;
    if (node.equals(goalNode))
    {
      continue;
    }
    Point3D tmpNode = node;
    double distance = nodeDistance(node, startNode);
    if (minDistance.second > distance)
    {
      minDistance.first = node;
      minDistance.second = distance;
    }
  }
  return minDistance.first;
}

Point3D PathPlaning::generateNewNode(Point3D nearest, Point3D random, int d)
{
  Point3D p;
  int a = random.point.x - nearest.point.x;
  int b = random.point.y - nearest.point.y;
  int z = random.point.z - nearest.point.z;
  if (a == 0 && b == 0 && z == 0)
  {
    p.valid = false;
    return p;
  }
  double c1 = sqrt(pow(a, 2) + pow(b, 2));
  double c2 = sqrt(pow(c1, 2) + pow(z, 2));
  p.point.x = (int)(((double)a / c2) * d);
  p.point.y = (int)(((double)b / c2) * d);
  p.point.z = (int)(((double)z / c2) * d);
  return p;
}

double PathPlaning::getPathLength(Point3D node)
{
  double length = 0;
  do
  {
    length += nodeDistance(node, Point3D::points[node]);
    if (node.equals(Point3D::points[node]))
    {
      return -1;
    }
    node = Point3D::points[node];
  } while (!node.start);
  return length;
}

std::vector<Point3D> PathPlaning::getNearestNeighbors(Point3D node, Point3D goalNode, double range)
{
  std::vector<Point3D> neighbors;
  for (auto n : Point3D::points)
  {
    Point3D nd = n.first;
    if (nodeDistance(node, nd) <= range)
    {
      if (nd.equals(goalNode))
      {
        continue;
      }
      neighbors.push_back(nd);
    }
  }
  return neighbors;
}

bool PathPlaning::prevPathValid(Point3D newPosNode, Point3D goalNode, int nodeSpacing)
{
  if (Point3D::points.count(goalNode) > 0)
  {
    Point3D node = goalNode;

    double minGoalPath = INT_MAX;
    Point3D *minDistNode = nullptr;

    double pathLength = nodeDistance(goalNode, Point3D::points[goalNode]);
    while (!Point3D::points[node].start)
    {
      //check for collisions and loops
      if (!pathIsFree(node, Point3D::points[node], radiusCollisionMin) || node.equals(Point3D::points[node]))
      {
        if (!node.equals(Point3D::points[node]))
        {
          //locate last node before collision and publish a path based on it
          Point3D endNode;
          endNode.valid = false;
          while (!Point3D::points[node].start)
          {
            if (node.equals(Point3D::points[node]))
            {
              break;
            }
            node = Point3D::points[node];
            if (pathIsFree(node, Point3D::points[node], radiusCollisionMin))
            {
              if (!endNode.valid)
              {
                endNode = node;
              }
            }
            else if (!Point3D::points[node].start)
            {
              endNode.valid = false;
            }
          }
          if (endNode.valid)
          {
            nav_msgs::Path path = generatePath(endNode);
            pubPath.publish(path);
            ros::spinOnce();
          }
        }
        return false;
      }
      node = Point3D::points[node];
      //smooth out path by checking if unneccecary nodes exist and rewire
      if (Point3D::points.count(Point3D::points[Point3D::points[node]]) && nodeDistance(Point3D::points[Point3D::points[node]], node) < nodeSpacing)
      {
        Point3D::points.erase(Point3D::points[node]);
        Point3D::points[node] = Point3D::points[Point3D::points[node]];
      }
      //check closest node to new position
      if (nodeDistance(node, newPosNode) + pathLength < minGoalPath && nodeDistance(node, newPosNode) < nodeSpacing && pathIsFree(node, newPosNode, radiusCollisionMin))
      {
        minGoalPath = nodeDistance(node, newPosNode) + pathLength;
        pathLength += nodeDistance(node, newPosNode);
        *minDistNode = node;
      }
    }
    //check if path has been found
    if (minGoalPath != 0)
    {
      if (minDistNode->start)
      {
        return false;
      }
      Point3D::points[*minDistNode] = newPosNode;
      Point3D::points[newPosNode].start = true;
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
  msg.point.x = node.point.x;
  msg.point.y = node.point.y;
  msg.point.z = node.point.z;
  return msg;
}

geometry_msgs::PoseStamped PathPlaning::generatePose(Point3D node)
{
  geometry_msgs::PoseStamped msg;
  msg.header.seq = 0;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.pose.position.x = node.point.x;
  msg.pose.position.y = node.point.y;
  msg.pose.position.z = node.point.z;
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
    if (node.equals(Point3D::points[node]))
    {
      break;
    }
    node = Point3D::points[node];
  } while (!node.start);
  return path;
}

bool PathPlaning::sphereIsFree(Point3D node, int radius){
  for (int i = 0; i <= radius; i++)
  {
    for (int j = 0; j <= radius - i; j++)
    {
      for(int k = 0; k <= radius - i; k++)
      {
        if (!cellIsFree(node.x + i, node.y + j, node.z + k)
        || !cellIsFree(node.x - i, node.y + j, node.z +k)
        || !cellIsFree(node.x - i, node.y - j, node.z +k)
        || !cellIsFree(node.x - i, node.y - j, node.z -k)
        || !cellIsFree(node.x + i, node.y - j, node.z -k)
        || !cellIsFree(node.x + i, node.y + j, node.z -k)
        || !cellIsFree(node.x - i, node.y + j, node.z -k)
        || !cellIsFree(node.x + i, node.y - j, node.z +k))
        {
          return false;
        }
      }
    }
  }
  return true;
}

bool lineIsFree(int x1, int y1, int x2, int y2, int radius)
{
  while (i != (y2 - y1))
  {
    int x = (int)(x1 + (int)(i / k));
    int y = y1 + i;
    for (int j = 1; j <= radius; j++)
    {
      if (!cellIsFree(x + j, y) || !cellIsFree(x - j, y) || !cellIsFree(x, y - j) || !cellIsFree(x, y + j))
      {
        return false;
      }
    }
    i += (y2 - y1) / abs(y2 - y1);
  }
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
  /*nav_msgs::OccupancyGrid defaultGrid;
  geometry_msgs::Pose defaultPose;
  geometry_msgs::Point defaultGoal;
  return !(this->Grid == defaultGrid) && !(this->Pose == defaultPose);*/
  return true;
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
  int a = abs(node1.point.x - node2.point.x);
  int b = abs(node1.point.y - node2.point.y);
  int z = abs(node1.point.z - node2.point.z);
  double c1 = sqrt(pow(a, 2) + pow(b, 2));
  double c2 = sqrt(pow(z, 2) + pow(c1, 2));
  return c2;
}
