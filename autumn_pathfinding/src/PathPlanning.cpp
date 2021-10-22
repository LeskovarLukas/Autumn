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
PathPlaning::PathPlaning(ros::NodeHandle n, float rMin, float rMax)
{
  this->pubNewNode = n.advertise<geometry_msgs::PointStamped>("autumn_newNodes", 50);
  this->pubRandNode = n.advertise<geometry_msgs::PointStamped>("autumn_randNode", 10);
  this->pubPath = n.advertise<nav_msgs::Path>("autumn_path", 1);
  this->radiusCollisionMin = rMin;
  this->radiusCollisionMax = rMax;
  srand(time(0));
};

//PathPlaning Methods
void PathPlaning::getPath(geometry_msgs::Pose p, geometry_msgs::Point point, pcl::PointCloud<pcl::PointXYZ> c, float nodeSpacing, int iteratons)
{
  std::cout << "start" << std::endl;
  if (isInitialized(c, p, point))
  {
    this->Pose = p;
    Point3D goalNode{static_cast<float>(point.x), static_cast<float>(point.y), static_cast<float>(point.z)};
    goalNode.goal = true;
    this->goal = goalNode;
    std::cout << "goal " << goal.point.getX() << " " << goal.point.getY() << " " << goal.point.getZ() << '\n';
    ros::Rate pubRate(100);

    Point3D::addPoints(c);
    //initialize goal node
    //NEED TO CHECK IF GOAL IS VALID!!
    Point3D startNode(Pose.position.x, Pose.position.y, Pose.position.z);
    std::cout << "start " << startNode.point.getX() << " " << startNode.point.getY() << " " << startNode.point.getZ() << '\n';
    startNode.start = true;
    //check if previous path is still valid && connects new position to best path node;
    if (prevPathValid(startNode, goalNode, nodeSpacing))
    {
      nav_msgs::Path path = generatePath(goalNode);
      pubPath.publish(path);
      ros::spinOnce();
      return;
    }
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
    float startGoalMinDistance = nodeDistance(goalNode, startNode);

    double minGoalPath = INT_MAX;
    for (int i = 0; i < iteratons; i++)
    {
      if(i % 1000 == 0){
        std::cout << "iteration: " << i << '\n';
      }
      //generate random Node
      Point3D randNode = generateXrand(startGoalMinDistance);
      //pubRandNode.publish(generatePoint(randNode));
      //find nearest node to x_rand
      Point3D nearestNode = getNearestNode(randNode);
      if(!nearestNode.valid){
        //std::cout << "nearest Node invalid" << '\n';
        continue;
      }
      //generate new node based of x_near in direction x_rand with distance d
      Point3D newNode = generateNewNode(nearestNode, randNode, nodeSpacing);
      if (Point3D::points.count(newNode) || !newNode.valid)
      { //Check if node is already in Tree
        //std::cout << "continue duplicate " << newNode.point.getX() << " " << newNode.point.getY() << " " << newNode.point.getZ() << '\n';
        continue;
      }
      if (nearestNode.start || pathIsFree(newNode, nearestNode, radiusCollisionMax))
      {
        //pubNewNode.publish(generatePoint(newNode));
        //ros::spinOnce();
        //pubRate.sleep();
        //Get neighboring nodes
        double range = nodeSpacing; //log(Tree.size()) / pow(Tree.size(), 1/nodeSpacing); //calculate search radius
        std::vector<Point3D> nearNeighbors = getNearestNeighbors(newNode, range);
        //Find closest node to x_new
        Point3D minNeighbor = nearestNode;
        double nearestNodePathLength = getPathLength(nearestNode);
        if (nearestNodePathLength == -1)
        {
          //std::cout << "continue path invalid" << '\n';
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
          nav_msgs::Path path = generatePath(goalNode);
          pubPath.publish(path);
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

Point3D PathPlaning::generateXrand(float goalDistance)
{
  Point3D p((goalDistance * 1.5 + 0.60) * ((double)rand() / (RAND_MAX)),
            (goalDistance * 1.5 + 0.60) * ((double)rand() / (RAND_MAX)),
            (goalDistance * 1.5 + 0.60) * ((double)rand() / (RAND_MAX)));
  p.point.setX(p.point.getX() * pow(-1, rand() % 2));
  p.point.setY(p.point.getY() * pow(-1, rand() % 2));
  p.point.setZ(p.point.getZ() * pow(-1, rand() % 2));
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
                  //float x1   , float y1     , float z1     , float x2     , float y2     , float z2     , int radius
  if (!lineIsFree(node1.point.getX(), node1.point.getY(), node1.point.getZ(), node2.point.getX(), node2.point.getY(), node2.point.getZ(), radius))
  {
    return false;
  }
  return true;
}

Point3D PathPlaning::getNearestNode(Point3D startNode)
{
  std::pair<Point3D, double> minDistance;
  minDistance.second = (double)INT_MAX;
  bool valid = false;
  for (auto point : Point3D::points)
  {
    Point3D node = point.first;
    if (node.goal)
    {
      continue;
    }
    double distance = nodeDistance(node, startNode);
    if (minDistance.second > distance)
    {
      valid = true;
      minDistance.first = node;
      minDistance.second = distance;
    }
  }
  minDistance.first.valid = valid;
  return minDistance.first;
}

Point3D PathPlaning::generateNewNode(Point3D nearest, Point3D random, float d)
{
  Point3D p;
  float a = random.point.getX() - nearest.point.getX();
  float b = random.point.getY() - nearest.point.getY();
  float z = random.point.getZ() - nearest.point.getZ();
  if (a == 0 && b == 0 && z == 0)
  {
    p.valid = false;
    return p;
  }
  double c1 = sqrt(pow(a, 2) + pow(b, 2));
  double c2 = sqrt(pow(c1, 2) + pow(z, 2));
  p.point.setX(nearest.point.getX() + ((a / c2) * d));
  p.point.setY(nearest.point.getY() + ((b / c2) * d));
  p.point.setZ(nearest.point.getZ() + ((z / c2) * d));

  p.valid = true;
  return p;
}

double PathPlaning::getPathLength(Point3D node)
{
  double length = 0;
  do
  {
    if(node.start){
      break;
    }
    length += nodeDistance(node, Point3D::points[node]);
    if (node.equals(Point3D::points[node]))
    {
      return -1;
    }
    node = Point3D::points[node];
  }while (!node.start);
  return length;
}

std::vector<Point3D> PathPlaning::getNearestNeighbors(Point3D node, double range)
{
  std::vector<Point3D> neighbors;
  for (auto n : Point3D::points)
  {
    Point3D nd = n.first;
    if (nodeDistance(node, nd) <= range)
    {
      if (nd.goal)
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
  msg.point.x = node.point.getX();
  msg.point.y = node.point.getY();
  msg.point.z = node.point.getZ();
  return msg;
}

geometry_msgs::PoseStamped PathPlaning::generatePose(Point3D node)
{
  geometry_msgs::PoseStamped msg;
  msg.header.seq = 0;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.pose.position.x = node.point.getX();
  msg.pose.position.y = node.point.getY();
  msg.pose.position.z = node.point.getZ();
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
  std::cout << "\n" << "goal Path:" << '\n';
  Point3D node = goalNode;
  do
  {
    path.poses.push_back(generatePose(node));
    if (node.equals(Point3D::points[node]))
    {
      break;
    }
    node = Point3D::points[node];
    std::cout << "node: x " << node.point.getX() << " y " << node.point.getY() << " z " << node.point.getZ()  << '\n';
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
        if (!cellIsFree(node.point.getX() + i, node.point.getY() + j, node.point.getZ() + k)
        || !cellIsFree(node.point.getX() - i, node.point.getY() + j, node.point.getZ() +k)
        || !cellIsFree(node.point.getX() - i, node.point.getY() - j, node.point.getZ() +k)
        || !cellIsFree(node.point.getX() - i, node.point.getY() - j, node.point.getZ() -k)
        || !cellIsFree(node.point.getX() + i, node.point.getY() - j, node.point.getZ() -k)
        || !cellIsFree(node.point.getX() + i, node.point.getY() + j, node.point.getZ() -k)
        || !cellIsFree(node.point.getX() - i, node.point.getY() + j, node.point.getZ() -k)
        || !cellIsFree(node.point.getX() + i, node.point.getY() - j, node.point.getZ() +k))
        {
          return false;
        }
      }
    }
  }
  return true;
}

bool PathPlaning::lineIsFree(float x0, float y0, float z0, float x1, float y1, float z1, int radius)
{
   float dx{abs(x1-x0)};
   int sx{x0<x1 ? 1 : -1};
   float dy{abs(y1-y0)};
   int sy{y0<y1 ? 1 : -1};
   float dz{abs(z1-z0)};
   int sz{z0<z1 ? 1 : -1};
   float dm{std::max(dx, std::max(dy, dz))};
   float i{dm}; /* maximum difference */
   x1 = y1 = z1 = dm/2; /* error offset */

   Point3D p_tmp;
   for(;;) {  /* loop */
      p_tmp.point.setX(x0);
      p_tmp.point.setY(y0);
      p_tmp.point.setZ(z0);
      if(!sphereIsFree(p_tmp, radius)){
        return false;
      }
      if (i-- <= 0) break;
      x1 -= dx; if (x1 < 0) { x1 += dm; x0 += sx; }
      y1 -= dy; if (y1 < 0) { y1 += dm; y0 += sy; }
      z1 -= dz; if (z1 < 0) { z1 += dm; z0 += sz; }
   }
}

bool PathPlaning::isInitialized(pcl::PointCloud<pcl::PointXYZ> cloud, geometry_msgs::Pose pose, geometry_msgs::Point goal)
{
  std::cout << "cloud length: " << cloud.points.size() << '\n';
  std::cout << "pose : " << pose.position << '\n';
  std::cout << "goal : " << goal << '\n';
  return true;
}

float PathPlaning::nodeDistance(Point3D node1, Point3D node2)
{
  float a = abs(node1.point.getX() - node2.point.getX());
  float b = abs(node1.point.getY() - node2.point.getY());
  float z = abs(node1.point.getZ() - node2.point.getZ());
  float c1 = sqrt(pow(a, 2) + pow(b, 2));
  float c2 = sqrt(pow(z, 2) + pow(c1, 2));
  return c2;
}
