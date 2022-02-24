//ROS Includes
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/PointCloud2.h"
//C++ Includes
#include <iostream>
#include <iterator>
#include <map>
#include <math.h>
#include <time.h>
#include <chrono>
//Custom Includes
#include "PathPlanning.h"
#include "spdlog/spdlog.h"

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
std::pair<double, double> PathPlaning::getPath(nav_msgs::OccupancyGrid g, geometry_msgs::Pose p, geometry_msgs::Point point, int nodeSpacing, int iteratons)
{
  this->Pose = p;
  this->Grid = g;
  this->goal = point;
  std::pair<double, double> res;
  ros::Rate pubRate(100);
  auto time_start = std::chrono::high_resolution_clock::now();
  if (isInitialized() || true)
  {
    //get rid of the floating point
    this->exponent = pow(10, abs((int)log10(Grid.info.resolution)) + 1);
    ajustFloatingPoint();
    //initialize goal node
    long goalNode = pairing(goal.x / Grid.info.resolution, goal.y / Grid.info.resolution);
    if (goalNode == 0)
    {
      return res;
    }
    long startNode = pairing(Pose.position.x / Grid.info.resolution , Pose.position.y / Grid.info.resolution);
    spdlog::debug("x: {}, y: {}", (Pose.position.x), (Pose.position.y));
    //check if previous path is still valid && connects new position to best path node;
    if (false && prevPathValid(startNode, goalNode, nodeSpacing))
    {
      nav_msgs::Path path = generatePath(goalNode);
      pubPath.publish(path);
      ros::spinOnce();
      return res;
    }
    Tree.clear();
    setCenterDelta();
    //check if goal is colliding
    if (!pathIsFree(goalNode, goalNode, radiusCollisionMax))
    {
      spdlog::debug("invalid goal");
      return res;
    }
    Tree.insert({goalNode, 0});
    //add x_init(zed position) to tree
    if (!pathIsFree(startNode, startNode, radiusCollisionMin))
    {
      spdlog::debug("colliding start position");
    }
    Tree.insert({startNode, -1});
    //calculate direct distance start => goal
    double startGoalMinDistance = nodeDistance(goalNode, startNode);
    double minGoalPath = INT_MAX;
    for (int i = 0; i < iteratons; i++)
    {
      //generate random Node
      std::pair<int, int> randCords = generateXrand(startGoalMinDistance);
      //pubRandNode.publish(generatePoint(pairing(randCords.first, randCords.second)));
      //find nearest node to x_rand
      long nearestNode = getNearestNode(randCords.first, randCords.second, goalNode);
      std::pair<int, int> cords = depairing(nearestNode);
      //generate new node based of x_near in direction x_rand with distance d
      long newNode = generateNewNode(nearestNode, pairing(randCords.first, randCords.second), nodeSpacing);
      cords = depairing(newNode);
      if (Tree.count(newNode) || newNode == -1)
      { //Check if node is already in Tree
        continue;
      }
      double spacingCounter = 1;
      while (startNode != nearestNode && !pathIsFree(newNode, nearestNode, radiusCollisionMax) && spacingCounter < nodeSpacing)
      {
        long newNode = generateNewNode(nearestNode, pairing(randCords.first, randCords.second), nodeSpacing - spacingCounter);
        spacingCounter += spacingCounter * 0.2;
      }
      if(startNode == nearestNode || pathIsFree(newNode, nearestNode, radiusCollisionMax)){
        pubNewNode.publish(generatePoint(newNode));
        ros::spinOnce();
        //Get neighboring nodes
        double range = nodeSpacing; //log(Tree.size()) / pow(Tree.size(), 1/nodeSpacing); //calculate search radius
        std::vector<long> nearNeighbors = getNearestNeighbors(newNode, range, goalNode);
        //Find closest node to x_new
        long minNeighbor = nearestNode;
        double nearestNodePathLength = getPathLength(nearestNode);
        if (nearestNodePathLength == -1)
        {
          continue;
        }
        double minDist = nearestNodePathLength + nodeDistance(nearestNode, newNode);
        for (long node : nearNeighbors)
        {
          double pathLength = getPathLength(node);
          if (pathLength != -1 && pathLength + nodeDistance(node, newNode) < minDist && pathIsFree(node, newNode, radiusCollisionMax))
          {
            minNeighbor = node;
            minDist = getPathLength(node) + nodeDistance(node, newNode);
          }
        }
        Tree.insert({newNode, minNeighbor});
        //Rewire neighbors to optimal pathSub
        for (long node : nearNeighbors)
        {
          double pathLength = getPathLength(node);
          if (pathLength != -1 && getPathLength(newNode) + nodeDistance(node, newNode) < pathLength && pathIsFree(newNode, node, radiusCollisionMax))
          {
            Tree[node] = newNode;
          }
        }
        //Check if goal is within reach
        double goalNewNodeDist = nodeDistance(newNode, goalNode);
        if (goalNewNodeDist <= nodeSpacing && getPathLength(newNode) + goalNewNodeDist < minGoalPath && pathIsFree(goalNode, newNode, radiusCollisionMax))
        {
          Tree[goalNode] = newNode;
          minGoalPath = getPathLength(newNode) + goalNewNodeDist;
          //break;
        }
        //pubRate.sleep();
      }
    }
    if(Tree.count(goalNode) && Tree[goalNode] != 0){
      spdlog::debug("goal parent ");
      nav_msgs::Path path = generatePath(goalNode);
      pubPath.publish(path);
      ros::spinOnce();
      spdlog::debug("Ended");
      spdlog::info("direct dist {}", nodeDistance(startNode, goalNode));
      auto time_end = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double, std::milli> ms_double = time_end - time_start;
      res = {ms_double.count(), getPathLength(goalNode)};
      return res;
    }else{
      spdlog::info("No path Found");
      res = {-1, -1};
      return res;
    }
  }
  else
  {
    spdlog::warn("Topics haven't published yet!");
  }
}

std::pair<int, int> PathPlaning::generateXrand(double goalDistance)
{
  int x = (goalDistance * 1.5 + 60) * ((double)rand() / (RAND_MAX));
  int y = (goalDistance * 1.5 + 60) * ((double)rand() / (RAND_MAX));
  x = x * pow(-1, rand() % 2);
  y = y * pow(-1, rand() % 2);
  return std::pair<int, int>(x, y);
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

bool PathPlaning::pathIsFree(long node1, long node2, int radius)
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

long PathPlaning::getNearestNode(int x, int y, long goalNode)
{
  long startNode = pairing(x, y);
  std::pair<long, double> minDistance;
  minDistance.second = (double)INT_MAX;
  for (auto node : Tree)
  {
    if (node.first == goalNode)
    {
      continue;
    }
    std::pair<int, int> tmpNode = depairing(node.first);
    double distance = nodeDistance(node.first, startNode);
    if (minDistance.second > distance)
    {
      minDistance.first = node.first;
      minDistance.second = distance;
    }
  }
  return minDistance.first;
}

long PathPlaning::generateNewNode(long nearest, long random, int d)
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

double PathPlaning::getPathLength(long node)
{
  double length = 0;
  while (Tree[node] != -1)
  {
    if (node == Tree[node])
    {
      return -1;
    }
    length += nodeDistance(node, Tree[node]);
    node = Tree[node];
  };
  return length;
}

std::vector<long> PathPlaning::getNearestNeighbors(long node, double range, long goalNode)
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

bool PathPlaning::prevPathValid(long newPosNode, long goalNode, int nodeSpacing)
{
  if (Tree.count(goalNode) > 0)
  {
    long node = goalNode;

    double minGoalPath = INT_MAX;
    long minDistNode = -1;

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
geometry_msgs::PointStamped PathPlaning::generatePoint(long node)
{
  geometry_msgs::PointStamped msg;
  msg.header.seq = 0;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  std::pair<int, int> cords = depairing(node);
  msg.point.x = (cords.first * Grid.info.resolution) / exponent;
  msg.point.y = (cords.second * Grid.info.resolution) / exponent;
  msg.point.z = 0;
  return msg;
}

geometry_msgs::PoseStamped PathPlaning::generatePose(long node)
{
  geometry_msgs::PoseStamped msg;
  msg.header.seq = 0;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  std::pair<int, int> cords = depairing(node);
  msg.pose.position.x = (cords.first * Grid.info.resolution) / exponent;
  msg.pose.position.y = (cords.second * Grid.info.resolution) / exponent;
  msg.pose.position.z = 0;
  msg.pose.orientation.x = 0;
  msg.pose.orientation.y = 0;
  msg.pose.orientation.z = 0;
  msg.pose.orientation.w = 0;
  return msg;
}

nav_msgs::Path PathPlaning::generatePath(long goalNode)
{
  nav_msgs::Path path;
  path.header.seq = 0;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";
  long node = goalNode;
  do
  {
    std::pair<int, int> cords = depairing(node);
    path.poses.push_back(generatePose(node));
    if (node == Tree[node])
    {
      break;
    }
    node = Tree[node];
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
  spdlog::info("Delta| originY {}, originX {}, deltaY {}, deltaX {}", originY, originX, deltaX, deltaY);
  this->centerDelta = std::pair<int, int>(deltaX, deltaY);
}

void PathPlaning::ajustFloatingPoint()
{ //Converting floating point numbers to natrual numbers because of the pairing function
  Pose.position.x = (int)(Pose.position.x * exponent);
  Pose.position.y = (int)(Pose.position.y * exponent);
  Pose.position.z = 0;

  goal.x = (int)(goal.x * exponent);
  goal.y = (int)(goal.y * exponent);
  goal.z = 0;

  Grid.info.origin.position.x = (int)(Grid.info.origin.position.x * exponent);
  Grid.info.origin.position.y = (int)(Grid.info.origin.position.y * exponent);
  Grid.info.origin.position.z = (int)(Grid.info.origin.position.z * exponent);

  Grid.info.resolution = Grid.info.resolution * exponent;
}

bool PathPlaning::isInitialized()
{
  nav_msgs::OccupancyGrid defaultGrid;
  geometry_msgs::Pose defaultPose;
  geometry_msgs::Point defaultGoal;
  return !(this->Grid == defaultGrid) && !(this->Pose == defaultPose) &&
         !(this->goal == defaultGoal);
}

long PathPlaning::pairing(int x, int y)
{
  x = x + this->pairingAdditiv;
  y = y + this->pairingAdditiv;
  long z = (0.5 * (x + y) * (x + y + 1) + y);
  return z;
}

double PathPlaning::nodeDistance(long node1, long node2)
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
