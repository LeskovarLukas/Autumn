#include <map>
#include <functional>
#include <iomanip>
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "Point3D.h"

std::map<Point3D, Point3D> Point3D::points {};
std::map<long, Point3D* const> Point3D::pointCldDic {};

PointVls::PointVls(){}

PointVls::PointVls(float x, float y, float z){
  this->x = x;
  this->y = y;
  this->z = z;
}

Point3D::Point3D(){}

bool Point3D::operator<(const Point3D b) const
{
    double xdiff = point.x - b.point.x;
    double ydiff = point.y - b.point.y;
    double zdiff = point.z - b.point.z;
    return (xdiff + ydiff + zdiff) > 0;
}

Point3D::Point3D(float x, float y, float z){
  PointVls tmp(x, y, z);
  this->point = tmp;
}

Point3D::Point3D(PointVls p){
  this->point = p;
}

bool Point3D::equals(Point3D p){
  return p.point.x == this->point.x && p.point.y == this->point.y && p.point.z == this->point.z;
}

void Point3D::addPoints(pcl::PointCloud<pcl::PointXYZ> cloud){
  
  for(int index = 0; index < cloud.points.size(); index++){
  	float x = cloud.points[index].x;
  	float y = cloud.points[index].y;
  	float z = cloud.points[index].z;
    if(index  < 10){
      std::cout << "Point: x " << x << " y " << y << " z " << z << std::endl;
    }
    Point3D p (x, y, z);
    pointCldDic.insert({hashPoint(p.point), &p});
  }
}

long hashPoint(PointVls p){
  std::hash<float> hash_float;
  std::size_t h1 = hash_float(p.x);
  std::size_t h2 = hash_float(p.y);
  std::size_t h3 = hash_float(p.z);
  return h1 ^ (h2 << 1) ^ (h3 << 2);
}
