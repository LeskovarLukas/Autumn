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
  this->x = ajustCord(x);
  this->y = ajustCord(y);
  this->z = ajustCord(z);
}

float PointVls::ajustCord(float cord){
  return floor(cord*ajustConstant)/ajustConstant;
}

float PointVls::getX() const{
  return x;
}

float PointVls::getY() const{
  return y;
}

float PointVls::getZ() const{
  return z;
}

void PointVls::setX(float x){
  this->x = x;
}

void PointVls::setY(float y){
  this->y = y;
}

void PointVls::setZ(float z){
  this->z = z;
}

PointVls& PointVls::operator=(const PointVls& p){
  x = p.x;
  y = p.y;
  z = p.z;
  return *this;
}

Point3D::Point3D(){
  goal = false;
  start = false;
  valid = false;
}

bool Point3D::operator<(const Point3D b) const
{
    double xdiff = point.getX() - b.point.getX();
    double ydiff = point.getY() - b.point.getY();
    double zdiff = point.getZ() - b.point.getZ();
    return (xdiff + ydiff + zdiff) > 0;
}

Point3D::Point3D(float x, float y, float z){
  PointVls tmp(x, y, z);
  this->point = tmp;
}

Point3D& Point3D::operator=(const Point3D& p){
  point = p.point;
  start = p.start;
  valid = p.valid;
  goal = p.goal;
  return *this;
}

Point3D::Point3D(PointVls p){
  this->point = p;
}

bool Point3D::equals(Point3D p){
  return p.point.getX() == this->point.getX() && p.point.getY() == this->point.getY() && p.point.getZ() == this->point.getZ();
}

void Point3D::addPoints(pcl::PointCloud<pcl::PointXYZ> cloud){

  for(int index = 0; index < cloud.points.size(); index++){
  	float x = cloud.points[index].x;
  	float y = cloud.points[index].y;
  	float z = cloud.points[index].z;
    Point3D p (x, y, z);
    pointCldDic.insert({hashPoint(p.point), &p});
  }
}

long hashPoint(PointVls p){
  std::hash<float> hash_float;
  std::size_t h1 = hash_float(p.getX());
  std::size_t h2 = hash_float(p.getY());
  std::size_t h3 = hash_float(p.getZ());
  return h1 ^ (h2 << 1) ^ (h3 << 2);
}
