#include <map>
#include "Point3D.h"

std::map<Point3D, Point3D> Point3D::points {};

Point3D::Point3D(){}

bool operator<(const elem b) const
{
    double x = val.x - b.x;
    double y = val.y - b.y;
    double z = val.z - b.z;
    return (x + y + z) > 0;
}

Point3D::Point3D(float x, float y, float z){
  this->x = x;
  this->y = y;
  this->z = z;
}

bool Point3D::equals(Point3D p){
  return p.x == this->x && p.y == this->y && p.z == this->z;
}
