#include <map>
#include "Point3D.h"

std::map<Point3D, Point3D> Point3D::points {};

Point3D::Point3D(){}

bool Point3D::operator<(const Point3D b) const
{
    double xdiff = x - b.x;
    double ydiff = y - b.y;
    double zdiff = z - b.z;
    return (xdiff + ydiff + zdiff) > 0;
}

Point3D::Point3D(float x, float y, float z){
  this->x = x;
  this->y = y;
  this->z = z;
}

bool Point3D::equals(Point3D p){
  return p.x == this->x && p.y == this->y && p.z == this->z;
}
