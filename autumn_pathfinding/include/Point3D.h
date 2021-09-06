#ifndef point3D_H
#define point3D_H

#include <map>

class Point3D
{
public:
  //Consturctor
  Point3D();
  Point3D(float x, float y, float z);

  //3D Point Map(key:Node, value:Parent)
  static std::map<Point3D, Point3D> points;

  //Methods
  bool equals(Point3D p);
  bool operator<(const Point3D b) const;

  //Member Variables [Cords]
  float x;
  float y;
  float z;

  bool valid;
  bool goal;
  bool start;
};

#endif
