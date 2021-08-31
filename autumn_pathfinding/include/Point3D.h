#include <map>
#ifndef Point3D_H
#define Point3D_H

class Point3D
{
public:
  //Consturctor
  Point3D(int x, int y, int z);

  //3D Point Collection
  static std::map<Point3D, Point3D> points;

  //Member Variables [Cords]
  int x;
  int y;
  int z;
};

#endif
