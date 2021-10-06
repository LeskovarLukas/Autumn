#ifndef point3D_H
#define point3D_H

#include <map>

struct PointVls{
  PointVls();
  PointVls(float x, float y, float z);

  float x;
  float y;
  float z;
};

struct Point3D
{
  //Consturctor
  Point3D();
  Point3D(float x, float y, float z);
  Point3D(PointVls p);

  //3D Point Map(key:Node, value:Parent)
  static std::map<Point3D, Point3D> points;
  static std::map<long, Point3D* const> pointCldDic; //Contains Point cloud points
  static void addPoints(pcl::PointCloud<pcl::PointXYZ> cloud);

  //Methods
  bool equals(Point3D p);
  bool operator<(const Point3D b) const;

  //Member Variables [Cords]
  PointVls point;

  bool valid;
  bool goal;
  bool start;
};

long hashPoint(PointVls p);

#endif
