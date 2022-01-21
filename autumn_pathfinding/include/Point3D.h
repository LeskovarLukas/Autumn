#ifndef point3D_H
#define point3D_H

#include <map>

class PointVls{
public:
  PointVls();
  PointVls(float x, float y, float z);
  float getX() const;
  float getY() const;
  float getZ() const;

  void setX(float x);
  void setY(float y);
  void setZ(float z);

 PointVls& operator=(const PointVls& p);

private:
  const int ajustConstant{100};

  float x{};
  float y{};
  float z{};

  float ajustCord(float cord);
};

std::ostream& operator<<(std::ostream& os, const PointVls& vls);

struct Point3D
{
  //Consturctor
  Point3D();
  Point3D(float x, float y, float z);
  Point3D(PointVls p);

  Point3D& operator=(const Point3D& p);

  //3D Point Map(key:Node, value:Parent)
  static std::map<Point3D, Point3D> points;
  static std::map<long, Point3D* const> pointCldDic; //Contains Point cloud points
  static void addPoints(pcl::PointCloud<pcl::PointXYZ> cloud);

  //Methods
  bool equals(Point3D p);
  bool operator<(const Point3D b) const;
  bool start();

  //Member Variables [Cords]
  static PointVls* point_start;
  PointVls point;

  bool valid{true};
  bool goal{false};
};

long hashPoint(PointVls p);

#endif
