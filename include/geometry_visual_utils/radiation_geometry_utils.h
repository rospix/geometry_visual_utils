#ifndef GEOMETRY_UTILS_H
#define GEOMETRY_UTILS_H

#include <vector>
#include <math.h>
#include <boost/optional.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <ignition/math.hh>

class Ray {
public:
  Ray();
  ~Ray();
  Ray(Eigen::Vector3d p1, Eigen::Vector3d p2);

  Eigen::Vector3d p1;
  Eigen::Vector3d p2;
  Eigen::Vector3d direction;

  static Ray twopointCast(Eigen::Vector3d pointFrom, Eigen::Vector3d pointTo) {
    return Ray(pointFrom, pointTo);
  }
  static Ray directionCast(Eigen::Vector3d origin, Eigen::Vector3d direction) {
    return Ray(origin, origin + direction);
  }
};

class Line2D {
  public:
    Line2D();
    ~Line2D();
    Line2D(Eigen::Vector2d normal, double c);

    Eigen::Vector2d normal;
    double c;
};

class Plane {
public:
  Plane();
  ~Plane();
  Plane(Eigen::Vector3d point, Eigen::Vector3d normal);

  Eigen::Vector3d point;
  Eigen::Vector3d normal;
  double          d = 0;

  boost::optional<Eigen::Vector3d> intersectionRay(Ray r, double epsilon = 1e-16);
};


class Rectangle {
public:
  Rectangle();
  ~Rectangle();
  Rectangle(Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d C, Eigen::Vector3d D);

  Eigen::Vector3d normal_vector;

  Eigen::Matrix3d basis;
  Eigen::Matrix3d projector;

  Plane plane;

  std::vector<Eigen::Vector3d> points;

  boost::optional<Eigen::Vector3d> intersectionRay(Ray r, double epsilon = 1e-16);
};

class Ellipse {

public:
  Ellipse();
  ~Ellipse();
  Ellipse(double a, double b, double x, double y, double phi);

  double a, b, x, y, phi;

};

class Cuboid {
public:
  Cuboid();
  ~Cuboid();
  Cuboid(Eigen::Vector3d center, Eigen::Quaterniond orientation, double depth, double width, double height);
  Cuboid(Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d C, Eigen::Vector3d D, Eigen::Vector3d E, Eigen::Vector3d F, Eigen::Vector3d G,
         Eigen::Vector3d H);

  std::vector<Eigen::Vector3d> vertices;
  std::vector<Rectangle>       sides;
};

double haversin(double angle);
double invHaversin(double angle);

double vectorAngle(Eigen::Vector3d v1, Eigen::Vector3d v2);
double solidAngle(double a, double b, double c);
double rectSolidAngle(Rectangle r, Eigen::Vector3d center);

double triangleArea(double a, double b, double c);
double sphericalTriangleArea(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c);

Eigen::Vector3d    pos3toVector3d(ignition::math::Pose3d gzpos);
Eigen::Quaterniond pos3toQuaterniond(ignition::math::Pose3d gzpos);
Rectangle          move(Rectangle r, Eigen::Vector3d translation, Eigen::Quaterniond rotation);

Eigen::Vector2d *intersection(Line2D l1, Line2D l2);

#endif
