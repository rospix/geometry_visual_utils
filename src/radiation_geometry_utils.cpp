#include <geometry_visual_utils/radiation_geometry_utils.h>
#include <iostream>
#include <ros/ros.h>

/* Line2D */  //{
Line2D::Line2D(Eigen::Vector2d normal, double c) {
  this->normal = normal;
  this->c      = c;
}
Line2D::~Line2D() {
}

//}

/* Ray */  //{
Ray::Ray() {
  this->p1 = Eigen::Vector3d(0.0, 0.0, 0.0);
  this->p2 = Eigen::Vector3d(0.0, 0.0, 0.0);
}

Ray::~Ray() {
}

Ray::Ray(Eigen::Vector3d p1, Eigen::Vector3d p2) {
  this->p1        = p1;
  this->p2        = p2;
  this->direction = p2 - p1;
}
//}

/* Plane */  //{
Plane::Plane() {
}

Plane::Plane(Eigen::Vector3d point, Eigen::Vector3d normal) {
  this->point  = point;
  this->normal = normal;
  this->d      = -(normal.dot(point));
}

Plane::~Plane() {
}

boost::optional<Eigen::Vector3d> Plane::intersectionRay(Ray r, double epsilon) {
  double denom = this->normal.dot(r.p2 - r.p1);
  if (abs(denom) < epsilon) {
    return boost::optional<Eigen::Vector3d>{};
  }
  double t = this->normal.dot(this->point - r.p1) / denom;
  if (t >= 0) {
    return Eigen::Vector3d(r.p1 + t * r.direction);
  } else {
    return boost::optional<Eigen::Vector3d>{};
  }
}
//}

/* Rectangle */  //{
Rectangle::Rectangle() {
}

Rectangle::~Rectangle() {
}

Rectangle::Rectangle(Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d C, Eigen::Vector3d D) {

  Eigen::Vector3d v1 = B - A;
  Eigen::Vector3d v2 = D - A;

  this->points.push_back(A);
  this->points.push_back(B);
  this->points.push_back(C);
  this->points.push_back(D);

  this->normal_vector = v1.cross(v2);
  this->normal_vector.normalize();

  if (A == B || A == C || A == D || B == C || B == D || C == D) {
    return;
  }

  this->plane = Plane(A, normal_vector);

  this->basis.col(0) << B - A;
  this->basis.col(1) << D - A;
  this->basis.col(2) << normal_vector;

  this->projector = basis * basis.transpose();
}

boost::optional<Eigen::Vector3d> Rectangle::intersectionRay(Ray r, double epsilon) {
  boost::optional<Eigen::Vector3d> intersect = this->plane.intersectionRay(r, epsilon);
  if (!intersect) {
    return intersect;
  }
  Eigen::Vector3d projection = basis.inverse() * (intersect.get() - points[0]);
  if (projection[0] >= 0.0 && projection[0] <= 1.0 && projection[1] >= 0.0 && projection[1] <= 1.0) {
    return intersect;
  }
  return boost::optional<Eigen::Vector3d>{};
}
//}

/* Ellipse */  //{
Ellipse::Ellipse() {
}

Ellipse::~Ellipse() {
}

Ellipse::Ellipse(double a, double b, double x, double y, double phi) {
  this->a   = a;
  this->b   = b;
  this->x   = x;
  this->y   = y;
  this->phi = phi;
}
//}

/* Cuboid */  //{
Cuboid::Cuboid() {
}

Cuboid::~Cuboid() {
}

Cuboid::Cuboid(Eigen::Vector3d center, Eigen::Quaterniond orientation, double depth, double width, double height) {

  Eigen::Vector3d A = orientation * Eigen::Vector3d(depth / 2.0, -width / 2.0, -height / 2.0) + center;
  Eigen::Vector3d B = orientation * Eigen::Vector3d(depth / 2.0, width / 2.0, -height / 2.0) + center;
  Eigen::Vector3d C = orientation * Eigen::Vector3d(depth / 2.0, width / 2.0, height / 2.0) + center;
  Eigen::Vector3d D = orientation * Eigen::Vector3d(depth / 2.0, -width / 2.0, height / 2.0) + center;

  Eigen::Vector3d E = orientation * Eigen::Vector3d(-depth / 2.0, width / 2.0, -height / 2.0) + center;
  Eigen::Vector3d F = orientation * Eigen::Vector3d(-depth / 2.0, -width / 2.0, -height / 2.0) + center;
  Eigen::Vector3d G = orientation * Eigen::Vector3d(-depth / 2.0, -width / 2.0, height / 2.0) + center;
  Eigen::Vector3d H = orientation * Eigen::Vector3d(-depth / 2.0, width / 2.0, height / 2.0) + center;

  Rectangle front(A, B, C, D);
  Rectangle back(E, F, G, H);
  Rectangle left(B, E, H, C);
  Rectangle right(F, A, D, G);
  Rectangle bottom(F, E, B, A);
  Rectangle top(D, C, H, G);

  sides.push_back(front);
  sides.push_back(back);
  sides.push_back(left);
  sides.push_back(right);
  sides.push_back(bottom);
  sides.push_back(top);

  this->vertices.clear();
  this->vertices.push_back(A);
  this->vertices.push_back(B);
  this->vertices.push_back(C);
  this->vertices.push_back(D);
  this->vertices.push_back(E);
  this->vertices.push_back(F);
  this->vertices.push_back(G);
  this->vertices.push_back(H);
}

Cuboid::Cuboid(Eigen::Vector3d A, Eigen::Vector3d B, Eigen::Vector3d C, Eigen::Vector3d D, Eigen::Vector3d E, Eigen::Vector3d F, Eigen::Vector3d G,
               Eigen::Vector3d H) {
  this->vertices.clear();
  this->vertices.push_back(A);
  this->vertices.push_back(B);
  this->vertices.push_back(C);
  this->vertices.push_back(D);
  this->vertices.push_back(E);
  this->vertices.push_back(F);
  this->vertices.push_back(G);
  this->vertices.push_back(H);

  Rectangle front(A, B, C, D);
  Rectangle back(E, F, G, H);
  Rectangle left(B, E, H, C);
  Rectangle right(F, A, D, G);
  Rectangle bottom(F, E, B, A);
  Rectangle top(D, C, H, G);

  sides.push_back(front);
  sides.push_back(back);
  sides.push_back(left);
  sides.push_back(right);
  sides.push_back(bottom);
  sides.push_back(top);
}
//}

/* Miscellaneous */  //{
double haversin(double angle) {
  return (1.0 - std::cos(angle)) / 2.0;
}

double invHaversin(double angle) {
  return 2.0 * std::asin(std::sqrt(angle));
}

double triangleArea(double a, double b, double c) {
  double s = (a + b + c) / 2.0;
  return std::sqrt(s * (s - a) * (s - b) * (s - c));
}


double vectorAngle(Eigen::Vector3d v1, Eigen::Vector3d v2) {
  return acos(v1.dot(v2) / (v1.norm() * v2.norm()));
}

double solidAngle(double a, double b, double c) {
  return invHaversin((haversin(c) - haversin(a - b)) / (std::sin(a) * std::sin(b)));
}

double sphericalTriangleArea(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c) {
  double ab = vectorAngle(a, b);
  double bc = vectorAngle(b, c);
  double ca = vectorAngle(c, a);

  if (ab < 1e-3 and bc < 1e-3 and ca < 1e-3) {
    return triangleArea(ab, bc, ca);
  }

  double A = solidAngle(ca, ab, bc);
  double B = solidAngle(ab, bc, ca);
  double C = solidAngle(bc, ca, ab);

  return A + B + C - M_PI;
}

double rectSolidAngle(Rectangle r, Eigen::Vector3d center) {
  Eigen::Vector3d a = r.points[0] - center;
  Eigen::Vector3d b = r.points[1] - center;
  Eigen::Vector3d c = r.points[2] - center;
  Eigen::Vector3d d = r.points[3] - center;

  a.normalize();
  b.normalize();
  c.normalize();
  d.normalize();

  double t1 = sphericalTriangleArea(a, b, c);
  double t2 = sphericalTriangleArea(c, d, a);

  return t1 + t2;
}

Eigen::Vector3d pos3toVector3d(ignition::math::Pose3d gzpos) {
  Eigen::Vector3d v;
  v[0] = gzpos.Pos().X();
  v[1] = gzpos.Pos().Y();
  v[2] = gzpos.Pos().Z();
  return v;
}

Eigen::Quaterniond pos3toQuaterniond(ignition::math::Pose3d gzpos) {
  Eigen::Quaterniond q(gzpos.Rot().W(), gzpos.Rot().X(), gzpos.Rot().Y(), gzpos.Rot().Z());
  return q;
}

Rectangle move(Rectangle r, Eigen::Vector3d translation, Eigen::Quaterniond rotation) {
  Rectangle ret;
  for (int i = 0; i < 4; i++) {
    ret.points.push_back((rotation * r.points[i]) + translation);
  }
  Eigen::Vector3d v1 = ret.points[1] - ret.points[0];
  Eigen::Vector3d v2 = ret.points[3] - ret.points[0];

  ret.normal_vector = v1.cross(v2);
  ret.normal_vector.normalize();

  ret.plane = Plane(ret.points[0], ret.normal_vector);

  ret.basis.col(0) << ret.points[1] - ret.points[0];
  ret.basis.col(1) << ret.points[3] - ret.points[0];
  ret.basis.col(2) << ret.normal_vector;

  ret.projector = ret.basis * ret.basis.transpose();

  return ret;
}

Eigen::Vector2d *intersection(Line2D l1, Line2D l2) {
  double denom = l1.normal[0] * l2.normal[1] - l1.normal[1] * l2.normal[0];
  if (denom == 0) {
    return nullptr;
  }
  Eigen::Vector2d *ret = new Eigen::Vector2d(0, 0);
  ret->x()             = (l1.c * l2.normal[1] - l2.c * l1.normal[1]) / denom;
  ret->y()             = (l1.normal[0] * l2.c - l2.normal[0] * l1.c) / denom;
  return ret;
}
//}
