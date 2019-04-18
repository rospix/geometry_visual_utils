#ifndef VISUAL_UTILS_H
#define VISUAL_UTILS_H

#include <ros/ros.h>
#include <vector>
#include <ignition/math.hh>
#include <eigen3/Eigen/Core>
#include "geometry_utils.h"
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>

class VisualTools {
public:
  static void visualizeRay(ros::Publisher pub, Ray ray, std::string frame);
  static void visualizePoint(ros::Publisher pub, Eigen::Vector3d p, std::string frame, double size, double r = 0.0, double g = 1.0, double b = 0.0);
  static void visualizeRect(ros::Publisher pub, Rectangle rect, std::string frame, double r = 0.0, double g = 0.7, double b = 1.0);
  static void visualizeCuboid(ros::Publisher pub, Cuboid cuboid, std::string frame, double r = 0.0, double g = 0.7, double b = 1.0);
};

class BatchVisualizer {
public:
  BatchVisualizer();
  ~BatchVisualizer();
  BatchVisualizer(ros::NodeHandle rosNode, std::string frame);

  void addRay(Ray ray, double r = 1.0, double g = 0.0, double b = 0.0);
  void addPoint(Eigen::Vector3d p);
  void addRect(Rectangle rect);
  void addCuboid(Cuboid cuboid, double r = 0.0, double g = 0.6, double b = 1.0);
  void addEllipse(Ellipse e, double r = 1.0, double g = 0.2, double b = 0.2);

  void clear();
  void publish();

  ros::Publisher                  visual_pub;
  visualization_msgs::MarkerArray msg;

private:
  std::string                  frame;
  std::vector<Ray>             rays;
  std::vector<Eigen::Vector3d> points;
  std::vector<Rectangle>       rects;
  std::vector<Cuboid>          cuboids;

  int marker_count = 0;
};

#endif
