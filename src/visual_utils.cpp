#include <visual_utils.h>

void VisualTools::visualizeRay(ros::Publisher pub, Ray ray, std::string frame) {
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id    = frame;
  line_strip.header.stamp       = ros::Time::now();
  line_strip.ns                 = "ray";
  line_strip.action             = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id                 = 1;
  line_strip.type               = visualization_msgs::Marker::LINE_STRIP;

  line_strip.scale.x = 0.002;

  line_strip.color.r = 1.0;
  line_strip.color.a = 1.0;

  geometry_msgs::Point p1, p2;

  p1.x = ray.p1[0];
  p1.y = ray.p1[1];
  p1.z = ray.p1[2];

  p2.x = ray.p2[0];
  p2.y = ray.p2[1];
  p2.z = ray.p2[2];

  line_strip.points.push_back(p1);
  line_strip.points.push_back(p2);
  pub.publish(line_strip);
}

void VisualTools::visualizePoint(ros::Publisher pub, Eigen::Vector3d p, std::string frame, double size, double r, double g, double b) {
  visualization_msgs::Marker marker;
  marker.header.frame_id    = frame;
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "point";
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.id                 = 0;
  marker.type               = visualization_msgs::Marker::POINTS;

  marker.scale.x = size;
  marker.scale.y = size;

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;

  geometry_msgs::Point gp;
  gp.x = p[0];
  gp.y = p[1];
  gp.z = p[2];

  marker.points.push_back(gp);

  pub.publish(marker);
}

void VisualTools::visualizeRect(ros::Publisher pub, Rectangle rect, std::string frame, double r, double g, double b) {
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id    = frame;
  line_strip.header.stamp       = ros::Time::now();
  line_strip.ns                 = "rect";
  line_strip.action             = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id                 = 1;
  line_strip.type               = visualization_msgs::Marker::LINE_STRIP;

  line_strip.scale.x = 0.002;

  line_strip.color.r = r;
  line_strip.color.g = g;
  line_strip.color.b = b;
  line_strip.color.a = 1.0;

  geometry_msgs::Point pa, pb, pc, pd;

  pa.x = rect.points[0][0];
  pa.y = rect.points[0][1];
  pa.z = rect.points[0][2];

  pb.x = rect.points[1][0];
  pb.y = rect.points[1][1];
  pb.z = rect.points[1][2];

  pc.x = rect.points[2][0];
  pc.y = rect.points[2][1];
  pc.z = rect.points[2][2];

  pd.x = rect.points[3][0];
  pd.y = rect.points[3][1];
  pd.z = rect.points[3][2];

  line_strip.points.push_back(pa);
  line_strip.points.push_back(pb);
  line_strip.points.push_back(pc);
  line_strip.points.push_back(pd);
  line_strip.points.push_back(pa);

  pub.publish(line_strip);
  ros::spinOnce();
}

BatchVisualizer::BatchVisualizer() {
}

BatchVisualizer::~BatchVisualizer() {
}

BatchVisualizer::BatchVisualizer(ros::NodeHandle rosNode, std::string frame) {
  this->frame      = frame;
  this->visual_pub = rosNode.advertise<visualization_msgs::MarkerArray>("/radiation/visualizer", 1);
}

void BatchVisualizer::addRay(Ray ray, double r, double g, double b) {
  ++marker_count;
  visualization_msgs::Marker marker;
  marker.header.frame_id    = frame;
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "ray";
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.id                 = marker_count;
  marker.type               = visualization_msgs::Marker::LINE_STRIP;

  marker.scale.x = 0.004;

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;

  geometry_msgs::Point p1, p2;

  p1.x = ray.p1[0];
  p1.y = ray.p1[1];
  p1.z = ray.p1[2];

  p2.x = ray.p2[0];
  p2.y = ray.p2[1];
  p2.z = ray.p2[2];

  marker.points.push_back(p1);
  marker.points.push_back(p2);
  msg.markers.push_back(marker);
}


void BatchVisualizer::addPoint(Eigen::Vector3d p, double r, double g, double b, double scale) {
  ++marker_count;
  visualization_msgs::Marker marker;
  marker.header.frame_id    = frame;
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "point";
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.id                 = marker_count;
  marker.type               = visualization_msgs::Marker::POINTS;

  marker.scale.x = scale;
  marker.scale.y = scale;

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 1.0;

  geometry_msgs::Point gp;
  gp.x = p[0];
  gp.y = p[1];
  gp.z = p[2];

  marker.points.push_back(gp);
  msg.markers.push_back(marker);
}

void BatchVisualizer::addRect(Rectangle rect) {
  ++marker_count;
  visualization_msgs::Marker marker;
  marker.header.frame_id    = frame;
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "rect";
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.id                 = marker_count;
  marker.type               = visualization_msgs::Marker::LINE_STRIP;

  marker.scale.x = 0.002;

  marker.color.r = 0.0;
  marker.color.g = 0.3;
  marker.color.b = 1.0;
  marker.color.a = 1.0;

  geometry_msgs::Point pa, pb, pc, pd;

  pa.x = rect.points[0][0];
  pa.y = rect.points[0][1];
  pa.z = rect.points[0][2];

  pb.x = rect.points[1][0];
  pb.y = rect.points[1][1];
  pb.z = rect.points[1][2];

  pc.x = rect.points[2][0];
  pc.y = rect.points[2][1];
  pc.z = rect.points[2][2];

  pd.x = rect.points[3][0];
  pd.y = rect.points[3][1];
  pd.z = rect.points[3][2];

  marker.points.push_back(pa);
  marker.points.push_back(pb);
  marker.points.push_back(pc);
  marker.points.push_back(pd);
  marker.points.push_back(pa);

  msg.markers.push_back(marker);
}

void BatchVisualizer::addEllipse(Ellipse e, double r, double g, double b) {
  ++marker_count;
  visualization_msgs::Marker marker;
  marker.header.frame_id    = frame;
  marker.header.stamp       = ros::Time::now();
  marker.ns                 = "point";
  marker.action             = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = e.phi;
  marker.id                 = marker_count;
  marker.type               = visualization_msgs::Marker::CYLINDER;

  marker.scale.x = e.a;
  marker.scale.y = e.b;
  marker.scale.z = 0.001;

  marker.pose.position.x = e.x;
  marker.pose.position.y = e.y;
  marker.pose.position.z = 0.0;


  Eigen::Quaterniond q =
      Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(e.phi, Eigen::Vector3d::UnitZ());


  marker.pose.orientation.w = q.w();
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();

  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = 0.4;

  msg.markers.push_back(marker);
}

void BatchVisualizer::addCuboid(Cuboid cuboid, double r, double g, double b) {
  ++marker_count;
  visualization_msgs::Marker marker_front;
  marker_front.header.frame_id    = frame;
  marker_front.header.stamp       = ros::Time::now();
  marker_front.ns                 = "cuboid";
  marker_front.action             = visualization_msgs::Marker::ADD;
  marker_front.pose.orientation.w = 1.0;
  marker_front.id                 = marker_count;
  marker_front.type               = visualization_msgs::Marker::LINE_STRIP;

  marker_front.scale.x = 0.002;

  marker_front.color.r = r;
  marker_front.color.g = g;
  marker_front.color.b = b;
  marker_front.color.a = 1.0;

  ++marker_count;
  visualization_msgs::Marker marker_back;
  marker_back    = marker_front;
  marker_back.id = marker_count;

  ++marker_count;
  visualization_msgs::Marker marker_af;
  marker_af    = marker_front;
  marker_af.id = marker_count;

  ++marker_count;
  visualization_msgs::Marker marker_be;
  marker_be    = marker_front;
  marker_be.id = marker_count;

  ++marker_count;
  visualization_msgs::Marker marker_dg;
  marker_dg    = marker_front;
  marker_dg.id = marker_count;

  ++marker_count;
  visualization_msgs::Marker marker_ch;
  marker_ch    = marker_front;
  marker_ch.id = marker_count;

  geometry_msgs::Point pa, pb, pc, pd, pe, pf, pg, ph;

  pa.x = cuboid.vertices[0][0];
  pa.y = cuboid.vertices[0][1];
  pa.z = cuboid.vertices[0][2];

  pb.x = cuboid.vertices[1][0];
  pb.y = cuboid.vertices[1][1];
  pb.z = cuboid.vertices[1][2];

  pc.x = cuboid.vertices[2][0];
  pc.y = cuboid.vertices[2][1];
  pc.z = cuboid.vertices[2][2];

  pd.x = cuboid.vertices[3][0];
  pd.y = cuboid.vertices[3][1];
  pd.z = cuboid.vertices[3][2];

  pe.x = cuboid.vertices[4][0];
  pe.y = cuboid.vertices[4][1];
  pe.z = cuboid.vertices[4][2];

  pf.x = cuboid.vertices[5][0];
  pf.y = cuboid.vertices[5][1];
  pf.z = cuboid.vertices[5][2];

  pg.x = cuboid.vertices[6][0];
  pg.y = cuboid.vertices[6][1];
  pg.z = cuboid.vertices[6][2];

  ph.x = cuboid.vertices[7][0];
  ph.y = cuboid.vertices[7][1];
  ph.z = cuboid.vertices[7][2];

  marker_front.points.push_back(pa);
  marker_front.points.push_back(pb);
  marker_front.points.push_back(pc);
  marker_front.points.push_back(pd);
  marker_front.points.push_back(pa);

  marker_back.points.push_back(pe);
  marker_back.points.push_back(pf);
  marker_back.points.push_back(pg);
  marker_back.points.push_back(ph);
  marker_back.points.push_back(pe);

  marker_af.points.push_back(pa);
  marker_af.points.push_back(pf);

  marker_be.points.push_back(pb);
  marker_be.points.push_back(pe);

  marker_dg.points.push_back(pd);
  marker_dg.points.push_back(pg);

  marker_ch.points.push_back(pc);
  marker_ch.points.push_back(ph);

  msg.markers.push_back(marker_front);
  msg.markers.push_back(marker_back);
  msg.markers.push_back(marker_af);
  msg.markers.push_back(marker_be);
  msg.markers.push_back(marker_dg);
  msg.markers.push_back(marker_ch);
}

void BatchVisualizer::clear() {
  msg.markers.clear();
  marker_count = 0;
}

void BatchVisualizer::publish() {
  visual_pub.publish(msg);
}
