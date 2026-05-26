#include "bt_manager/area_visualizer.hpp"

#include "bt_manager/utils/area.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace Sentry_BT {

namespace {

bool shouldFillBasePolygon(const std::string & name)
{
  return name != "own_defense_zone" && name != "enemy_defense_zone" && name != "highland_zone" &&
         name != "engeneering_zone";
}

void appendPolygonFillTriangles(
  visualization_msgs::msg::Marker & fill_marker, const std::vector<Sentry_BT::Point2D> & vertices, double z)
{
  if (vertices.size() < 3) {
    return;
  }

  for (std::size_t i = 1; i + 1 < vertices.size(); ++i) {
    geometry_msgs::msg::Point p0;
    p0.x = vertices[0].x;
    p0.y = vertices[0].y;
    p0.z = z;
    geometry_msgs::msg::Point p1;
    p1.x = vertices[i].x;
    p1.y = vertices[i].y;
    p1.z = z;
    geometry_msgs::msg::Point p2;
    p2.x = vertices[i + 1].x;
    p2.y = vertices[i + 1].y;
    p2.z = z;
    fill_marker.points.push_back(p0);
    fill_marker.points.push_back(p1);
    fill_marker.points.push_back(p2);
    // Add reverse winding to avoid back-face culling from different viewpoints.
    fill_marker.points.push_back(p0);
    fill_marker.points.push_back(p2);
    fill_marker.points.push_back(p1);
  }
}

void appendCircleLineStrip(
  visualization_msgs::msg::Marker & line_marker, const Sentry_BT::Area_Circle & circle, double z)
{
  constexpr std::size_t kSegments = 48;
  const double step = 2.0 * M_PI / static_cast<double>(kSegments);

  line_marker.points.reserve(kSegments + 1);
  for (std::size_t i = 0; i <= kSegments; ++i) {
    const double angle = step * static_cast<double>(i);
    geometry_msgs::msg::Point p;
    p.x = circle.center.x + circle.radius * std::cos(angle);
    p.y = circle.center.y + circle.radius * std::sin(angle);
    p.z = z;
    line_marker.points.push_back(p);
  }
}

}  // namespace

AreaVisualizer::AreaVisualizer(rclcpp::Node & node)
{
  rclcpp::QoS area_marker_qos(1);
  area_marker_qos.transient_local().reliable();
  area_marker_pub_ =
    node.create_publisher<visualization_msgs::msg::MarkerArray>("/sentry/area_markers", area_marker_qos);
}

void AreaVisualizer::publishAreaMarkers(const rclcpp::Time & now)
{
  if (!area_marker_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  const auto square_areas = getAreaVizConfigs();
  const auto base_polygon_areas = getBasePolygonVizConfigs();
  const auto tracking_polygon_areas = getTrackingPolygonVizConfigs();
  const auto circle_areas = getCircleVizConfigs();

  int marker_id = 0;
  int tunnel_zone_index = 0;
  for (const auto & cfg : square_areas) {
    std::string display_name = cfg.name;
    if (cfg.name == "tunnel_zone") {
      display_name = cfg.name + "[" + std::to_string(tunnel_zone_index++) + "]";
    }

    const double min_x = std::min(cfg.area.top_left.x, cfg.area.bottom_right.x);
    const double max_x = std::max(cfg.area.top_left.x, cfg.area.bottom_right.x);
    const double min_y = std::min(cfg.area.top_left.y, cfg.area.bottom_right.y);
    const double max_y = std::max(cfg.area.top_left.y, cfg.area.bottom_right.y);

    visualization_msgs::msg::Marker box_marker;
    box_marker.header.frame_id = "map";
    box_marker.header.stamp = now;
    box_marker.ns = "sentry_area_box/" + display_name;
    box_marker.id = marker_id++;
    box_marker.type = visualization_msgs::msg::Marker::CUBE;
    box_marker.action = visualization_msgs::msg::Marker::ADD;
    box_marker.pose.position.x = (min_x + max_x) * 0.5;
    box_marker.pose.position.y = (min_y + max_y) * 0.5;
    box_marker.pose.position.z = 0.05;
    box_marker.pose.orientation.w = 1.0;
    box_marker.scale.x = std::max(0.05, max_x - min_x);
    box_marker.scale.y = std::max(0.05, max_y - min_y);
    box_marker.scale.z = 0.1;
    box_marker.color.r = cfg.color[0];
    box_marker.color.g = cfg.color[1];
    box_marker.color.b = cfg.color[2];
    box_marker.color.a = 0.35F;
    marker_array.markers.push_back(box_marker);

    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "map";
    text_marker.header.stamp = now;
    text_marker.ns = "sentry_area_label/" + display_name;
    text_marker.id = marker_id++;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = box_marker.pose.position.x;
    text_marker.pose.position.y = box_marker.pose.position.y;
    text_marker.pose.position.z = 0.35;
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.z = 0.25;
    text_marker.color.r = cfg.color[0];
    text_marker.color.g = cfg.color[1];
    text_marker.color.b = cfg.color[2];
    text_marker.color.a = 1.0F;
    text_marker.text = display_name;
    marker_array.markers.push_back(text_marker);
  }

  for (const auto & cfg : base_polygon_areas) {
    if (cfg.vertices.size() < 3) {
      continue;
    }

    if (shouldFillBasePolygon(cfg.name)) {
      visualization_msgs::msg::Marker fill_marker;
      fill_marker.header.frame_id = "map";
      fill_marker.header.stamp = now;
      fill_marker.ns = "sentry_area_poly_fill/base/" + cfg.name;
      fill_marker.id = marker_id++;
      fill_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
      fill_marker.action = visualization_msgs::msg::Marker::ADD;
      fill_marker.pose.orientation.w = 1.0;
      fill_marker.color.r = cfg.color[0];
      fill_marker.color.g = cfg.color[1];
      fill_marker.color.b = cfg.color[2];
      fill_marker.color.a = 0.35F;
      appendPolygonFillTriangles(fill_marker, cfg.vertices, 0.14);
      marker_array.markers.push_back(fill_marker);
    }

    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = now;
    line_marker.ns = "sentry_area_poly/base/" + cfg.name;
    line_marker.id = marker_id++;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.pose.orientation.w = 1.0;
    line_marker.scale.x = 0.08;
    line_marker.color.r = cfg.color[0];
    line_marker.color.g = cfg.color[1];
    line_marker.color.b = cfg.color[2];
    line_marker.color.a = 1.0F;

    double center_x = 0.0;
    double center_y = 0.0;
    for (const auto & vertex : cfg.vertices) {
      geometry_msgs::msg::Point p;
      p.x = vertex.x;
      p.y = vertex.y;
      p.z = 0.15;
      line_marker.points.push_back(p);
      center_x += vertex.x;
      center_y += vertex.y;
    }
    line_marker.points.push_back(line_marker.points.front());
    marker_array.markers.push_back(line_marker);

    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "map";
    text_marker.header.stamp = now;
    text_marker.ns = "sentry_area_poly_label/base/" + cfg.name;
    text_marker.id = marker_id++;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = center_x / static_cast<double>(cfg.vertices.size());
    text_marker.pose.position.y = center_y / static_cast<double>(cfg.vertices.size());
    text_marker.pose.position.z = 0.45;
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.z = 0.25;
    text_marker.color.r = cfg.color[0];
    text_marker.color.g = cfg.color[1];
    text_marker.color.b = cfg.color[2];
    text_marker.color.a = 1.0F;
    text_marker.text = cfg.name;
    marker_array.markers.push_back(text_marker);
  }

  for (const auto & cfg : tracking_polygon_areas) {
    if (cfg.vertices.size() < 3) {
      continue;
    }

    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = now;
    line_marker.ns = "sentry_area_poly/tracking/" + cfg.name;
    line_marker.id = marker_id++;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.pose.orientation.w = 1.0;
    line_marker.scale.x = 0.1;
    line_marker.color.r = cfg.color[0];
    line_marker.color.g = cfg.color[1];
    line_marker.color.b = cfg.color[2];
    line_marker.color.a = 1.0F;

    double center_x = 0.0;
    double center_y = 0.0;
    for (const auto & vertex : cfg.vertices) {
      geometry_msgs::msg::Point p;
      p.x = vertex.x;
      p.y = vertex.y;
      p.z = 0.22;
      line_marker.points.push_back(p);
      center_x += vertex.x;
      center_y += vertex.y;
    }
    line_marker.points.push_back(line_marker.points.front());
    marker_array.markers.push_back(line_marker);

    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "map";
    text_marker.header.stamp = now;
    text_marker.ns = "sentry_area_poly_label/tracking/" + cfg.name;
    text_marker.id = marker_id++;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = center_x / static_cast<double>(cfg.vertices.size()) + 0.35;
    text_marker.pose.position.y = center_y / static_cast<double>(cfg.vertices.size()) + 0.35;
    text_marker.pose.position.z = 0.6;
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.z = 0.28;
    text_marker.color.r = cfg.color[0];
    text_marker.color.g = cfg.color[1];
    text_marker.color.b = cfg.color[2];
    text_marker.color.a = 1.0F;
    text_marker.text = "tracking_" + cfg.name;
    marker_array.markers.push_back(text_marker);
  }

  for (const auto & cfg : circle_areas) {
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = now;
    line_marker.ns = "sentry_area_circle/" + cfg.name;
    line_marker.id = marker_id++;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.pose.orientation.w = 1.0;
    line_marker.scale.x = 0.08;
    line_marker.color.r = cfg.color[0];
    line_marker.color.g = cfg.color[1];
    line_marker.color.b = cfg.color[2];
    line_marker.color.a = 1.0F;
    appendCircleLineStrip(line_marker, cfg.area, 0.18);
    marker_array.markers.push_back(line_marker);

    visualization_msgs::msg::Marker text_marker;
    text_marker.header.frame_id = "map";
    text_marker.header.stamp = now;
    text_marker.ns = "sentry_area_circle_label/" + cfg.name;
    text_marker.id = marker_id++;
    text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::msg::Marker::ADD;
    text_marker.pose.position.x = cfg.area.center.x;
    text_marker.pose.position.y = cfg.area.center.y;
    text_marker.pose.position.z = 0.45;
    text_marker.pose.orientation.w = 1.0;
    text_marker.scale.z = 0.25;
    text_marker.color.r = cfg.color[0];
    text_marker.color.g = cfg.color[1];
    text_marker.color.b = cfg.color[2];
    text_marker.color.a = 1.0F;
    text_marker.text = cfg.name;
    marker_array.markers.push_back(text_marker);
  }

  area_marker_pub_->publish(marker_array);
}

void AreaVisualizer::clearAreaMarkers(const rclcpp::Time & now)
{
  if (!area_marker_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray marker_array;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = now;
  marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(marker);
  area_marker_pub_->publish(marker_array);
}

}  // namespace Sentry_BT
