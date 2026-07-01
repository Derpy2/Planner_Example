#include "visualization/visualization_manager.h"

#include <rclcpp/rclcpp.hpp>

namespace visualization {

VisualizationManager& VisualizationManager::Instance() {
  static VisualizationManager instance;
  return instance;
}

void VisualizationManager::AddMarker(const VisualData& data) {
  std::lock_guard<std::mutex> lock(mutex_);
  markers_[data.ns][data.id] = data;
}

void VisualizationManager::RemoveMarker(const std::string& ns, int id) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto it_ns = markers_.find(ns);
  if (it_ns != markers_.end()) {
    it_ns->second.erase(id);
    if (it_ns->second.empty()) {
      markers_.erase(it_ns);
    }
  }
}

void VisualizationManager::ClearAll() {
  std::lock_guard<std::mutex> lock(mutex_);
  markers_.clear();
}

void VisualizationManager::ClearNamespace(const std::string& ns) {
  std::lock_guard<std::mutex> lock(mutex_);
  markers_.erase(ns);
}

visualization_msgs::msg::MarkerArray VisualizationManager::GetAllMarkers() {
  std::lock_guard<std::mutex> lock(mutex_);
  visualization_msgs::msg::MarkerArray array;
  for (const auto& ns_pair : markers_) {
    for (const auto& id_pair : ns_pair.second) {
      array.markers.push_back(ConvertToMarker(id_pair.second));
    }
  }
  return array;
}

// ========== 便捷API实现 ==========

void VisualizationManager::AddPoint(const std::string& ns, int id,
                                    const geometry_msgs::msg::Point& pt,
                                    const Color& color, float scale,
                                    const std::string& frame_id) {
  VisualData data;
  data.ns = ns;
  data.id = id;
  data.shape = ShapeType::POINT;
  data.points.push_back(pt);
  data.color = color;
  data.scale_x = scale;
  data.scale_y = scale;
  data.scale_z = scale;
  data.frame_id = frame_id;
  AddMarker(data);
}

void VisualizationManager::AddLine(const std::string& ns, int id,
                                   const geometry_msgs::msg::Point& start,
                                   const geometry_msgs::msg::Point& end,
                                   const Color& color, float scale,
                                   const std::string& frame_id) {
  VisualData data;
  data.ns = ns;
  data.id = id;
  data.shape = ShapeType::LINE;
  data.points.push_back(start);
  data.points.push_back(end);
  data.color = color;
  data.scale_x = scale;
  data.frame_id = frame_id;
  AddMarker(data);
}

void VisualizationManager::AddLineStrip(
    const std::string& ns, int id,
    const std::vector<geometry_msgs::msg::Point>& points, const Color& color,
    float scale, const std::string& frame_id) {
  VisualData data;
  data.ns = ns;
  data.id = id;
  data.shape = ShapeType::LINE_LIST;
  data.points = points;
  data.color = color;
  data.scale_x = scale;
  data.frame_id = frame_id;
  AddMarker(data);
}

void VisualizationManager::AddPolygon(
    const std::string& ns, int id,
    const std::vector<geometry_msgs::msg::Point>& points, const Color& color,
    float scale, const std::string& frame_id) {
  VisualData data;
  data.ns = ns;
  data.id = id;
  data.shape = ShapeType::POLYGON;
  data.points = points;
  // 闭合多边形
  if (!points.empty() && (points.front().x != points.back().x ||
                          points.front().y != points.back().y ||
                          points.front().z != points.back().z)) {
    data.points.push_back(points.front());
  }
  data.color = color;
  data.scale_x = scale;
  data.frame_id = frame_id;
  AddMarker(data);
}

void VisualizationManager::AddArrow(const std::string& ns, int id,
                                    const geometry_msgs::msg::Point& start,
                                    const geometry_msgs::msg::Point& end,
                                    const Color& color, float scale,
                                    const std::string& frame_id) {
  VisualData data;
  data.ns = ns;
  data.id = id;
  data.shape = ShapeType::ARROW;
  data.points.push_back(start);
  data.points.push_back(end);
  data.color = color;
  data.scale_x = scale;
  data.scale_y = scale * 0.2f;
  data.scale_z = scale * 0.2f;
  data.frame_id = frame_id;
  AddMarker(data);
}

void VisualizationManager::AddCube(const std::string& ns, int id,
                                   const geometry_msgs::msg::Point& center,
                                   const geometry_msgs::msg::Point& size,
                                   const Color& color,
                                   const std::string& frame_id) {
  VisualData data;
  data.ns = ns;
  data.id = id;
  data.shape = ShapeType::CUBE;
  data.points.push_back(center);
  data.color = color;
  data.scale_x = size.x;
  data.scale_y = size.y;
  data.scale_z = size.z;
  data.frame_id = frame_id;
  AddMarker(data);
}

void VisualizationManager::AddSphere(const std::string& ns, int id,
                                     const geometry_msgs::msg::Point& center,
                                     float radius, const Color& color,
                                     const std::string& frame_id) {
  VisualData data;
  data.ns = ns;
  data.id = id;
  data.shape = ShapeType::SPHERE;
  data.points.push_back(center);
  data.color = color;
  data.scale_x = radius * 2.0f;
  data.scale_y = radius * 2.0f;
  data.scale_z = radius * 2.0f;
  data.frame_id = frame_id;
  AddMarker(data);
}

void VisualizationManager::AddCylinder(const std::string& ns, int id,
                                       const geometry_msgs::msg::Point& center,
                                       float radius, float height,
                                       const Color& color,
                                       const std::string& frame_id) {
  VisualData data;
  data.ns = ns;
  data.id = id;
  data.shape = ShapeType::CYLINDER;
  data.points.push_back(center);
  data.color = color;
  data.scale_x = radius * 2.0f;
  data.scale_y = radius * 2.0f;
  data.scale_z = height;
  data.frame_id = frame_id;
  AddMarker(data);
}

void VisualizationManager::AddText(const std::string& ns, int id,
                                   const geometry_msgs::msg::Point& position,
                                   const std::string& text, const Color& color,
                                   float scale, const std::string& frame_id) {
  VisualData data;
  data.ns = ns;
  data.id = id;
  data.shape = ShapeType::TEXT;
  data.points.push_back(position);
  data.color = color;
  data.scale_z = scale;  // 文本大小
  data.text = text;
  data.frame_id = frame_id;
  AddMarker(data);
}

// ========== 内部转换 ==========

visualization_msgs::msg::Marker VisualizationManager::ConvertToMarker(
    const VisualData& data) {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = data.frame_id;
  marker.header.stamp = rclcpp::Clock().now();
  marker.ns = data.ns;
  marker.id = data.id;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.color.r = data.color.r;
  marker.color.g = data.color.g;
  marker.color.b = data.color.b;
  marker.color.a = data.color.a;

  marker.scale.x = data.scale_x;
  marker.scale.y = data.scale_y;
  marker.scale.z = data.scale_z;

  marker.points = data.points;

  switch (data.shape) {
    case ShapeType::POINT:
      marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      marker.pose.orientation.w = 1.0;
      break;
    case ShapeType::LINE:
      marker.type = visualization_msgs::msg::Marker::LINE_LIST;
      marker.pose.orientation.w = 1.0;
      break;
    case ShapeType::LINE_LIST:
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.pose.orientation.w = 1.0;
      break;
    case ShapeType::POLYGON:
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.pose.orientation.w = 1.0;
      break;
    case ShapeType::ARROW:
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.pose.orientation.w = 1.0;
      break;
    case ShapeType::CUBE:
      marker.type = visualization_msgs::msg::Marker::CUBE;
      if (!data.points.empty()) {
        marker.pose.position = data.points[0];
      }
      marker.pose.orientation.w = 1.0;
      break;
    case ShapeType::SPHERE:
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      if (!data.points.empty()) {
        marker.pose.position = data.points[0];
      }
      marker.pose.orientation.w = 1.0;
      break;
    case ShapeType::CYLINDER:
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      if (!data.points.empty()) {
        marker.pose.position = data.points[0];
      }
      marker.pose.orientation.w = 1.0;
      break;
    case ShapeType::TEXT:
      marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      if (!data.points.empty()) {
        marker.pose.position = data.points[0];
      }
      marker.pose.orientation.w = 1.0;
      marker.text = data.text;
      break;
    default:
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.pose.orientation.w = 1.0;
      break;
  }

  return marker;
}

}  // namespace visualization
