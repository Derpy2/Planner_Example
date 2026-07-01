#pragma once

#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace visualization {

enum class ShapeType {
  POINT,
  LINE,
  LINE_LIST,
  POLYGON,
  ARROW,
  CUBE,
  SPHERE,
  CYLINDER,
  TEXT
};

struct Color {
  float r, g, b, a;
  Color(float r_ = 1.0f, float g_ = 0.0f, float b_ = 0.0f, float a_ = 1.0f)
      : r(r_), g(g_), b(b_), a(a_) {}
};

struct VisualData {
  std::string ns;                                 // namespace
  int id = 0;                                     // marker id
  ShapeType shape;                                // 形状类型
  std::vector<geometry_msgs::msg::Point> points;  // 几何点
  Color color{1.0f, 0.0f, 0.0f, 1.0f};            // 颜色
  float scale_x = 0.1f;                           // x方向尺寸
  float scale_y = 0.1f;                           // y方向尺寸
  float scale_z = 0.1f;                           // z方向尺寸
  std::string text;                               // 文本内容（仅TEXT类型使用）
  std::string frame_id = "map";                   // 参考坐标系
};

/**
 * @brief 可视化数据管理单例类
 *
 * 支持添加、删除、更新各种可视化Marker数据。
 * 线程安全，可在多线程环境中使用。
 */
class VisualizationManager {
 public:
  /**
   * @brief 获取单例实例
   */
  static VisualizationManager& Instance();

  VisualizationManager(const VisualizationManager&) = delete;
  VisualizationManager& operator=(const VisualizationManager&) = delete;

  /**
   * @brief 添加或更新一个Marker
   * @param data 可视化数据
   */
  void AddMarker(const VisualData& data);

  /**
   * @brief 删除指定Marker
   * @param ns namespace
   * @param id marker id
   */
  void RemoveMarker(const std::string& ns, int id);

  /**
   * @brief 清空所有Marker
   */
  void ClearAll();

  /**
   * @brief 清空指定namespace下的所有Marker
   * @param ns namespace
   */
  void ClearNamespace(const std::string& ns);

  /**
   * @brief 获取当前所有Marker的MarkerArray
   */
  visualization_msgs::msg::MarkerArray GetAllMarkers();

  // ========== 便捷API ==========

  /**
   * @brief 添加单个点（球体）
   */
  void AddPoint(const std::string& ns, int id,
                const geometry_msgs::msg::Point& pt,
                const Color& color = Color(1.0f, 0.0f, 0.0f),
                float scale = 0.1f, const std::string& frame_id = "map");

  /**
   * @brief 添加一条线段（起点到终点）
   */
  void AddLine(const std::string& ns, int id,
               const geometry_msgs::msg::Point& start,
               const geometry_msgs::msg::Point& end,
               const Color& color = Color(1.0f, 0.0f, 0.0f),
               float scale = 0.05f, const std::string& frame_id = "map");

  /**
   * @brief 添加多段线（连续线段）
   */
  void AddLineStrip(const std::string& ns, int id,
                    const std::vector<geometry_msgs::msg::Point>& points,
                    const Color& color = Color(1.0f, 0.0f, 0.0f),
                    float scale = 0.05f, const std::string& frame_id = "map");

  /**
   * @brief 添加多边形（闭合线段）
   */
  void AddPolygon(const std::string& ns, int id,
                  const std::vector<geometry_msgs::msg::Point>& points,
                  const Color& color = Color(1.0f, 0.0f, 0.0f),
                  float scale = 0.05f, const std::string& frame_id = "map");

  /**
   * @brief 添加箭头
   */
  void AddArrow(const std::string& ns, int id,
                const geometry_msgs::msg::Point& start,
                const geometry_msgs::msg::Point& end,
                const Color& color = Color(1.0f, 0.0f, 0.0f),
                float scale = 0.1f, const std::string& frame_id = "map");

  /**
   * @brief 添加立方体
   */
  void AddCube(const std::string& ns, int id,
               const geometry_msgs::msg::Point& center,
               const geometry_msgs::msg::Point& size,
               const Color& color = Color(1.0f, 0.0f, 0.0f),
               const std::string& frame_id = "map");

  /**
   * @brief 添加球体
   */
  void AddSphere(const std::string& ns, int id,
                 const geometry_msgs::msg::Point& center, float radius,
                 const Color& color = Color(1.0f, 0.0f, 0.0f),
                 const std::string& frame_id = "map");

  /**
   * @brief 添加圆柱体
   */
  void AddCylinder(const std::string& ns, int id,
                   const geometry_msgs::msg::Point& center, float radius,
                   float height, const Color& color = Color(1.0f, 0.0f, 0.0f),
                   const std::string& frame_id = "map");

  /**
   * @brief 添加文本
   */
  void AddText(const std::string& ns, int id,
               const geometry_msgs::msg::Point& position,
               const std::string& text,
               const Color& color = Color(1.0f, 1.0f, 1.0f), float scale = 0.2f,
               const std::string& frame_id = "map");

 private:
  VisualizationManager() = default;

  visualization_msgs::msg::Marker ConvertToMarker(const VisualData& data);

  std::mutex mutex_;
  // namespace -> (id -> VisualData)
  std::unordered_map<std::string, std::unordered_map<int, VisualData>> markers_;
};

}  // namespace visualization
