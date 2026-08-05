#include <gtest/gtest.h>

#include <fstream>
#include <iostream>
#include <sstream>

#include "common/bcd_data.h"
#include "common/node2d.h"
#include "planner/global_planner/complete_cover_path/bcd.h"

using namespace global_planner;
using namespace global_planner::complete_cover_path;
using namespace common;

namespace {

// std::string getSvgHeader(double view_min_x, double view_min_y,
//                          double view_width, double view_height) {
//   std::ostringstream ss;
//   ss << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
//      << "width=\"" << view_width << "\" height=\"" << view_height << "\" "
//      << "viewBox=\"" << view_min_x << " " << view_min_y << " " << view_width
//      << " " << view_height << "\">\n";
//   ss << "<rect width=\"100%\" height=\"100%\" fill=\"#f0f0f0\"/>\n";
//   return ss.str();
// }

// std::string getSvgFooter() { return "</svg>\n"; }

// std::string polygonToSvgPath(const Polygon2D& polygon,
//                              const std::string& stroke_color,
//                              double stroke_width, const std::string&
//                              fill_color, double fill_opacity = 0.3, double
//                              origin_x = 0.0, double origin_y = 0.0) {
//   if (polygon.empty()) return "";

//   std::ostringstream ss;
//   ss << "<polygon points=\"";
//   for (const auto& p : polygon) {
//     double svg_x = p.x - origin_x;
//     double svg_y = origin_y - p.y;
//     ss << svg_x << "," << svg_y << " ";
//   }
//   ss << "\" stroke=\"" << stroke_color << "\" stroke-width=\"" <<
//   stroke_width
//      << "\" fill=\"" << fill_color << "\" fill-opacity=\"" << fill_opacity
//      << "\"/>\n";
//   return ss.str();
// }

// std::string cellToSvgPath(const Cell& cell, int cell_id, double origin_x =
// 0.0,
//                           double origin_y = 0.0) {
//   std::ostringstream ss;
//   std::ostringstream fill_color;
//   int r = (cell_id * 50 + 100) % 255;
//   int g = (cell_id * 80 + 50) % 255;
//   int b = (cell_id * 120 + 150) % 255;
//   fill_color << "#" << std::hex << (r << 16 | g << 8 | b);

//   ss << polygonToSvgPath(cell.polygon, "#333333", 1.0, fill_color.str(), 0.4,
//                          origin_x, origin_y);

//   double cx = (cell.x_left + cell.x_right) / 2.0 - origin_x;
//   double cy = 0;
//   if (cell.polygon.size() >= 4) {
//     cy = origin_y - (cell.polygon[0].y + cell.polygon[2].y) / 2.0;
//   }
//   ss << "<text x=\"" << cx << "\" y=\"" << cy << "\" "
//      << "font-size=\"8\" text-anchor=\"middle\" fill=\"#000000\">" << cell_id
//      << "</text>\n";
//   return ss.str();
// }

// std::string eventPointToSvgPath(const EventPoint& ep, double origin_x = 0.0,
//                                 double origin_y = 0.0) {
//   std::string color;
//   double radius = 4.0;
//   switch (ep.type) {
//     case EventType::SPLIT:
//       color = "#ff0000";
//       radius = 6.0;
//       break;
//     case EventType::MERGE:
//       color = "#0000ff";
//       radius = 6.0;
//       break;
//     case EventType::NORMAL_UP:
//       color = "#00aa00";
//       break;
//     case EventType::NORMAL_DOWN:
//       color = "#888888";
//       break;
//     default:
//       color = "#cccccc";
//       break;
//   }

//   double svg_x = ep.pt.x - origin_x;
//   double svg_y = origin_y - ep.pt.y;

//   std::ostringstream ss;
//   ss << "<circle cx=\"" << svg_x << "\" cy=\"" << svg_y << "\" r=\"" <<
//   radius
//      << "\" fill=\"" << color << "\" stroke=\"#000000\" "
//      << "stroke-width=\"0.5\"/>\n";

//   ss << "<text x=\"" << (svg_x + 5) << "\" y=\"" << (svg_y - 5) << "\" "
//      << "font-size=\"6\" fill=\"#000000\">";
//   switch (ep.type) {
//     case EventType::SPLIT:
//       ss << "S";
//       break;
//     case EventType::MERGE:
//       ss << "M";
//       break;
//     case EventType::NORMAL_UP:
//       ss << "U";
//       break;
//     case EventType::NORMAL_DOWN:
//       ss << "D";
//       break;
//     default:
//       ss << "N";
//       break;
//   }
//   ss << "</text>\n";
//   return ss.str();
// }

// void saveSvg(const std::string& filename, const Polygon2D& boundary,
//              const std::vector<Polygon2D>& obstacles,
//              const std::vector<EventPoint>& events,
//              const std::vector<Cell>& cells) {
//   double min_x = 1e9, max_x = -1e9;
//   double min_y = 1e9, max_y = -1e9;

//   auto updateBounds = [&](const Node2D& p) {
//     min_x = std::min(min_x, p.x);
//     max_x = std::max(max_x, p.x);
//     min_y = std::min(min_y, p.y);
//     max_y = std::max(max_y, p.y);
//   };

//   for (const auto& p : boundary) updateBounds(p);
//   for (const auto& obs : obstacles)
//     for (const auto& p : obs) updateBounds(p);

//   double origin_x = min_x - 10;
//   double origin_y = max_y + 10;
//   double width = max_x - min_x + 1000;
//   double height = max_y - min_y + 1000;

//   std::ofstream ofs(filename);
//   ofs << getSvgHeader(origin_x, -origin_y, width, height);

//   double legend_x = origin_x + 5;
//   double legend_y = -origin_y + 5;
//   double legend_w = 130;
//   double legend_h = 80;
//   ofs << "<!-- Legend -->\n";
//   ofs << "<rect x=\"" << legend_x << "\" y=\"" << legend_y << "\" width=\""
//       << legend_w << "\" height=\"" << legend_h << "\" fill=\"white\" "
//       << "stroke=\"black\"/>\n";
//   ofs << "<circle cx=\"" << (legend_x + 15) << "\" cy=\"" << (legend_y + 15)
//       << "\" r=\"5\" fill=\"#ff0000\"/><text x=\"" << (legend_x + 25)
//       << "\" y=\"" << (legend_y + 18) << "\" font-size=\"8\">SPLIT</text>\n";
//   ofs << "<circle cx=\"" << (legend_x + 15) << "\" cy=\"" << (legend_y + 32)
//       << "\" r=\"5\" fill=\"#0000ff\"/><text x=\"" << (legend_x + 25)
//       << "\" y=\"" << (legend_y + 35) << "\" font-size=\"8\">MERGE</text>\n";
//   ofs << "<circle cx=\"" << (legend_x + 15) << "\" cy=\"" << (legend_y + 49)
//       << "\" r=\"5\" fill=\"#00aa00\"/><text x=\"" << (legend_x + 25)
//       << "\" y=\"" << (legend_y + 52)
//       << "\" font-size=\"8\">NORMAL_UP</text>\n";
//   ofs << "<circle cx=\"" << (legend_x + 15) << "\" cy=\"" << (legend_y + 66)
//       << "\" r=\"5\" fill=\"#888888\"/><text x=\"" << (legend_x + 25)
//       << "\" y=\"" << (legend_y + 69)
//       << "\" font-size=\"8\">NORMAL_DOWN</text>\n";

//   ofs << "<!-- Boundary -->\n";
//   ofs << polygonToSvgPath(boundary, "#000000", 2.0, "none", 0.0, origin_x,
//                           origin_y);

//   ofs << "<!-- Obstacles -->\n";
//   for (const auto& obs : obstacles) {
//     ofs << polygonToSvgPath(obs, "#666666", 1.5, "#888888", 0.5, origin_x,
//                             origin_y);
//   }

//   ofs << "<!-- Cells -->\n";
//   for (size_t i = 0; i < cells.size(); ++i) {
//     ofs << "<!-- Cell " << i << " x:[" << cells[i].x_left << ", "
//         << cells[i].x_right << "] -->\n";
//     ofs << cellToSvgPath(cells[i], i, origin_x, origin_y);
//   }

//   ofs << "<!-- Event Points -->\n";
//   for (const auto& ep : events) {
//     ofs << eventPointToSvgPath(ep, origin_x, origin_y);
//   }

//   ofs << getSvgFooter();
//   ofs.close();
//   std::cout << "SVG saved to " << filename << std::endl;
// }

}  // namespace

// 计算多边形有向面积（逆时针为正）
double polygonArea(const Polygon2D& poly) {
  double area = 0.0;
  int n = static_cast<int>(poly.size());
  for (int i = 0; i < n; ++i) {
    const Node2D& p0 = poly[i];
    const Node2D& p1 = poly[(i + 1) % n];
    area += p0.x * p1.y - p1.x * p0.y;
  }
  return std::fabs(area) * 0.5;
}

class BCDTest : public ::testing::Test {
 protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(BCDTest, SimpleSquareBoundary) {
  Polygon2D boundary = {Node2D(0.0, 0.0), Node2D(100.0, 0.0),
                        Node2D(100.0, 100.0), Node2D(0.0, 100.0)};

  Polygon2D obstacle = {Node2D(40.0, 40.0), Node2D(60.0, 40.0),
                        Node2D(60.0, 60.0), Node2D(40.0, 60.0)};
  std::vector<Polygon2D> obstacles;
  obstacles.emplace_back(obstacle);

  BCDDecomposer decomposer;
  boundary = decomposer.preProcessPolygon(boundary);
  std::vector<Polygon2D> cells = decomposer.decompose(boundary, obstacles);

  EXPECT_FALSE(cells.empty());
  EXPECT_EQ(cells.size(), 4u);

  for (const Polygon2D& poly : cells) {
    std::cout << "\n";
    for (const Node2D& pt : poly) {
      std::cout << "x: " << pt.x << " y: " << pt.y << std::endl;
    }
  }
}

TEST_F(BCDTest, DecomposeAlongXAxisMatchesDefault) {
  Polygon2D boundary = {Node2D(0.0, 0.0), Node2D(100.0, 0.0),
                        Node2D(100.0, 100.0), Node2D(0.0, 100.0)};
  Polygon2D obstacle = {Node2D(40.0, 40.0), Node2D(60.0, 40.0),
                        Node2D(60.0, 60.0), Node2D(40.0, 60.0)};
  std::vector<Polygon2D> obstacles = {obstacle};

  BCDDecomposer decomposer;
  std::vector<Polygon2D> default_cells =
      decomposer.decompose(boundary, obstacles);
  std::vector<Polygon2D> x_dir_cells =
      decomposer.decompose(boundary, obstacles, Node2D(1.0, 0.0));

  EXPECT_EQ(default_cells.size(), x_dir_cells.size());

  // 按面积排序后比较，避免顺序不同
  std::vector<double> default_areas;
  std::vector<double> x_dir_areas;
  for (const auto& cell : default_cells) {
    default_areas.push_back(polygonArea(cell));
  }
  for (const auto& cell : x_dir_cells) {
    x_dir_areas.push_back(polygonArea(cell));
  }
  std::sort(default_areas.begin(), default_areas.end());
  std::sort(x_dir_areas.begin(), x_dir_areas.end());

  for (size_t i = 0; i < default_areas.size(); ++i) {
    EXPECT_NEAR(default_areas[i], x_dir_areas[i], 1e-6);
  }
}

TEST_F(BCDTest, DecomposeAlongYAxis) {
  Polygon2D boundary = {Node2D(0.0, 0.0), Node2D(100.0, 0.0),
                        Node2D(100.0, 100.0), Node2D(0.0, 100.0)};
  Polygon2D obstacle = {Node2D(40.0, 40.0), Node2D(60.0, 40.0),
                        Node2D(60.0, 60.0), Node2D(40.0, 60.0)};
  std::vector<Polygon2D> obstacles = {obstacle};

  BCDDecomposer decomposer;
  std::vector<Polygon2D> cells =
      decomposer.decompose(boundary, obstacles, Node2D(0.0, 1.0));

  EXPECT_EQ(cells.size(), 4u);

  double total_area = 0.0;
  for (const auto& cell : cells) {
    EXPECT_GE(cell.size(), 3u);
    total_area += polygonArea(cell);
  }
  EXPECT_NEAR(total_area, 100.0 * 100.0 - 20.0 * 20.0, 1e-3);
}

TEST_F(BCDTest, DecomposeAlong45Degree) {
  Polygon2D boundary = {Node2D(0.0, 0.0), Node2D(100.0, 0.0),
                        Node2D(100.0, 100.0), Node2D(0.0, 100.0)};
  Polygon2D obstacle = {Node2D(40.0, 40.0), Node2D(60.0, 40.0),
                        Node2D(60.0, 60.0), Node2D(40.0, 60.0)};
  std::vector<Polygon2D> obstacles = {obstacle};

  BCDDecomposer decomposer;
  std::vector<Polygon2D> cells =
      decomposer.decompose(boundary, obstacles, Node2D(1.0, 1.0));

  EXPECT_FALSE(cells.empty());

  double total_area = 0.0;
  for (const auto& cell : cells) {
    EXPECT_GE(cell.size(), 3u);
    total_area += polygonArea(cell);
  }
  EXPECT_NEAR(total_area, 100.0 * 100.0 - 20.0 * 20.0, 1e-3);
}

TEST_F(BCDTest, DecomposeWithZeroDirReturnsEmpty) {
  Polygon2D boundary = {Node2D(0.0, 0.0), Node2D(100.0, 0.0),
                        Node2D(100.0, 100.0), Node2D(0.0, 100.0)};

  BCDDecomposer decomposer;
  std::vector<Polygon2D> cells =
      decomposer.decompose(boundary, {}, Node2D(0.0, 0.0));

  EXPECT_TRUE(cells.empty());
}

TEST_F(BCDTest, SquareWithCentralObstacle) {
  Polygon2D boundary = {Node2D(0.0, 0.0), Node2D(100.0, 0.0),
                        Node2D(100.0, 100.0), Node2D(0.0, 100.0)};

  Polygon2D obstacle = {Node2D(40.0, 40.0), Node2D(60.0, 40.0),
                        Node2D(60.0, 60.0), Node2D(40.0, 60.0),
                        Node2D(50.0, 50.0)};
  std::vector<Polygon2D> obstacles = {obstacle};

  BCDDecomposer decomposer;

  boundary = decomposer.preProcessPolygon(boundary);
  std::vector<Polygon2D> cells = decomposer.decompose(boundary, obstacles);

  EXPECT_FALSE(cells.empty());
  // EXPECT_EQ(cells.size(), 8u);

  for (const Polygon2D& poly : cells) {
    std::cout << "\n";
    for (const Node2D& pt : poly) {
      std::cout << "x: " << pt.x << " y: " << pt.y << std::endl;
    }
  }
}
