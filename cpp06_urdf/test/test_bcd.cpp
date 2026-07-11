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

std::string getSvgHeader(double view_min_x, double view_min_y,
                         double view_width, double view_height) {
  std::ostringstream ss;
  ss << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
     << "width=\"" << view_width << "\" height=\"" << view_height << "\" "
     << "viewBox=\"" << view_min_x << " " << view_min_y << " " << view_width
     << " " << view_height << "\">\n";
  ss << "<rect width=\"100%\" height=\"100%\" fill=\"#f0f0f0\"/>\n";
  return ss.str();
}

std::string getSvgFooter() { return "</svg>\n"; }

std::string polygonToSvgPath(const Polygon2D& polygon,
                             const std::string& stroke_color,
                             double stroke_width, const std::string& fill_color,
                             double fill_opacity = 0.3, double origin_x = 0.0,
                             double origin_y = 0.0) {
  if (polygon.empty()) return "";

  std::ostringstream ss;
  ss << "<polygon points=\"";
  for (const auto& p : polygon) {
    double svg_x = p.x - origin_x;
    double svg_y = origin_y - p.y;
    ss << svg_x << "," << svg_y << " ";
  }
  ss << "\" stroke=\"" << stroke_color << "\" stroke-width=\"" << stroke_width
     << "\" fill=\"" << fill_color << "\" fill-opacity=\"" << fill_opacity
     << "\"/>\n";
  return ss.str();
}

std::string cellToSvgPath(const Cell& cell, int cell_id, double origin_x = 0.0,
                          double origin_y = 0.0) {
  std::ostringstream ss;
  std::ostringstream fill_color;
  int r = (cell_id * 50 + 100) % 255;
  int g = (cell_id * 80 + 50) % 255;
  int b = (cell_id * 120 + 150) % 255;
  fill_color << "#" << std::hex << (r << 16 | g << 8 | b);

  ss << polygonToSvgPath(cell.polygon, "#333333", 1.0, fill_color.str(), 0.4,
                         origin_x, origin_y);

  double cx = (cell.x_left + cell.x_right) / 2.0 - origin_x;
  double cy = 0;
  if (cell.polygon.size() >= 4) {
    cy = origin_y - (cell.polygon[0].y + cell.polygon[2].y) / 2.0;
  }
  ss << "<text x=\"" << cx << "\" y=\"" << cy << "\" "
     << "font-size=\"8\" text-anchor=\"middle\" fill=\"#000000\">" << cell_id
     << "</text>\n";
  return ss.str();
}

std::string eventPointToSvgPath(const EventPoint& ep, double origin_x = 0.0,
                                double origin_y = 0.0) {
  std::string color;
  double radius = 4.0;
  switch (ep.type) {
    case EventType::SPLIT:
      color = "#ff0000";
      radius = 6.0;
      break;
    case EventType::MERGE:
      color = "#0000ff";
      radius = 6.0;
      break;
    case EventType::NORMAL_UP:
      color = "#00aa00";
      break;
    case EventType::NORMAL_DOWN:
      color = "#888888";
      break;
    default:
      color = "#cccccc";
      break;
  }

  double svg_x = ep.pt.x - origin_x;
  double svg_y = origin_y - ep.pt.y;

  std::ostringstream ss;
  ss << "<circle cx=\"" << svg_x << "\" cy=\"" << svg_y << "\" r=\"" << radius
     << "\" fill=\"" << color << "\" stroke=\"#000000\" "
     << "stroke-width=\"0.5\"/>\n";

  ss << "<text x=\"" << (svg_x + 5) << "\" y=\"" << (svg_y - 5) << "\" "
     << "font-size=\"6\" fill=\"#000000\">";
  switch (ep.type) {
    case EventType::SPLIT:
      ss << "S";
      break;
    case EventType::MERGE:
      ss << "M";
      break;
    case EventType::NORMAL_UP:
      ss << "U";
      break;
    case EventType::NORMAL_DOWN:
      ss << "D";
      break;
    default:
      ss << "N";
      break;
  }
  ss << "</text>\n";
  return ss.str();
}

void saveSvg(const std::string& filename, const Polygon2D& boundary,
             const std::vector<Polygon2D>& obstacles,
             const std::vector<EventPoint>& events,
             const std::vector<Cell>& cells) {
  double min_x = 1e9, max_x = -1e9;
  double min_y = 1e9, max_y = -1e9;

  auto updateBounds = [&](const Node2D& p) {
    min_x = std::min(min_x, p.x);
    max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y);
    max_y = std::max(max_y, p.y);
  };

  for (const auto& p : boundary) updateBounds(p);
  for (const auto& obs : obstacles)
    for (const auto& p : obs) updateBounds(p);

  double origin_x = min_x - 10;
  double origin_y = max_y + 10;
  double width = max_x - min_x + 1000;
  double height = max_y - min_y + 1000;

  std::ofstream ofs(filename);
  ofs << getSvgHeader(origin_x, -origin_y, width, height);

  double legend_x = origin_x + 5;
  double legend_y = -origin_y + 5;
  double legend_w = 130;
  double legend_h = 80;
  ofs << "<!-- Legend -->\n";
  ofs << "<rect x=\"" << legend_x << "\" y=\"" << legend_y << "\" width=\""
      << legend_w << "\" height=\"" << legend_h << "\" fill=\"white\" "
      << "stroke=\"black\"/>\n";
  ofs << "<circle cx=\"" << (legend_x + 15) << "\" cy=\"" << (legend_y + 15)
      << "\" r=\"5\" fill=\"#ff0000\"/><text x=\"" << (legend_x + 25)
      << "\" y=\"" << (legend_y + 18) << "\" font-size=\"8\">SPLIT</text>\n";
  ofs << "<circle cx=\"" << (legend_x + 15) << "\" cy=\"" << (legend_y + 32)
      << "\" r=\"5\" fill=\"#0000ff\"/><text x=\"" << (legend_x + 25)
      << "\" y=\"" << (legend_y + 35) << "\" font-size=\"8\">MERGE</text>\n";
  ofs << "<circle cx=\"" << (legend_x + 15) << "\" cy=\"" << (legend_y + 49)
      << "\" r=\"5\" fill=\"#00aa00\"/><text x=\"" << (legend_x + 25)
      << "\" y=\"" << (legend_y + 52)
      << "\" font-size=\"8\">NORMAL_UP</text>\n";
  ofs << "<circle cx=\"" << (legend_x + 15) << "\" cy=\"" << (legend_y + 66)
      << "\" r=\"5\" fill=\"#888888\"/><text x=\"" << (legend_x + 25)
      << "\" y=\"" << (legend_y + 69)
      << "\" font-size=\"8\">NORMAL_DOWN</text>\n";

  ofs << "<!-- Boundary -->\n";
  ofs << polygonToSvgPath(boundary, "#000000", 2.0, "none", 0.0, origin_x,
                          origin_y);

  ofs << "<!-- Obstacles -->\n";
  for (const auto& obs : obstacles) {
    ofs << polygonToSvgPath(obs, "#666666", 1.5, "#888888", 0.5, origin_x,
                            origin_y);
  }

  ofs << "<!-- Cells -->\n";
  for (size_t i = 0; i < cells.size(); ++i) {
    ofs << "<!-- Cell " << i << " x:[" << cells[i].x_left << ", "
        << cells[i].x_right << "] -->\n";
    ofs << cellToSvgPath(cells[i], i, origin_x, origin_y);
  }

  ofs << "<!-- Event Points -->\n";
  for (const auto& ep : events) {
    ofs << eventPointToSvgPath(ep, origin_x, origin_y);
  }

  ofs << getSvgFooter();
  ofs.close();
  std::cout << "SVG saved to " << filename << std::endl;
}

}  // namespace

class BCDTest : public ::testing::Test {
 protected:
  void SetUp() override {}
  void TearDown() override {}
};

TEST_F(BCDTest, SimpleSquareBoundary) {
  Polygon2D boundary = {Node2D(0.0, 0.0), Node2D(100.0, 0.0),
                        Node2D(100.0, 100.0), Node2D(0.0, 100.0)};
  std::vector<Polygon2D> obstacles;

  BCDDecomposer decomposer;
  std::vector<Cell> cells = decomposer.decompose(boundary, obstacles);
  std::vector<EventPoint> events =
      decomposer.generateEvents(boundary, obstacles);

  saveSvg("/tmp/bcd_test_simple.svg", boundary, obstacles, events, cells);

  EXPECT_FALSE(events.empty());
  EXPECT_EQ(events.size(), 4u);

  int split_count = 0, merge_count = 0, normal_count = 0;
  for (const auto& e : events) {
    if (e.type == EventType::SPLIT)
      split_count++;
    else if (e.type == EventType::MERGE)
      merge_count++;
    else
      normal_count++;
  }
  EXPECT_EQ(split_count + merge_count + normal_count, events.size());
}

TEST_F(BCDTest, SquareWithCentralObstacle) {
  Polygon2D boundary = {Node2D(0.0, 0.0), Node2D(100.0, 0.0),
                        Node2D(100.0, 100.0), Node2D(0.0, 100.0)};

  Polygon2D obstacle = {Node2D(40.0, 40.0), Node2D(60.0, 40.0),
                        Node2D(60.0, 60.0), Node2D(40.0, 60.0)};
  std::vector<Polygon2D> obstacles = {obstacle};

  BCDDecomposer decomposer;
  std::vector<Cell> cells = decomposer.decompose(boundary, obstacles);
  std::vector<EventPoint> events =
      decomposer.generateEvents(boundary, obstacles);

  saveSvg("/tmp/bcd_test_with_obstacle.svg", boundary, obstacles, events,
          cells);

  EXPECT_FALSE(events.empty());
  EXPECT_EQ(events.size(), 8u);
}

TEST_F(BCDTest, ComplexLShape) {
  Polygon2D boundary = {Node2D(0.0, 0.0),    Node2D(100.0, 0.0),
                        Node2D(100.0, 40.0), Node2D(40.0, 40.0),
                        Node2D(40.0, 100.0), Node2D(0.0, 100.0)};

  Polygon2D obstacle1 = {Node2D(10.0, 10.0), Node2D(30.0, 10.0),
                         Node2D(30.0, 30.0), Node2D(10.0, 30.0)};
  Polygon2D obstacle2 = {Node2D(60.0, 50.0), Node2D(90.0, 50.0),
                         Node2D(90.0, 70.0), Node2D(60.0, 70.0)};

  std::vector<Polygon2D> obstacles = {obstacle1, obstacle2};

  BCDDecomposer decomposer;
  std::vector<Cell> cells = decomposer.decompose(boundary, obstacles);
  std::vector<EventPoint> events =
      decomposer.generateEvents(boundary, obstacles);

  saveSvg("/tmp/bcd_test_lshape.svg", boundary, obstacles, events, cells);

  EXPECT_EQ(events.size(), 14u);
}

TEST_F(BCDTest, ClassifyVertexTriangle) {
  Polygon2D triangle = {Node2D(0.0, 0.0), Node2D(50.0, 0.0),
                        Node2D(25.0, 50.0)};

  BCDDecomposer decomposer;

  EventType t0 = decomposer.classifyVertex(triangle[0], triangle);
  EventType t1 = decomposer.classifyVertex(triangle[1], triangle);
  EventType t2 = decomposer.classifyVertex(triangle[2], triangle);

  EXPECT_EQ(t0, EventType::NORMAL_UP);
  EXPECT_EQ(t1, EventType::SPLIT);
  EXPECT_EQ(t2, EventType::NORMAL_DOWN);
}

TEST_F(BCDTest, ClassifyVertexSquare) {
  Polygon2D square = {Node2D(0.0, 0.0), Node2D(50.0, 0.0), Node2D(50.0, 50.0),
                      Node2D(0.0, 50.0)};

  BCDDecomposer decomposer;

  for (int i = 0; i < 4; ++i) {
    EventType t = decomposer.classifyVertex(square[i], square);
    EXPECT_NE(t, EventType::NONE);
  }
}

TEST_F(BCDTest, GenerateSnakePath) {
  Polygon2D boundary = {Node2D(0.0, 0.0), Node2D(100.0, 0.0),
                        Node2D(100.0, 60.0), Node2D(0.0, 60.0)};
  std::vector<Polygon2D> obstacles;

  BCDDecomposer decomposer;
  std::vector<Cell> cells = decomposer.decompose(boundary, obstacles);
  std::vector<EventPoint> events =
      decomposer.generateEvents(boundary, obstacles);

  saveSvg("/tmp/bcd_test_snake.svg", boundary, obstacles, events, cells);

  if (!cells.empty()) {
    std::vector<Node2D> path = decomposer.generateSnakePath(cells[0], 2.0);
    EXPECT_GE(path.size(), 4u);
    EXPECT_EQ(path.front().x, 0.0);
    EXPECT_EQ(path.front().y, 0.0);
  } else {
    Cell dummy_cell;
    dummy_cell.x_left = 0.0;
    dummy_cell.x_right = 10.0;
    dummy_cell.polygon = {Node2D(0.0, 0.0), Node2D(100.0, 0.0),
                          Node2D(100.0, 60.0), Node2D(0.0, 60.0)};
    std::vector<Node2D> path = decomposer.generateSnakePath(dummy_cell, 2.0);
    EXPECT_GE(path.size(), 4u);
  }
}

TEST_F(BCDTest, MultipleObstacles) {
  Polygon2D boundary = {Node2D(0.0, 0.0), Node2D(200.0, 0.0),
                        Node2D(200.0, 150.0), Node2D(0.0, 150.0)};

  std::vector<Polygon2D> obstacles;
  obstacles.push_back({Node2D(30.0, 30.0), Node2D(70.0, 30.0),
                       Node2D(70.0, 70.0), Node2D(30.0, 70.0)});
  obstacles.push_back({Node2D(100.0, 20.0), Node2D(140.0, 20.0),
                       Node2D(140.0, 60.0), Node2D(100.0, 60.0)});
  obstacles.push_back({Node2D(50.0, 90.0), Node2D(90.0, 90.0),
                       Node2D(90.0, 130.0), Node2D(50.0, 130.0)});

  BCDDecomposer decomposer;
  std::vector<Cell> cells = decomposer.decompose(boundary, obstacles);
  std::vector<EventPoint> events =
      decomposer.generateEvents(boundary, obstacles);

  saveSvg("/tmp/bcd_test_multi_obstacle.svg", boundary, obstacles, events,
          cells);

  EXPECT_EQ(events.size(), 16u);
}

TEST_F(BCDTest, EventGeneration) {
  Polygon2D boundary = {Node2D(0.0, 0.0), Node2D(80.0, 0.0), Node2D(80.0, 80.0),
                        Node2D(0.0, 80.0)};
  Polygon2D obstacle = {Node2D(30.0, 30.0), Node2D(50.0, 30.0),
                        Node2D(50.0, 50.0), Node2D(30.0, 50.0)};

  BCDDecomposer decomposer;
  std::vector<EventPoint> events =
      decomposer.generateEvents(boundary, {obstacle});

  int split_count = 0, merge_count = 0, normal_up_count = 0,
      normal_down_count = 0;
  for (const auto& e : events) {
    if (e.type == EventType::SPLIT)
      split_count++;
    else if (e.type == EventType::MERGE)
      merge_count++;
    else if (e.type == EventType::NORMAL_UP)
      normal_up_count++;
    else if (e.type == EventType::NORMAL_DOWN)
      normal_down_count++;
  }

  EXPECT_EQ(split_count, 4);
  EXPECT_EQ(merge_count, 0);
  EXPECT_EQ(events.size(), 8u);
}
