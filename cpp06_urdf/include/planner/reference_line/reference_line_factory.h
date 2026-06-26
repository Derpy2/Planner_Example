#pragma once

#include <memory>
#include <string>

#include "b_spline.h"
#include "bezier.h"
#include "reference_line_base.h"
#include "slide_window.h"

namespace reference_line {

enum ReferenceLineType { BSpline = 0, SlideWindow = 1, Bezier = 2 };

class ReferenceLineFactory {
 public:
  static std::unique_ptr<ReferenceLineBase> GetReferenceLineCreator(
      const ReferenceLineType& type) {
    switch (type) {
      case ReferenceLineType::BSpline:
        return std::make_unique<BSplineReferenceLine>();
      case ReferenceLineType::SlideWindow:
        return std::make_unique<SlideWindowReferenceLine>();
      case ReferenceLineType::Bezier:
        return std::make_unique<BezierReferenceLine>();
      default:
        throw std::invalid_argument("Unknown reference line type");
    }
  }
};

}  // namespace reference_line