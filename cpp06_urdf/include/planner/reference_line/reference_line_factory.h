#pragma once

#include <string>
#include <memory>

#include "reference_line_base.h"
#include "slide_window.h"
#include "b_spline.h"

namespace reference_line {

enum ReferenceLineType {
    BSpline = 0,
    SlideWindow = 1
};

class ReferenceLineFactory {
public:
    static std::unique_ptr<ReferenceLineBase> GetReferenceLineCreator(const ReferenceLineType& type)
    {
        switch (type) {
            case ReferenceLineType::BSpline:
                return std::make_unique<BSplineReferenceLine>();
            case ReferenceLineType::SlideWindow:
                return std::make_unique<SlideWindowReferenceLine>();
            default:
                throw std::invalid_argument("Unknown reference line type");
        }
    }
};

} // namespace reference_line