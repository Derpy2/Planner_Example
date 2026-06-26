#pragma once

#include "reference_line_base.h"

namespace reference_line {

class SlideWindowReferenceLine : public ReferenceLineBase {
public:
    SlideWindowReferenceLine() {};

    nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path& path) override;
private:

};

} // namespace reference_line