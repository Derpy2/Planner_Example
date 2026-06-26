#pragma once

#include <nav_msgs/msg/path.hpp>

namespace reference_line {

class ReferenceLineBase {
public:
    ReferenceLineBase() = default;
    
    virtual ~ReferenceLineBase() {};

    virtual nav_msgs::msg::Path smoothPath(const nav_msgs::msg::Path& path) = 0;
private:
};

} // namespace reference_line