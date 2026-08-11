#pragma once

#include <g2o/stuff/misc.h>
#include <tf2/convert.h>
#include <tf2/utils.h>

#include <Eigen/Core>
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace teb_local_planner {

class PoseSE2 {
 public:
  /**
   * @brief Default constructor
   */
  PoseSE2() { setZero(); }

  /**
   * @brief Construct pose given a position vector and an angle theta
   * @param position 2D position vector
   * @param theta angle given in rad
   */
  PoseSE2(const Eigen::Ref<const Eigen::Vector2d>& position, double theta) {
    position_ = position;
    theta_ = theta;
  }

  /**
   * @brief Construct pose using single components x, y, and the yaw angle
   * @param x x-coordinate
   * @param y y-coordinate
   * @param theta yaw angle in rad
   */
  PoseSE2(double x, double y, double theta) {
    position_.coeffRef(0) = x;
    position_.coeffRef(1) = y;
    theta_ = theta;
  }

  /**
   * @brief Construct pose using a geometry_msgs::msg::Pose
   * @param pose geometry_msgs::msg::Pose object
   */
  PoseSE2(const geometry_msgs::msg::PoseStamped& pose) : PoseSE2(pose.pose) {}

  /**
   * @brief Construct pose using a geometry_msgs::msg::Pose
   * @param pose geometry_msgs::msg::Pose object
   */
  PoseSE2(const geometry_msgs::msg::Pose& pose) {
    position_.coeffRef(0) = pose.position.x;
    position_.coeffRef(1) = pose.position.y;
    theta_ = tf2::getYaw(pose.orientation);
  }

  /**
   * @brief Construct pose using a geometry_msgs::msg::Pose2D
   * @param pose geometry_msgs::msg::Pose2D object
   */
  PoseSE2(const geometry_msgs::msg::Pose2D& pose) {
    position_.coeffRef(0) = pose.x;
    position_.coeffRef(1) = pose.y;
    theta_ = pose.theta;
  }

  /**
   * @brief Copy constructor
   * @param pose PoseSE2 instance
   */
  PoseSE2(const PoseSE2& pose) {
    position_ = pose.position_;
    theta_ = pose.theta_;
  }

  ///@}

  /**
   * @brief Destructs the PoseSE2
   */
  ~PoseSE2() {}

  /** @name Access and modify values */
  ///@{

  /**
   * @brief Access the 2D position part
   * @see estimate
   * @return reference to the 2D position part
   */
  Eigen::Vector2d& position() { return position_; }

  /**
   * @brief Access the 2D position part (read-only)
   * @see estimate
   * @return const reference to the 2D position part
   */
  const Eigen::Vector2d& position() const { return position_; }

  /**
   * @brief Access the x-coordinate the pose
   * @return reference to the x-coordinate
   */
  double& x() { return position_.coeffRef(0); }

  /**
   * @brief Access the x-coordinate the pose (read-only)
   * @return const reference to the x-coordinate
   */
  const double& x() const { return position_.coeffRef(0); }

  /**
   * @brief Access the y-coordinate the pose
   * @return reference to the y-coordinate
   */
  double& y() { return position_.coeffRef(1); }

  /**
   * @brief Access the y-coordinate the pose (read-only)
   * @return const reference to the y-coordinate
   */
  const double& y() const { return position_.coeffRef(1); }

  /**
   * @brief Access the orientation part (yaw angle) of the pose
   * @return reference to the yaw angle
   */
  double& theta() { return theta_; }

  /**
   * @brief Access the orientation part (yaw angle) of the pose (read-only)
   * @return const reference to the yaw angle
   */
  const double& theta() const { return theta_; }

  /**
   * @brief Set pose to [0,0,0]
   */
  void setZero() {
    position_.setZero();
    theta_ = 0;
  }

  /**
   * @brief Convert PoseSE2 to a geometry_msgs::msg::Pose
   * @param[out] pose Pose message
   */
  void toPoseMsg(geometry_msgs::msg::Pose& pose) const {
    pose.position.x = position_.x();
    pose.position.y = position_.y();
    pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    pose.orientation = tf2::toMsg(q);
  }

  /**
   * @brief Convert PoseSE2 to a geometry_msgs::msg::Pose2D
   * @param[out] pose Pose message
   */
  void toPoseMsg(geometry_msgs::msg::Pose2D& pose) const {
    pose.x = position_.x();
    pose.y = position_.y();
    pose.theta = theta_;
  }

  /**
   * @brief Return the unit vector of the current orientation
   * @returns [cos(theta), sin(theta))]^T
   */
  Eigen::Vector2d orientationUnitVec() const {
    return Eigen::Vector2d(std::cos(theta_), std::sin(theta_));
  }

  ///@}

  /** @name Arithmetic operations for which operators are not always reasonable
   */
  ///@{

  /**
   * @brief Scale all SE2 components (x,y,theta) and normalize theta afterwards
   * to [-pi, pi]
   * @param factor scale factor
   */
  void scale(double factor) {
    position_ *= factor;
    theta_ = g2o::normalize_theta(theta_ * factor);
  }

  /**
   * @brief Increment the pose by adding a double[3] array
   * The angle is normalized afterwards
   * @param pose_as_array 3D double array [x, y, theta]
   */
  void plus(const double* pose_as_array) {
    position_.coeffRef(0) += pose_as_array[0];
    position_.coeffRef(1) += pose_as_array[1];
    theta_ = g2o::normalize_theta(theta_ + pose_as_array[2]);
  }

  /**
   * @brief Get the mean / average of two poses and store it in the caller class
   * For the position part: 0.5*(x1+x2)
   * For the angle: take the angle of the mean direction vector
   * @param pose1 first pose to consider
   * @param pose2 second pose to consider
   */
  void averageInPlace(const PoseSE2& pose1, const PoseSE2& pose2) {
    position_ = (pose1.position_ + pose2.position_) / 2;
    theta_ = std::atan2(std::sin(pose1.theta_) + std::sin(pose2.theta_),
                        std::cos(pose1.theta_) + std::cos(pose2.theta_));
  }

  /**
   * @brief Get the mean / average of two poses and return the result (static)
   * For the position part: 0.5*(x1+x2)
   * For the angle: take the angle of the mean direction vector
   * @param pose1 first pose to consider
   * @param pose2 second pose to consider
   * @return mean / average of \c pose1 and \c pose2
   */
  static PoseSE2 average(const PoseSE2& pose1, const PoseSE2& pose2) {
    double avetheta =
        std::atan2(std::sin(pose1.theta_) + std::sin(pose2.theta_),
                   std::cos(pose1.theta_) + std::cos(pose2.theta_));
    return PoseSE2((pose1.position_ + pose2.position_) / 2, avetheta);
  }

  /**
   * @brief Rotate pose globally
   *
   * Compute [pose_x, pose_y] = Rot(\c angle) * [pose_x, pose_y].
   * if \c adjusttheta_, posetheta_ is also rotated by \c angle
   * @param angle the angle defining the 2d rotation
   * @param adjusttheta_ if \c true, the orientation theta is also rotated
   */
  void rotateGlobal(double angle, bool adjusttheta_ = true) {
    double new_x =
        std::cos(angle) * position_.x() - std::sin(angle) * position_.y();
    double new_y =
        std::sin(angle) * position_.x() + std::cos(angle) * position_.y();
    position_.x() = new_x;
    position_.y() = new_y;
    if (adjusttheta_) theta_ = g2o::normalize_theta(theta_ + angle);
  }

  ///@}

  /** @name Operator overloads / Allow some arithmetic operations */
  ///@{

  /**
   * @brief Asignment operator
   * @param rhs PoseSE2 instance
   * @todo exception safe version of the assignment operator
   */
  PoseSE2& operator=(const PoseSE2& rhs) {
    if (&rhs != this) {
      position_ = rhs.position_;
      theta_ = rhs.theta_;
    }
    return *this;
  }

  /**
   * @brief Compound assignment operator (addition)
   * @param rhs addend
   */
  PoseSE2& operator+=(const PoseSE2& rhs) {
    position_ += rhs.position_;
    theta_ = g2o::normalize_theta(theta_ + rhs.theta_);
    return *this;
  }

  /**
   * @brief Arithmetic operator overload for additions
   * @param lhs First addend
   * @param rhs Second addend
   */
  friend PoseSE2 operator+(PoseSE2 lhs, const PoseSE2& rhs) {
    return lhs += rhs;
  }

  /**
   * @brief Compound assignment operator (subtraction)
   * @param rhs value to subtract
   */
  PoseSE2& operator-=(const PoseSE2& rhs) {
    position_ -= rhs.position_;
    theta_ = g2o::normalize_theta(theta_ - rhs.theta_);
    return *this;
  }

  /**
   * @brief Arithmetic operator overload for subtractions
   * @param lhs First term
   * @param rhs Second term
   */
  friend PoseSE2 operator-(PoseSE2 lhs, const PoseSE2& rhs) {
    return lhs -= rhs;
  }

  /**
   * @brief Multiply pose with scalar and return copy without normalizing theta
   * This operator is useful for calculating velocities ...
   * @param pose pose to scale
   * @param scalar factor to multiply with
   * @warning theta is not normalized after multiplying
   */
  friend PoseSE2 operator*(PoseSE2 pose, double scalar) {
    pose.position_ *= scalar;
    pose.theta_ *= scalar;
    return pose;
  }

  /**
   * @brief Multiply pose with scalar and return copy without normalizing theta
   * This operator is useful for calculating velocities ...
   * @param scalar factor to multiply with
   * @param pose pose to scale
   * @warning theta is not normalized after multiplying
   */
  friend PoseSE2 operator*(double scalar, PoseSE2 pose) {
    pose.position_ *= scalar;
    pose.theta_ *= scalar;
    return pose;
  }

  /**
   * @brief Output stream operator
   * @param stream output stream
   * @param pose to be used
   */
  friend std::ostream& operator<<(std::ostream& stream, const PoseSE2& pose) {
    stream << "x: " << pose.position_[0] << " y: " << pose.position_[1]
           << " theta: " << pose.theta_;
    return stream;
  }

 private:
  Eigen::Vector2d position_;
  double theta_;
};

}  // namespace teb_local_planner
