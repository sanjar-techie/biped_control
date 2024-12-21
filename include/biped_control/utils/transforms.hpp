#pragma once

#include <Eigen/Dense>
#include <utility>

namespace biped_control {
namespace utils {

// Transform representation for 2D (SE(2))
struct SE2Transform {
    Eigen::Vector2d translation;
    double rotation;

    SE2Transform() : translation(Eigen::Vector2d::Zero()), rotation(0.0) {}
    SE2Transform(double x, double y, double theta) 
        : translation(x, y), rotation(theta) {}
};

/**
 * @brief Creates a homogeneous transformation matrix from x, y translation and rotation
 * @param x X translation
 * @param y Y translation
 * @param theta Rotation angle in radians
 * @return 3x3 homogeneous transformation matrix
 */
Eigen::Matrix3d hatting(double x, double y, double theta);

/**
 * @brief Extracts translation and rotation from a homogeneous transformation matrix
 * @param g 3x3 homogeneous transformation matrix
 * @return Pair of Vector2d (translation) and double (rotation)
 */
std::pair<Eigen::Vector2d, double> unhatting(const Eigen::Matrix3d& g);

/**
 * @brief Composes two SE(2) transforms
 * @param g1 First transform
 * @param g2 Second transform
 * @return Composed transform
 */
SE2Transform compose(const SE2Transform& g1, const SE2Transform& g2);

/**
 * @brief Inverts an SE(2) transform
 * @param g Transform to invert
 * @return Inverted transform
 */
SE2Transform inverse(const SE2Transform& g);

} // namespace utils
} // namespace biped_control