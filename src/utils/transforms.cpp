#include "biped_control/utils/transforms.hpp"
#include <cmath>

namespace biped_control {
namespace utils {

Eigen::Matrix3d hatting(double x, double y, double theta) {
    Eigen::Matrix3d g = Eigen::Matrix3d::Identity();
    
    // Rotation part
    g(0,0) = std::cos(theta);
    g(0,1) = -std::sin(theta);
    g(1,0) = std::sin(theta);
    g(1,1) = std::cos(theta);
    
    // Translation part
    g(0,2) = x;
    g(1,2) = y;
    
    return g;
}

std::pair<Eigen::Vector2d, double> unhatting(const Eigen::Matrix3d& g) {
    Eigen::Vector2d translation(g(0,2), g(1,2));
    double theta = std::atan2(g(1,0), g(0,0));
    
    return std::make_pair(translation, theta);
}

SE2Transform compose(const SE2Transform& g1, const SE2Transform& g2) {
    // Convert to matrices
    Eigen::Matrix3d m1 = hatting(g1.translation.x(), 
                                g1.translation.y(), 
                                g1.rotation);
    Eigen::Matrix3d m2 = hatting(g2.translation.x(), 
                                g2.translation.y(), 
                                g2.rotation);
    
    // Compose
    Eigen::Matrix3d result = m1 * m2;
    
    // Convert back to SE2Transform
    auto [translation, rotation] = unhatting(result);
    return SE2Transform(translation.x(), translation.y(), rotation);
}

SE2Transform inverse(const SE2Transform& g) {
    // For SE(2), the inverse is:
    // R^T | -R^T * t
    // 0   |    1
    double cos_theta = std::cos(g.rotation);
    double sin_theta = std::sin(g.rotation);
    
    double x = g.translation.x();
    double y = g.translation.y();
    
    SE2Transform result;
    result.rotation = -g.rotation;
    result.translation.x() = -x * cos_theta - y * sin_theta;
    result.translation.y() = x * sin_theta - y * cos_theta;
    
    return result;
}

} // namespace utils
} // namespace biped_control