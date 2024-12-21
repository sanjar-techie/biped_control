#pragma once

#include <array>
#include <string>
#include <Eigen/Dense>

namespace biped_control {

// Common types using Eigen
using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using VectorXd = Eigen::VectorXd;
using MatrixXd = Eigen::MatrixXd;

// Constants
constexpr int NUM_JOINTS = 6;
constexpr double PI = 3.14159265359;

// Stance types
enum class Stance {
    LEFT_FOOT,
    RIGHT_FOOT,
    TORSO
};

// Frame identifiers
struct Frames {
    static constexpr const char* RIGHT_FOOT = "right_foot";
    static constexpr const char* LEFT_FOOT = "left_foot";
    static constexpr const char* TORSO = "torso";
};

// Robot geometry structure
struct RobotGeometry {
    double torso_length;    // l0
    double thigh_length;    // l1
    double calf_length;     // l2
    double foot_height;     // l3
    
    RobotGeometry()
        : torso_length(4.5)
        , thigh_length(9.5)
        , calf_length(7.0)
        , foot_height(4.0)
    {}
};

// Joint state structure
struct JointState {
    std::array<double, NUM_JOINTS> position;
    std::array<double, NUM_JOINTS> velocity;
    std::array<double, NUM_JOINTS> effort;
    
    JointState() {
        position.fill(0.0);
        velocity.fill(0.0);
        effort.fill(0.0);
    }
};

// Motor configuration
struct MotorConfig {
    int id;
    double init_pos;
    double min_pos;
    double max_pos;
    double bias;
    
    MotorConfig()
        : id(0)
        , init_pos(512)
        , min_pos(0)
        , max_pos(1024)
        , bias(0.0)
    {}
};

} // namespace biped_control