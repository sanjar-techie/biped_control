#include "biped_control/biped.hpp"
#include "biped_control/kinematics/forward_kinematics.hpp"
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

namespace biped_control {

Biped::Biped(ros::NodeHandle& nh, bool use_visualization)
    : nh_(nh)
    , use_visualization_(use_visualization)
    , is_initialized_(false)
    , current_stance_(Stance::TORSO) {
    
    // Create components
    motor_controller_ = std::make_unique<hardware::MotorController>(nh);
    if (use_visualization_) {
        visualizer_ = std::make_unique<visualization::BipedVisualizer>();
    }
    
    // Initialize default values
    loadDefaultParameters();
}

Biped::~Biped() {
    if (use_visualization_) {
        visualizer_->stopVisualization();
    }
}

bool Biped::initialize() {
    if (is_initialized_) {
        return true;
    }

    // Initialize hardware
    if (!motor_controller_->initialize()) {
        ROS_ERROR("Failed to initialize motor controller");
        return false;
    }

    // Initialize visualization if enabled
    if (use_visualization_ && !visualizer_->initialize()) {
        ROS_ERROR("Failed to initialize visualizer");
        return false;
    }

    // Load model from default file
    std::string model_file = ros::package::getPath("biped_control") + 
                            "/config/biped_model.yaml";
    if (!loadModel(model_file)) {
        ROS_ERROR("Failed to load default model");
        return false;
    }

    is_initialized_ = true;
    return true;
}

bool Biped::loadModel(const std::string& model_file) {
    try {
        YAML::Node config = YAML::LoadFile(model_file);
        
        // Load geometry
        auto geom = config["biped"]["geometry"];
        geometry_.torso_length = geom[0].as<double>();
        geometry_.thigh_length = geom[1].as<double>();
        geometry_.calf_length = geom[2].as<double>();
        geometry_.foot_height = geom[3].as<double>();

        // Load initial angles
        auto alpha = config["biped"]["alpha"];
        for (size_t i = 0; i < NUM_JOINTS; ++i) {
            joint_angles_[i] = alpha[i].as<double>();
        }

        // Load other parameters
        com_motor_displacement_ = config["biped"]["com_displacement_from_joint"]
            .as<double>();
        foot_width_ = config["biped"]["foot_length"].as<double>();

        // Update kinematics with new parameters
        updateKinematics();
        
        return true;
    } catch (const YAML::Exception& e) {
        ROS_ERROR_STREAM("Failed to load model: " << e.what());
        return false;
    }
}

void Biped::setStance(Stance stance) {
    if (stance != current_stance_) {
        current_stance_ = stance;
        updateKinematics();
    }
}

void Biped::setJointAngles(const std::array<double, NUM_JOINTS>& angles) {
    if (validateConfiguration()) {
        joint_angles_ = angles;
        updateKinematics();
    }
}

void Biped::setGeometry(const RobotGeometry& geometry) {
    geometry_ = geometry;
    updateKinematics();
}

std::array<double, NUM_JOINTS> Biped::getJointAngles() const {
    return joint_angles_;
}

utils::SE2Transform Biped::getTransform(const std::string& frame_name) const {
    auto it = joint_transforms_.find(frame_name);
    if (it != joint_transforms_.end()) {
        return it->second;
    }
    throw std::runtime_error("Invalid frame name: " + frame_name);
}

bool Biped::isBalanced() const {
    if (current_stance_ == Stance::TORSO) {
        return true;  // No balance check needed for torso stance
    }
    return (com_.x() > -0.5 * foot_width_ && com_.x() < 0.5 * foot_width_);
}

bool Biped::moveToTarget(
    const std::array<double, NUM_JOINTS>& target_angles, 
    double duration) {
    
    if (!is_initialized_) {
        ROS_ERROR("Biped not initialized");
        return false;
    }

    // Validate target configuration
    auto temp_angles = joint_angles_;
    joint_angles_ = target_angles;
    if (!validateConfiguration()) {
        joint_angles_ = temp_angles;
        return false;
    }
    joint_angles_ = temp_angles;

    // Execute motion
    return motor_controller_->moveJoints(target_angles, duration);
}

bool Biped::executeTrajectory(
    const std::vector<std::array<double, NUM_JOINTS>>& trajectory,
    const std::vector<double>& times) {
    
    if (!is_initialized_) {
        ROS_ERROR("Biped not initialized");
        return false;
    }

    // Validate entire trajectory
    for (const auto& angles : trajectory) {
        auto temp_angles = joint_angles_;
        joint_angles_ = angles;
        if (!validateConfiguration()) {
            joint_angles_ = temp_angles;
            return false;
        }
        joint_angles_ = temp_angles;
    }

    return motor_controller_->executeTrajectory(trajectory, times);
}

void Biped::stop() {
    if (motor_controller_) {
        motor_controller_->emergencyStop();
    }
}

bool Biped::startVisualization() {
    if (!use_visualization_ || !visualizer_) {
        return false;
    }
    return visualizer_->startRealtimeVisualization();
}

void Biped::stopVisualization() {
    if (visualizer_) {
        visualizer_->stopVisualization();
    }
}

void Biped::updateKinematics() {
    // Compute forward kinematics
    joint_transforms_ = kinematics::ForwardKinematics::compute(
        geometry_, joint_angles_, current_stance_, com_motor_displacement_);
    
    // Update COM position
    com_ = kinematics::ForwardKinematics::computeCOM(
        geometry_, joint_angles_, current_stance_, com_motor_displacement_);

    // Update visualization if enabled
    if (use_visualization_ && visualizer_) {
        visualizer_->update(joint_transforms_, com_);
    }
}

bool Biped::validateConfiguration() {
    // Check joint limits
    for (double angle : joint_angles_) {
        if (std::abs(angle) > M_PI/3.0) {  // ±60 degrees limit
            ROS_ERROR("Joint angle exceeds limit");
            return false;
        }
    }

    // Compute and check kinematics
    auto test_transforms = kinematics::ForwardKinematics::compute(
        geometry_, joint_angles_, current_stance_, com_motor_displacement_);
    
    // Check for invalid transforms
    for (const auto& [name, transform] : test_transforms) {
        if (std::isnan(transform.translation.x()) || 
            std::isnan(transform.translation.y()) ||
            std::isnan(transform.rotation)) {
            ROS_ERROR_STREAM("Invalid transform for frame: " << name);
            return false;
        }
    }

    return true;
}

void Biped::loadDefaultParameters() {
    geometry_ = RobotGeometry();  // Uses default constructor values
    joint_angles_.fill(0.0);
    com_motor_displacement_ = 1.35;  // Default from original config
    foot_width_ = 6.0;  // Default from original config
}

} // namespace biped_control