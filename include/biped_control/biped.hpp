#pragma once

#include "biped_control/types.hpp"
#include "biped_control/utils/transforms.hpp"
#include "biped_control/kinematics/forward_kinematics.hpp"
#include "biped_control/visualization/biped_visualizer.hpp"
#include "biped_control/hardware/motor_controller.hpp"
#include <ros/ros.h>
#include <memory>
#include <string>

namespace biped_control {

class Biped {
public:
    /**
     * @brief Constructor
     * @param nh ROS node handle
     * @param use_visualization Enable/disable visualization
     */
    Biped(ros::NodeHandle& nh, bool use_visualization = true);
    ~Biped();

    /**
     * @brief Initialize the robot
     * @return true if initialization successful
     */
    bool initialize();

    /**
     * @brief Load robot model from YAML configuration
     * @param model_file Path to model YAML file
     * @return true if model loaded successfully
     */
    bool loadModel(const std::string& model_file);

    // Configuration methods
    /**
     * @brief Set the current stance of the robot
     * @param stance New stance (LEFT_FOOT, RIGHT_FOOT, or TORSO)
     */
    void setStance(Stance stance);

    /**
     * @brief Set joint angles and update kinematics
     * @param angles Array of joint angles in radians
     */
    void setJointAngles(const std::array<double, NUM_JOINTS>& angles);

    /**
     * @brief Set robot geometry parameters
     * @param geometry Robot geometry parameters
     */
    void setGeometry(const RobotGeometry& geometry);

    // State access methods
    /**
     * @brief Get current joint angles
     * @return Array of current joint angles
     */
    std::array<double, NUM_JOINTS> getJointAngles() const;

    /**
     * @brief Get center of mass position
     * @return 2D vector of COM position
     */
    Vector2d getCOM() const { return com_; }

    /**
     * @brief Get transform for a specific frame
     * @param frame_name Name of the frame
     * @return SE(2) transform of the frame
     */
    utils::SE2Transform getTransform(const std::string& frame_name) const;

    /**
     * @brief Check if robot is balanced in current configuration
     * @return true if robot is balanced
     */
    bool isBalanced() const;

    // Motion control methods
    /**
     * @brief Move joints to target positions
     * @param target_angles Target joint angles
     * @param duration Time to reach target
     * @return true if motion started successfully
     */
    bool moveToTarget(const std::array<double, NUM_JOINTS>& target_angles, 
                     double duration = 1.0);

    /**
     * @brief Execute a walking trajectory
     * @param trajectory Vector of joint angles
     * @param times Vector of time points
     * @return true if trajectory started successfully
     */
    bool executeTrajectory(const std::vector<std::array<double, NUM_JOINTS>>& trajectory,
                          const std::vector<double>& times);

    /**
     * @brief Stop all motion immediately
     */
    void stop();

    // Visualization methods
    /**
     * @brief Start real-time visualization
     * @return true if visualization started successfully
     */
    bool startVisualization();

    /**
     * @brief Stop visualization
     */
    void stopVisualization();

private:
    // ROS communication
    ros::NodeHandle& nh_;
    
    // Component interfaces
    std::unique_ptr<hardware::MotorController> motor_controller_;
    std::unique_ptr<visualization::BipedVisualizer> visualizer_;
    
    // Robot state
    Vector2d com_;
    std::array<double, NUM_JOINTS> joint_angles_;
    std::map<std::string, utils::SE2Transform> joint_transforms_;
    Stance current_stance_;
    
    // Robot parameters
    RobotGeometry geometry_;
    double com_motor_displacement_;
    double foot_width_;
    bool use_visualization_;
    bool is_initialized_;

    // Internal methods
    void updateKinematics();
    void updateVisualization();
    bool validateConfiguration();
    void loadDefaultParameters();
};

} // namespace biped_control