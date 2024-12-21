#pragma once

#include "biped_control/types.hpp"
#include <ros/ros.h>
#include <dynamixel_msgs/JointState.h>
#include <sensor_msgs/JointState.h>
#include <map>
#include <string>
#include <memory>

namespace biped_control {
namespace hardware {

class DynamixelInterface {
public:
    DynamixelInterface(ros::NodeHandle& nh);
    ~DynamixelInterface();

    /**
     * @brief Initialize the interface with motor configurations
     * @return true if initialization successful
     */
    bool initialize();

    /**
     * @brief Set target positions for all joints
     * @param positions Array of target positions in radians
     * @return true if command was sent successfully
     */
    bool setJointPositions(const std::array<double, NUM_JOINTS>& positions);

    /**
     * @brief Get current joint states
     * @return Current joint state including position, velocity, and effort
     */
    JointState getJointStates() const;

    /**
     * @brief Execute a trajectory
     * @param trajectory Vector of joint positions
     * @param times Vector of time points
     * @param velocities Vector of joint velocities
     * @return true if trajectory execution started successfully
     */
    bool executeTrajectory(const std::vector<std::array<double, NUM_JOINTS>>& trajectory,
                          const std::vector<double>& times,
                          const std::vector<std::array<double, NUM_JOINTS>>& velocities);

    /**
     * @brief Enable torque for all motors
     * @return true if successful
     */
    bool enableTorque();

    /**
     * @brief Disable torque for all motors
     * @return true if successful
     */
    bool disableTorque();

private:
    // ROS communication
    ros::NodeHandle& nh_;
    std::vector<ros::Publisher> joint_command_pubs_;
    std::vector<ros::Subscriber> joint_state_subs_;
    ros::Publisher trajectory_pub_;
    ros::Publisher joint_states_pub_;

    // Motor states
    std::map<int, dynamixel_msgs::JointState> motor_states_;
    
    // Configuration
    std::array<MotorConfig, NUM_JOINTS> motor_configs_;
    bool is_initialized_;

    // Callbacks
    void jointStateCallback(const dynamixel_msgs::JointState::ConstPtr& msg, int motor_id);
    void publishJointStates();

    // Conversion utilities
    double ticksToRadians(int motor_id, int ticks) const;
    int radiansToTicks(int motor_id, double radians) const;

    // Configuration loading
    bool loadMotorConfigs();
    bool setupPublishersAndSubscribers();
    bool waitForMotorConnection(double timeout_sec = 5.0);
};

} // namespace hardware
} // namespace biped_control