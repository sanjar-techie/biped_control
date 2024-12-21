#pragma once

#include "biped_control/types.hpp"
#include "biped_control/hardware/dynamixel_interface.hpp"
#include <ros/ros.h>
#include <memory>
#include <string>
#include <vector>

namespace biped_control {
namespace hardware {

class MotorController {
public:
    /**
     * @brief Constructor
     * @param nh ROS node handle
     */
    MotorController(ros::NodeHandle& nh);
    ~MotorController();

    /**
     * @brief Initialize the controller and underlying hardware
     * @return true if initialization successful
     */
    bool initialize();

    /**
     * @brief Move all joints to specified positions
     * @param positions Target joint positions in radians
     * @param duration Time to reach target in seconds
     * @return true if motion command accepted
     */
    bool moveJoints(const std::array<double, NUM_JOINTS>& positions, 
                   double duration = 1.0);

    /**
     * @brief Execute a walking trajectory
     * @param joint_positions Vector of joint position arrays
     * @param times Vector of time points
     * @param joint_velocities Optional vector of joint velocity arrays
     * @return true if trajectory execution started
     */
    bool executeTrajectory(
        const std::vector<std::array<double, NUM_JOINTS>>& joint_positions,
        const std::vector<double>& times,
        const std::vector<std::array<double, NUM_JOINTS>>& joint_velocities = {});

    /**
     * @brief Emergency stop - halts all motion
     */
    void emergencyStop();

    /**
     * @brief Get current joint states
     * @return Current joint positions, velocities, and efforts
     */
    JointState getCurrentState() const;

    /**
     * @brief Check if the controller is currently executing a trajectory
     * @return true if trajectory is in progress
     */
    bool isTrajectoryActive() const;

    /**
     * @brief Wait for current trajectory to complete
     * @param timeout_sec Maximum time to wait in seconds
     * @return true if trajectory completed successfully
     */
    bool waitForTrajectoryCompletion(double timeout_sec = 30.0);

private:
    ros::NodeHandle& nh_;
    std::unique_ptr<DynamixelInterface> hardware_;
    
    // Controller state
    bool is_initialized_;
    bool trajectory_active_;
    ros::Time trajectory_start_time_;
    
    // Safety parameters
    static constexpr double MAX_VELOCITY = 1.5;  // rad/s
    static constexpr double MAX_ACCELERATION = 2.0;  // rad/s^2
    static constexpr double POSITION_TOLERANCE = 0.05;  // rad
    
    // Trajectory execution
    bool validateTrajectory(
        const std::vector<std::array<double, NUM_JOINTS>>& positions,
        const std::vector<double>& times);
    
    void trajectoryTimerCallback(const ros::TimerEvent& event);
    ros::Timer trajectory_timer_;
    
    // Current trajectory data
    std::vector<std::array<double, NUM_JOINTS>> current_trajectory_;
    std::vector<double> trajectory_times_;
    std::vector<std::array<double, NUM_JOINTS>> trajectory_velocities_;
    size_t trajectory_index_;
    
    // Safety checks
    bool checkJointLimits(const std::array<double, NUM_JOINTS>& positions) const;
    bool checkVelocityLimits(const std::array<double, NUM_JOINTS>& velocities) const;
    bool checkAccelerationLimits(
        const std::array<double, NUM_JOINTS>& current_pos,
        const std::array<double, NUM_JOINTS>& target_pos,
        double duration) const;
};

} // namespace hardware
} // namespace biped_control