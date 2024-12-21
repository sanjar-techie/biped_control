#include "biped_control/hardware/dynamixel_interface.hpp"
#include <dynamixel_controllers/SetComplianceMargin.h>
#include <dynamixel_controllers/SetComplianceSlope.h>
#include <dynamixel_controllers/SetSpeed.h>
#include <dynamixel_controllers/TorqueEnable.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <cmath>

namespace biped_control {
namespace hardware {

DynamixelInterface::DynamixelInterface(ros::NodeHandle& nh) 
    : nh_(nh)
    , is_initialized_(false) {
    
    motor_states_.clear();
    joint_command_pubs_.resize(NUM_JOINTS);
    joint_state_subs_.resize(NUM_JOINTS);
}

DynamixelInterface::~DynamixelInterface() {
    if (is_initialized_) {
        disableTorque();
    }
}

bool DynamixelInterface::initialize() {
    if (is_initialized_) {
        return true;
    }

    if (!loadMotorConfigs()) {
        ROS_ERROR("Failed to load motor configurations");
        return false;
    }

    if (!setupPublishersAndSubscribers()) {
        ROS_ERROR("Failed to setup ROS communication");
        return false;
    }

    if (!waitForMotorConnection()) {
        ROS_ERROR("Timeout waiting for motor connection");
        return false;
    }

    trajectory_pub_ = nh_.advertise<trajectory_msgs::JointTrajectory>(
        "/biped_joint_trajectory_action_controller/command", 1);
    
    joint_states_pub_ = nh_.advertise<sensor_msgs::JointState>(
        "/joint_states", 10);

    is_initialized_ = true;
    return true;
}

bool DynamixelInterface::loadMotorConfigs() {
    for (int i = 0; i < NUM_JOINTS; ++i) {
        std::string prefix = "/biped_position_controller_" + std::to_string(i + 1);
        
        // Load motor parameters
        MotorConfig config;
        if (!nh_.getParam(prefix + "/motor/id", config.id) ||
            !nh_.getParam(prefix + "/motor/init", config.init_pos) ||
            !nh_.getParam(prefix + "/motor/min", config.min_pos) ||
            !nh_.getParam(prefix + "/motor/max", config.max_pos)) {
            ROS_ERROR("Failed to load motor parameters for joint %d", i + 1);
            return false;
        }
        
        // Get optional bias parameter
        nh_.param(prefix + "/motor/bias", config.bias, 0.0);
        motor_configs_[i] = config;
    }
    return true;
}

bool DynamixelInterface::setupPublishersAndSubscribers() {
    for (int i = 0; i < NUM_JOINTS; ++i) {
        std::string prefix = "/biped_position_controller_" + std::to_string(i + 1);
        
        // Setup publishers
        joint_command_pubs_[i] = nh_.advertise<std_msgs::Float64>(
            prefix + "/command", 1);
        
        // Setup subscribers
        joint_state_subs_[i] = nh_.subscribe<dynamixel_msgs::JointState>(
            prefix + "/state", 1,
            boost::bind(&DynamixelInterface::jointStateCallback, this, _1, i));
    }
    return true;
}

bool DynamixelInterface::waitForMotorConnection(double timeout_sec) {
    ros::Time start_time = ros::Time::now();
    ros::Rate rate(10);  // 10 Hz checking
    
    while (ros::ok()) {
        bool all_connected = true;
        for (const auto& state : motor_states_) {
            if (state.second.current_pos == 0.0) {
                all_connected = false;
                break;
            }
        }
        
        if (all_connected) {
            return true;
        }
        
        if ((ros::Time::now() - start_time).toSec() > timeout_sec) {
            return false;
        }
        
        rate.sleep();
        ros::spinOnce();
    }
    return false;
}

void DynamixelInterface::jointStateCallback(
    const dynamixel_msgs::JointState::ConstPtr& msg, int motor_id) {
    motor_states_[motor_id] = *msg;
    publishJointStates();
}

void DynamixelInterface::publishJointStates() {
    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    
    for (const auto& [id, state] : motor_states_) {
        msg.name.push_back(state.name);
        msg.position.push_back(state.current_pos);
        msg.velocity.push_back(state.velocity);
        msg.effort.push_back(state.load);
    }
    
    joint_states_pub_.publish(msg);
}

bool DynamixelInterface::setJointPositions(
    const std::array<double, NUM_JOINTS>& positions) {
    if (!is_initialized_) {
        ROS_ERROR("DynamixelInterface not initialized");
        return false;
    }

    for (int i = 0; i < NUM_JOINTS; ++i) {
        std_msgs::Float64 command;
        command.data = positions[i];
        joint_command_pubs_[i].publish(command);
    }
    return true;
}

JointState DynamixelInterface::getJointStates() const {
    JointState state;
    for (const auto& [id, motor_state] : motor_states_) {
        state.position[id] = motor_state.current_pos;
        state.velocity[id] = motor_state.velocity;
        state.effort[id] = motor_state.load;
    }
    return state;
}

bool DynamixelInterface::executeTrajectory(
    const std::vector<std::array<double, NUM_JOINTS>>& trajectory,
    const std::vector<double>& times,
    const std::vector<std::array<double, NUM_JOINTS>>& velocities) {
    
    if (!is_initialized_) {
        ROS_ERROR("DynamixelInterface not initialized");
        return false;
    }

    trajectory_msgs::JointTrajectory traj_msg;
    traj_msg.header.stamp = ros::Time::now();
    
    // Add joint names
    for (int i = 0; i < NUM_JOINTS; ++i) {
        traj_msg.joint_names.push_back("joint_" + std::to_string(i + 1));
    }

    // Fill trajectory points
    for (size_t i = 0; i < trajectory.size(); ++i) {
        trajectory_msgs::JointTrajectoryPoint point;
        point.positions = std::vector<double>(trajectory[i].begin(), trajectory[i].end());
        
        if (!velocities.empty()) {
            point.velocities = std::vector<double>(velocities[i].begin(), velocities[i].end());
        }
        
        point.time_from_start = ros::Duration(times[i]);
        traj_msg.points.push_back(point);
    }

    trajectory_pub_.publish(traj_msg);
    return true;
}

bool DynamixelInterface::enableTorque() {
    if (!is_initialized_) return false;

    for (int i = 0; i < NUM_JOINTS; ++i) {
        std::string service_name = "/biped_position_controller_" + 
                                 std::to_string(i + 1) + 
                                 "/torque_enable";
        ros::ServiceClient client = 
            nh_.serviceClient<dynamixel_controllers::TorqueEnable>(service_name);
        
        dynamixel_controllers::TorqueEnable srv;
        srv.request.torque_enable = true;
        
        if (!client.call(srv)) {
            ROS_ERROR("Failed to enable torque for joint %d", i + 1);
            return false;
        }
    }
    return true;
}

bool DynamixelInterface::disableTorque() {
    for (int i = 0; i < NUM_JOINTS; ++i) {
        std::string service_name = "/biped_position_controller_" + 
                                 std::to_string(i + 1) + 
                                 "/torque_enable";
        ros::ServiceClient client = 
            nh_.serviceClient<dynamixel_controllers::TorqueEnable>(service_name);
        
        dynamixel_controllers::TorqueEnable srv;
        srv.request.torque_enable = false;
        
        if (!client.call(srv)) {
            ROS_ERROR("Failed to disable torque for joint %d", i + 1);
            return false;
        }
    }
    return true;
}

double DynamixelInterface::ticksToRadians(int motor_id, int ticks) const {
    const auto& config = motor_configs_[motor_id];
    return (2.0 * M_PI * (ticks - config.init_pos)) / 
           (config.max_pos - config.min_pos) + config.bias;
}

int DynamixelInterface::radiansToTicks(int motor_id, double radians) const {
    const auto& config = motor_configs_[motor_id];
    return static_cast<int>(
        ((radians - config.bias) * (config.max_pos - config.min_pos)) / 
        (2.0 * M_PI) + config.init_pos
    );
}

} // namespace hardware
} // namespace biped_control