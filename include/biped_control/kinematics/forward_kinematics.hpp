#pragma once

#include "biped_control/types.hpp"
#include "biped_control/utils/transforms.hpp"
#include <map>
#include <string>

namespace biped_control {
namespace kinematics {

class ForwardKinematics {
public:
    /**
     * @brief Computes forward kinematics for the entire robot
     * @param geometry Robot geometry parameters
     * @param joint_angles Current joint angles
     * @param stance Current stance (LEFT_FOOT, RIGHT_FOOT, or TORSO)
     * @param com_displacement Center of mass displacement from joint
     * @return Map of frame names to their transforms
     */
    static std::map<std::string, utils::SE2Transform> 
    compute(const RobotGeometry& geometry,
           const std::array<double, NUM_JOINTS>& joint_angles,
           Stance stance,
           double com_displacement);

    /**
     * @brief Computes the center of mass position
     * @param geometry Robot geometry parameters
     * @param joint_angles Current joint angles
     * @param stance Current stance
     * @param com_displacement Center of mass displacement from joint
     * @return 2D vector of COM position
     */
    static Vector2d 
    computeCOM(const RobotGeometry& geometry,
               const std::array<double, NUM_JOINTS>& joint_angles,
               Stance stance,
               double com_displacement);

    /**
     * @brief Computes the Jacobian matrix for a target frame with respect to a reference frame
     * @param geometry Robot geometry
     * @param joint_angles Current joint configuration
     * @param ref_frame Reference frame
     * @param target_frame Target frame
     * @return 3x3 Jacobian matrix [dx/dq, dy/dq, dtheta/dq]
     */
    static Matrix3d
    computeJacobian(const RobotGeometry& geometry,
                    const std::array<double, NUM_JOINTS>& joint_angles,
                    const std::string& ref_frame,
                    const std::string& target_frame);

private:
    // Helper methods for different stance computations
    static std::map<std::string, utils::SE2Transform> 
    computeFromRightFoot(const RobotGeometry& geometry,
                        const std::array<double, NUM_JOINTS>& joint_angles,
                        double com_displacement);

    static std::map<std::string, utils::SE2Transform>
    computeFromLeftFoot(const RobotGeometry& geometry,
                       const std::array<double, NUM_JOINTS>& joint_angles,
                       double com_displacement);

    static std::map<std::string, utils::SE2Transform>
    computeFromTorso(const RobotGeometry& geometry,
                    const std::array<double, NUM_JOINTS>& joint_angles,
                    double com_displacement);

    // Helper method to update COM for a single motor
    static void updateCOMContribution(Vector2d& com,
                                    const utils::SE2Transform& motor_transform,
                                    double com_displacement);

    // Prevent instantiation as this is a static utility class
    ForwardKinematics() = delete;
    ~ForwardKinematics() = delete;
};

} // namespace kinematics
} // namespace biped_control