#include "biped_control/kinematics/forward_kinematics.hpp"
#include <cmath>

namespace biped_control {
namespace kinematics {

std::map<std::string, utils::SE2Transform> 
ForwardKinematics::compute(const RobotGeometry& geometry,
                          const std::array<double, NUM_JOINTS>& joint_angles,
                          Stance stance,
                          double com_displacement) {
    switch(stance) {
        case Stance::RIGHT_FOOT:
            return computeFromRightFoot(geometry, joint_angles, com_displacement);
        case Stance::LEFT_FOOT:
            return computeFromLeftFoot(geometry, joint_angles, com_displacement);
        case Stance::TORSO:
            return computeFromTorso(geometry, joint_angles, com_displacement);
        default:
            throw std::runtime_error("Invalid stance specified");
    }
}

std::map<std::string, utils::SE2Transform> 
ForwardKinematics::computeFromRightFoot(const RobotGeometry& geometry,
                                      const std::array<double, NUM_JOINTS>& joint_angles,
                                      double com_displacement) {
    std::map<std::string, utils::SE2Transform> transforms;
    
    // Initialize right foot frame as origin
    transforms[Frames::RIGHT_FOOT] = utils::SE2Transform(0, 0, 0);
    
    // Compute transforms for right leg (bottom to top)
    auto g1 = utils::hatting(0, geometry.foot_height, joint_angles[5]); // Joint 6
    transforms["joint_6"] = utils::SE2Transform(g1(0,2), g1(1,2), std::atan2(g1(1,0), g1(0,0)));
    
    auto g2 = g1 * utils::hatting(0, geometry.calf_length, joint_angles[4]); // Joint 5
    transforms["joint_5"] = utils::SE2Transform(g2(0,2), g2(1,2), std::atan2(g2(1,0), g2(0,0)));
    
    auto g3 = g2 * utils::hatting(0, geometry.thigh_length, joint_angles[3]); // Joint 4
    transforms["joint_4"] = utils::SE2Transform(g3(0,2), g3(1,2), std::atan2(g3(1,0), g3(0,0)));
    
    // Torso frame
    auto g4 = g3 * utils::hatting(0, geometry.torso_length, 0);
    transforms[Frames::TORSO] = utils::SE2Transform(g4(0,2), g4(1,2), std::atan2(g4(1,0), g4(0,0)));
    
    // Compute transforms for left leg (top to bottom)
    auto g5 = g4 * utils::hatting(0, -geometry.torso_length, joint_angles[2]); // Joint 3
    transforms["joint_3"] = utils::SE2Transform(g5(0,2), g5(1,2), std::atan2(g5(1,0), g5(0,0)));
    
    auto g6 = g5 * utils::hatting(0, -geometry.thigh_length, joint_angles[1]); // Joint 2
    transforms["joint_2"] = utils::SE2Transform(g6(0,2), g6(1,2), std::atan2(g6(1,0), g6(0,0)));
    
    auto g7 = g6 * utils::hatting(0, -geometry.calf_length, joint_angles[0]); // Joint 1
    transforms["joint_1"] = utils::SE2Transform(g7(0,2), g7(1,2), std::atan2(g7(1,0), g7(0,0)));
    
    auto g8 = g7 * utils::hatting(0, -geometry.foot_height, 0);
    transforms[Frames::LEFT_FOOT] = utils::SE2Transform(g8(0,2), g8(1,2), std::atan2(g8(1,0), g8(0,0)));
    
    return transforms;
}

Vector2d 
ForwardKinematics::computeCOM(const RobotGeometry& geometry,
                             const std::array<double, NUM_JOINTS>& joint_angles,
                             Stance stance,
                             double com_displacement) {
    Vector2d com = Vector2d::Zero();
    auto transforms = compute(geometry, joint_angles, stance, com_displacement);
    
    // Add COM contribution from each motor
    for (int i = 0; i < NUM_JOINTS; ++i) {
        std::string joint_name = "joint_" + std::to_string(i + 1);
        auto& transform = transforms[joint_name];
        updateCOMContribution(com, transform, com_displacement);
    }
    
    // Average the COM position
    com /= NUM_JOINTS;
    return com;
}

void ForwardKinematics::updateCOMContribution(Vector2d& com,
                                            const utils::SE2Transform& motor_transform,
                                            double com_displacement) {
    // Compute COM position for this motor
    double motor_angle = motor_transform.rotation;
    Vector2d motor_com;
    motor_com.x() = motor_transform.translation.x() + com_displacement * std::sin(motor_angle);
    motor_com.y() = motor_transform.translation.y() - com_displacement * std::cos(motor_angle);
    
    com += motor_com;
}

Matrix3d
ForwardKinematics::computeJacobian(const RobotGeometry& geometry,
                                  const std::array<double, NUM_JOINTS>& joint_angles,
                                  const std::string& ref_frame,
                                  const std::string& target_frame) {
    if (ref_frame != Frames::TORSO) {
        throw std::runtime_error("Currently only supporting Jacobian computation from torso frame");
    }
    
    Matrix3d jacobian = Matrix3d::Zero();
    
    if (target_frame == Frames::RIGHT_FOOT) {
        // Compute Jacobian for right leg joints
        double theta4 = joint_angles[3];
        double theta5 = joint_angles[4];
        double theta6 = joint_angles[5];
        
        // For x component
        jacobian(0,0) = -geometry.thigh_length * std::cos(theta4)
                        - geometry.calf_length * std::cos(theta4 + theta5)
                        - geometry.foot_height * std::cos(theta4 + theta5 + theta6);
                        
        jacobian(0,1) = -geometry.calf_length * std::cos(theta4 + theta5)
                        - geometry.foot_height * std::cos(theta4 + theta5 + theta6);
                        
        jacobian(0,2) = -geometry.foot_height * std::cos(theta4 + theta5 + theta6);
        
        // For y component
        jacobian(1,0) = geometry.thigh_length * std::sin(theta4)
                        + geometry.calf_length * std::sin(theta4 + theta5)
                        + geometry.foot_height * std::sin(theta4 + theta5 + theta6);
                        
        jacobian(1,1) = geometry.calf_length * std::sin(theta4 + theta5)
                        + geometry.foot_height * std::sin(theta4 + theta5 + theta6);
                        
        jacobian(1,2) = geometry.foot_height * std::sin(theta4 + theta5 + theta6);
        
        // For theta component
        jacobian(2,0) = -1;
        jacobian(2,1) = -1;
        jacobian(2,2) = -1;
    }
    
    return jacobian;
}

std::map<std::string, utils::SE2Transform>
ForwardKinematics::computeFromLeftFoot(const RobotGeometry& geometry,
                                     const std::array<double, NUM_JOINTS>& joint_angles,
                                     double com_displacement) {
    // Similar to computeFromRightFoot but with reversed chain and negated angles
    // Implementation here...
    std::map<std::string, utils::SE2Transform> transforms;
    // ... (similar pattern to computeFromRightFoot but starting from left foot)
    return transforms;
}

std::map<std::string, utils::SE2Transform>
ForwardKinematics::computeFromTorso(const RobotGeometry& geometry,
                                  const std::array<double, NUM_JOINTS>& joint_angles,
                                  double com_displacement) {
    // Similar pattern but starting from torso
    // Implementation here...
    std::map<std::string, utils::SE2Transform> transforms;
    // ... (similar pattern but starting from torso frame)
    return transforms;
}

} // namespace kinematics
} // namespace biped_control