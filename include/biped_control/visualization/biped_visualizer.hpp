#pragma once

#include "biped_control/types.hpp"
#include "biped_control/utils/transforms.hpp"
#include <memory>
#include <array>
#include <map>
#include <string>

namespace biped_control {
namespace visualization {

/**
 * @brief Class to handle the visualization of the biped robot
 * Uses PIMPL idiom to hide visualization implementation details
 */
class BipedVisualizer {
public:
    BipedVisualizer();
    ~BipedVisualizer();

    /**
     * @brief Initialize the visualization window and plots
     * @return true if initialization successful
     */
    bool initialize();

    /**
     * @brief Update the visualization with new robot state
     * @param joint_transforms Current transforms of all joints
     * @param com Current center of mass position
     * @return true if update successful
     */
    bool update(const std::map<std::string, utils::SE2Transform>& joint_transforms,
                const Vector2d& com);
    
    /**
     * @brief Start real-time visualization mode
     * @param update_rate Update rate in Hz
     * @return true if started successfully
     */
    bool startRealtimeVisualization(double update_rate = 10.0);

    /**
     * @brief Stop real-time visualization
     */
    void stopRealtimeVisualization();
    
    /**
     * @brief Animate a trajectory of joint configurations
     * @param trajectory Vector of joint angle arrays
     * @param time_unit Time between frames in milliseconds
     * @return true if animation completed successfully
     */
    bool animateTrajectory(const std::vector<std::array<double, NUM_JOINTS>>& trajectory,
                          double time_unit);

    /**
     * @brief Check if robot is balanced in current configuration
     * @param com Current center of mass position
     * @param foot_width Width of the foot
     * @param stance Current stance
     * @return true if robot is balanced
     */
    bool checkBalance(const Vector2d& com, double foot_width, Stance stance) const;

private:
    struct PlotData {
        std::array<std::array<double, 2>, 2> link_points;  // Start and end points of link
        bool visible;
        // Additional plotting properties can be added here
    };

    struct Impl;
    std::unique_ptr<Impl> impl_;  // PIMPL idiom for implementation details

    // Internal plotting methods
    void plotLink(const utils::SE2Transform& start, 
                 const utils::SE2Transform& end,
                 const std::string& link_name);
    
    void plotFrame(const utils::SE2Transform& transform,
                  const std::string& frame_name,
                  double axis_length = 1.0);
    
    void plotPoint(const Vector2d& point,
                  const std::string& point_name,
                  const std::string& color = "green");

    // Setup methods
    void setupPlotWindow();
    void setupLinkPlotters();
    void setupFramePlotters();
};

} // namespace visualization
} // namespace biped_control