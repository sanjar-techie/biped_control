#include "biped_control/visualization/biped_visualizer.hpp"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>

namespace biped_control {
namespace visualization {

// PIMPL implementation
struct BipedVisualizer::Impl {
    ros::NodeHandle nh;
    ros::Publisher marker_pub;
    ros::Publisher marker_array_pub;
    bool is_initialized{false};
    double update_rate{10.0};
    int marker_id{0};

    // Store markers for efficient updates
    std::map<std::string, visualization_msgs::Marker> link_markers;
    std::map<std::string, visualization_msgs::Marker> frame_markers;
    visualization_msgs::Marker com_marker;
    
    std::string base_frame{"world"};
};

BipedVisualizer::BipedVisualizer() : impl_(std::make_unique<Impl>()) {
}

BipedVisualizer::~BipedVisualizer() = default;

bool BipedVisualizer::initialize() {
    if (impl_->is_initialized) {
        return true;
    }

    // Set up ROS publishers
    impl_->marker_pub = impl_->nh.advertise<visualization_msgs::Marker>(
        "biped_visualization/markers", 10);
    impl_->marker_array_pub = impl_->nh.advertise<visualization_msgs::MarkerArray>(
        "biped_visualization/marker_array", 10);

    // Initialize COM marker
    impl_->com_marker.header.frame_id = impl_->base_frame;
    impl_->com_marker.ns = "com";
    impl_->com_marker.id = impl_->marker_id++;
    impl_->com_marker.type = visualization_msgs::Marker::SPHERE;
    impl_->com_marker.action = visualization_msgs::Marker::ADD;
    impl_->com_marker.scale.x = 0.02;  // 2cm diameter sphere
    impl_->com_marker.scale.y = 0.02;
    impl_->com_marker.scale.z = 0.02;
    impl_->com_marker.color.r = 0.0;
    impl_->com_marker.color.g = 1.0;
    impl_->com_marker.color.b = 0.0;
    impl_->com_marker.color.a = 1.0;

    impl_->is_initialized = true;
    return true;
}

bool BipedVisualizer::update(const std::map<std::string, utils::SE2Transform>& joint_transforms,
                            const Vector2d& com) {
    if (!impl_->is_initialized) {
        ROS_ERROR("BipedVisualizer not initialized!");
        return false;
    }

    visualization_msgs::MarkerArray marker_array;
    ros::Time current_time = ros::Time::now();

    // Update links
    for (const auto& [name, transform] : joint_transforms) {
        // Skip the terminal frames
        if (name == "joint_1" || name == "joint_6") continue;

        // Create or update link marker
        auto& marker = impl_->link_markers[name];
        marker.header.frame_id = impl_->base_frame;
        marker.header.stamp = current_time;
        marker.ns = "links";
        marker.id = impl_->marker_id++;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;

        // Set position
        marker.pose.position.x = transform.translation.x();
        marker.pose.position.y = transform.translation.y();
        marker.pose.position.z = 0;

        // Set orientation
        tf2::Quaternion q;
        q.setRPY(0, 0, transform.rotation);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        // Set scale
        marker.scale.x = 0.02;  // 2cm diameter
        marker.scale.y = 0.02;
        marker.scale.z = 0.1;   // 10cm length

        // Set color
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker_array.markers.push_back(marker);
    }

    // Update frames (coordinate axes)
    for (const auto& [name, transform] : joint_transforms) {
        if (name != Frames::LEFT_FOOT && 
            name != Frames::RIGHT_FOOT && 
            name != Frames::TORSO) continue;

        // X axis (red)
        auto& x_marker = impl_->frame_markers[name + "_x"];
        x_marker.header.frame_id = impl_->base_frame;
        x_marker.header.stamp = current_time;
        x_marker.ns = "frames";
        x_marker.id = impl_->marker_id++;
        x_marker.type = visualization_msgs::Marker::ARROW;
        x_marker.action = visualization_msgs::Marker::ADD;

        x_marker.points.resize(2);
        x_marker.points[0].x = transform.translation.x();
        x_marker.points[0].y = transform.translation.y();
        x_marker.points[0].z = 0;
        x_marker.points[1].x = transform.translation.x() + 0.05 * cos(transform.rotation);
        x_marker.points[1].y = transform.translation.y() + 0.05 * sin(transform.rotation);
        x_marker.points[1].z = 0;

        x_marker.scale.x = 0.01;  // shaft diameter
        x_marker.scale.y = 0.02;  // head diameter
        x_marker.scale.z = 0.0;
        x_marker.color.r = 1.0;
        x_marker.color.g = 0.0;
        x_marker.color.b = 0.0;
        x_marker.color.a = 1.0;

        marker_array.markers.push_back(x_marker);

        // Y axis (green)
        auto& y_marker = impl_->frame_markers[name + "_y"];
        y_marker = x_marker;
        y_marker.id = impl_->marker_id++;
        y_marker.points[1].x = transform.translation.x() - 
            0.05 * sin(transform.rotation);
        y_marker.points[1].y = transform.translation.y() + 
            0.05 * cos(transform.rotation);
        y_marker.color.r = 0.0;
        y_marker.color.g = 1.0;
        y_marker.color.b = 0.0;

        marker_array.markers.push_back(y_marker);
    }

    // Update COM
    impl_->com_marker.header.stamp = current_time;
    impl_->com_marker.pose.position.x = com.x();
    impl_->com_marker.pose.position.y = com.y();
    impl_->com_marker.pose.position.z = 0;
    marker_array.markers.push_back(impl_->com_marker);

    // Publish all markers
    impl_->marker_array_pub.publish(marker_array);
    return true;
}

bool BipedVisualizer::startRealtimeVisualization(double update_rate) {
    if (!impl_->is_initialized) {
        return false;
    }
    impl_->update_rate = update_rate;
    return true;
}

void BipedVisualizer::stopRealtimeVisualization() {
    // Clear all markers
    visualization_msgs::MarkerArray marker_array;
    for (auto& [name, marker] : impl_->link_markers) {
        marker.action = visualization_msgs::Marker::DELETE;
        marker_array.markers.push_back(marker);
    }
    for (auto& [name, marker] : impl_->frame_markers) {
        marker.action = visualization_msgs::Marker::DELETE;
        marker_array.markers.push_back(marker);
    }
    impl_->com_marker.action = visualization_msgs::Marker::DELETE;
    marker_array.markers.push_back(impl_->com_marker);
    
    impl_->marker_array_pub.publish(marker_array);
}

bool BipedVisualizer::checkBalance(const Vector2d& com, 
                                 double foot_width, 
                                 Stance stance) const {
    if (stance == Stance::TORSO) {
        return true;  // No balance check needed for torso stance
    }
    
    return (com.x() > -0.5 * foot_width && com.x() < 0.5 * foot_width);
}

} // namespace visualization
} // namespace biped_control