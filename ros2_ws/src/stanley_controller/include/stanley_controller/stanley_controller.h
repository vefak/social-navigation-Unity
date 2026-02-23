#ifndef STANLEY_CONTROLLER_H
#define STANLEY_CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include "unitycustommsg/msg/point2_d_array.hpp"
#include "unitycustommsg/msg/point2_d.hpp"
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>

class Parameters
{
public:
    // Parameters();
    Parameters(const std::string &root_path);

    // Physical parameters
    double l;
    double wheelsize;
    double dred;
    double velk;
    double k;
    double ksoft;
    double dt;
    double maxDelta;
    double maxDeltaVel;
    double minSpeed;
    double tt;
    double vx;
    std::vector<double> xc;
    std::vector<double> x0;
    double sampling_rate_m;
    double goal_distance_threshold;
};

class StanleyController : public rclcpp::Node
{
public:
    StanleyController(Parameters &parameters);
    bool goal_reached_ = false;

private:
    // ROS Callbacks and Publishers
    void pose_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg);
    void trajectory_callback(const unitycustommsg::msg::Point2DArray::SharedPtr msg);

    rclcpp::Subscription<unitycustommsg::msg::Point2DArray>::SharedPtr trajectory_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    // Timer to control the publishing frequency (10 Hz)
    rclcpp::TimerBase::SharedPtr timer_; // Declare the timer

    // Method to publish velocity commands
    void publish_velocity_command(); // Declare the publishing method

    // Trajectory
    std::vector<Eigen::Vector2d> referenceTraj_; // Store the trajectory points

    // Parameters
    Parameters params_;
    Eigen::Vector3d x_traj;
    geometry_msgs::msg::Twist twist_msg_;
    Eigen::MatrixXd referenceTraj;
    int current_segment_index_;

    // Helper Methods
    void referenceTrajectoryFromFile(const std::string &fileName);
    std::vector<Eigen::Vector2d> discretize_points(const Eigen::MatrixXd &trajectory);
    double compute_orientation_usingCardinalSplines(const Eigen::MatrixXd &C, double t, double tau = 0);
    Eigen::MatrixXd add_orientation_rad_to_points_cubic(const std::vector<Eigen::Vector2d> &discretized_points, double tau = 0);
    Eigen::MatrixXd compute_tangents_to_plot(double ds, const Eigen::MatrixXd &points_with_orientation);

    std::tuple<Eigen::Vector2d, double, double> getTargetPoint(const Eigen::Vector3d &x, const Eigen::MatrixXd &r);

    std::tuple<double, double, std::vector<double>> control(double eForw, double e_fbForw, double dForw,
                                                            double eBack, double e_fbBack, double dBack,
                                                            std::vector<double> xc);

    // Fix the return type to match with cpp file: 4-tuple (Vector2d, Vector2d, double, double)
    std::tuple<Eigen::Vector2d, Eigen::Vector2d, double, double> controlError(const Eigen::Vector3d &x, const Eigen::MatrixXd &r, int direction);
};

#endif // STANLEY_CONTROLLER_H
