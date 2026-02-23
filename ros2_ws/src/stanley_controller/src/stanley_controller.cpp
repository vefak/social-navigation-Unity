#include <eigen3/Eigen/Dense>
#include <fstream>
#include <cmath>
#include <iostream>
#include <chrono>
#include "stanley_controller.h"
using namespace std;
using namespace Eigen;

Parameters::Parameters(const std::string &root_path)
{

    std::filesystem::path config_path = root_path + "/" + "config.json";
    std::ifstream file(config_path);
    nlohmann::json data = nlohmann::json::parse(file);

    l = data["robot"]["length"];                                          // robot lenght
    wheelsize = data["stanleyControllerParams"]["wheelsize"];             // wheelsize
    dred = data["stanleyControllerParams"]["dred"];                       // Look-ahead distance adjustment
    velk = data["stanleyControllerParams"]["velk"];                       // Velocity control gain (speed reduction)
    k = data["stanleyControllerParams"]["k"];                             // Stanley gain for lateral error.
    ksoft = data["stanleyControllerParams"]["ksoft"];                     // to avoid division by zero at low speeds
    maxDelta = data["stanleyControllerParams"]["maxDelta"];               // Max steering angle (rad)
    maxDeltaVel = data["stanleyControllerParams"]["maxDeltaVel"];         // Max angular velocity (rad/s)
    minSpeed = data["stanleyControllerParams"]["minSpeed"];               // Minimum speed threshold
    vx = data["stanleyControllerParams"]["vx"];                           // 	Nominal (cruise) linear velocity (m/s)
    sampling_rate_m = data["stanleyControllerParams"]["sampling_rate_m"]; // Trajectory resampling distance (in meters). Controls spacing of trajectory points
    goal_distance_threshold = data["stanleyControllerParams"]["goal_distance_threshold"];
    x0 = {
        data["robot"]["start"]["position"]["x"],
        data["robot"]["start"]["position"]["y"],
        data["robot"]["start"]["theta"]}; // Initial robot pose

    xc = {
        data["stanleyControllerParams"]["xc"]["x"],
        data["stanleyControllerParams"]["xc"]["y"]}; // State vector (lateral/heading error tracking)
}

StanleyController::StanleyController(Parameters &parameters)
    : Node("stanley_controller"), params_(parameters), current_segment_index_(0)
{
    RCLCPP_INFO(this->get_logger(), " [Stanley Controller] Stanley Controller is enabled");

    subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
        "/robot_pose", 10, std::bind(&StanleyController::pose_callback, this, std::placeholders::_1));

    // Subscribe to trajectory points
    trajectory_subscription_ = this->create_subscription<unitycustommsg::msg::Point2DArray>(
        "/global_trajectory", 10, std::bind(&StanleyController::trajectory_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    x_traj = Vector3d(params_.x0[0], params_.x0[1], params_.x0[2]);

    // referenceTrajectoryFromFile(trajectory_file);

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(50), // 10 ms interval = 100 Hz
        std::bind(&StanleyController::publish_velocity_command, this));
}

void StanleyController::publish_velocity_command()
{
    if (referenceTraj.rows() == 0)
    {
        twist_msg_.linear.x = 0.0;
        twist_msg_.angular.z = 0.0;
    }
    publisher_->publish(twist_msg_);
    // RCLCPP_INFO(this->get_logger(), "Publishing velocity command: linear.x=%.5f, angular.z=%.5f", twist_msg_.linear.x, twist_msg_.angular.z);
}

bool is_static_trajectory(const Eigen::MatrixXd &traj, double threshold = 1e-2)
{
    if (traj.rows() < 2)
        return true; // One or zero points = static

    Eigen::Vector2d ref = traj.row(0).head<2>();
    for (int i = 1; i < traj.rows(); ++i)
    {
        if ((traj.row(i).head<2>().transpose() - ref).norm() > threshold)
        {
            return false;
        }
    }
    return true;
}
void StanleyController::pose_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Robot's position is updated.");
    if (referenceTraj.rows() == 0)
    {
        // RCLCPP_WARN(this->get_logger(), "No reference trajectory to follow. Publishing stop command.");
        twist_msg_.linear.x = 0.0;
        twist_msg_.angular.z = 0.0;
        publisher_->publish(twist_msg_);
        return;
    }
    // Start measuring time for the control loop
    auto loop_start = std::chrono::high_resolution_clock::now();

    // Extract robot's position and orientation from TransformStamped message
    // WARNING: Robot pose orientation is published as Euler angles (not quaternion).
    // We treat rotation.z as yaw. Because Unity and ROS2 have diff coordination system
    double x = msg->transform.translation.x;
    double y = msg->transform.translation.z;
    double orientation = msg->transform.rotation.y * -1 + M_PI / 2;

    if (goal_reached_)
    {
        twist_msg_.linear.x = 0.0;
        twist_msg_.angular.z = 0.0;
        publisher_->publish(twist_msg_);
        return;
    }

    if (referenceTraj.rows() > 0)
    {
        Eigen::Vector2d final_point = referenceTraj.bottomRows(1).leftCols(2).transpose();
        Eigen::Vector2d robot_pos(x, y);
        double distance_to_goal = (final_point - robot_pos).norm();
        if (distance_to_goal < params_.goal_distance_threshold)
        {
            RCLCPP_INFO(this->get_logger(), "Reached goal. Stopping robot.");
            goal_reached_ = true;
            twist_msg_.linear.x = 0.0;
            twist_msg_.angular.z = 0.0;
            publisher_->publish(twist_msg_);
            return;
        }
    }

    x_traj = Vector3d(x, y, orientation);
    // Compute control error
    auto [eForward, nextTargetPosForw, d_trajForward, e_fbForward] = controlError(x_traj, referenceTraj, 1);
    auto [eBackward, nextTargetPosBack, d_trajBackward, e_fbBackward] = controlError(x_traj, referenceTraj, -1);

    // Control logic
    auto [long_Velocity, ang_Velocity, xc] = control(eForward[0], e_fbForward, d_trajForward,
                                                     eBackward[0], e_fbBackward, d_trajBackward,
                                                     params_.xc);
    // Publish control command

    twist_msg_.linear.x = long_Velocity;
    twist_msg_.angular.z = ang_Velocity;
    publisher_->publish(twist_msg_);
    // RCLCPP_INFO(this->get_logger(), " [Stanley Controller] Control Output - linear.x: %.3f, angular.z: %.3f", long_Velocity, ang_Velocity);

    // End measuring time for the control loop
    auto loop_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> loop_duration = loop_end - loop_start;
    // RCLCPP_INFO(this->get_logger(), "[Stanley Controller] Control Loop Time: %.6f seconds", loop_duration.count());
}

void StanleyController::trajectory_callback(const unitycustommsg::msg::Point2DArray::SharedPtr msg)
{
    goal_reached_ = false;
    RCLCPP_INFO(this->get_logger(), "[Stanley Controller] Received New Trajectory ");
    referenceTraj.resize(0, 0);
    std::vector<Eigen::Vector2d> points;
    for (const auto &point : msg->points)
    {
        points.push_back(Eigen::Vector2d(point.x, point.y));
    }
    Eigen::MatrixXd points_matrix(points.size(), 2);
    for (size_t i = 0; i < points.size(); ++i)
    {
        points_matrix(i, 0) = points[i].x(); // Set the x-coordinate
        points_matrix(i, 1) = points[i].y(); // Set the y-coordinate
    }
    auto discretized_points = discretize_points(points_matrix);
    if (discretized_points.size() < 4)
    {
        RCLCPP_WARN(this->get_logger(), " [Stanley Controller] Discretized trajectory too short. Ignoring.");
        referenceTraj.resize(0, 0);
        return;
    }
    if (is_static_trajectory(points_matrix))
    {
        RCLCPP_WARN(this->get_logger(), "[Stanley Controller] Received static or degenerate trajectory. Ignoring.");
        referenceTraj.resize(0, 0);
        return;
    }
    auto full_traj = add_orientation_rad_to_points_cubic(discretized_points, 0.5);
    referenceTraj = full_traj;
}

void StanleyController::referenceTrajectoryFromFile(const std::string &fileName)
{
    ifstream file(fileName);
    vector<Vector2d> points;

    double x, y, temp;
    while (file >> x >> y >> temp)
    {
        points.push_back(Vector2d(x, y)); // Add x, y as Eigen::Vector2d into the points vector
    }

    // Initialize a matrix with the size of points (rows) and 2 columns
    MatrixXd points_matrix(points.size(), 2);

    // Loop through the points and assign the x and y values to the matrix
    for (size_t i = 0; i < points.size(); ++i)
    {
        points_matrix(i, 0) = points[i].x(); // Set the x-coordinate
        points_matrix(i, 1) = points[i].y(); // Set the y-coordinate
    }

    // Now pass the matrix to the discretize_points function
    auto discretized_points = discretize_points(points_matrix);

    // Continue with your existing logic
    referenceTraj = add_orientation_rad_to_points_cubic(discretized_points, 0.5);
}

vector<Vector2d> StanleyController::discretize_points(const MatrixXd &trajectory)
{
    vector<Vector2d> discretized_points;
    // Check if matrix has rows
    if (trajectory.rows() == 0)
    {
        std::cerr << "Error: Empty trajectory matrix!" << std::endl;
        return discretized_points;
    }
    Vector2d last_point = trajectory.row(0);
    discretized_points.push_back(last_point);

    for (int i = 1; i < trajectory.rows(); ++i)
    {
        Vector2d current_point = trajectory.row(i);
        double distance = (current_point - last_point).norm();

        if (distance >= params_.sampling_rate_m)
        {
            discretized_points.push_back(current_point);
            last_point = current_point;
        }
    }
    return discretized_points;
}

double StanleyController::compute_orientation_usingCardinalSplines(const MatrixXd &C, double t, double tau)
{
    double s = 0.5 * (1 - tau);
    Matrix4d M;
    M << -s, 2 - s, s - 2, s,
        2 * s, s - 3, 3 - 2 * s, -s,
        -s, 0, s, 0,
        0, 1, 0, 0;

    Vector4d dT(3 * t * t, 2 * t, 1, 0);
    Vector2d dP = dT.transpose() * M * C;

    return atan2(dP[1], dP[0]);
}
MatrixXd StanleyController::add_orientation_rad_to_points_cubic(const vector<Vector2d> &discretized_points, double tau)
{

    MatrixXd points_with_orientation(discretized_points.size(), 3);
    // Handle from 2nd to 2nd last points
    for (size_t i = 1; i < discretized_points.size() - 2; ++i)
    {
        MatrixXd C(4, 2);
        for (int j = 0; j < 4; ++j)
        {
            C.row(j) = discretized_points[i - 1 + j];
        }
        double orientation_rad = compute_orientation_usingCardinalSplines(C, 0, tau);

        // Assign the row properly
        points_with_orientation.row(i) = Eigen::RowVector3d(discretized_points[i].x(), discretized_points[i].y(), orientation_rad);
    }

    // Handle the first point
    MatrixXd C(4, 2);
    C.row(1) = discretized_points[0];
    C.row(2) = discretized_points[1];
    C.row(3) = discretized_points[2];
    C.row(0) = 2 * C.row(1) - C.row(2); // Mirror the second point to get a virtual point
    double orientation_rad = compute_orientation_usingCardinalSplines(C, 0, tau);
    points_with_orientation.row(0) = Eigen::RowVector3d(discretized_points[0].x(), discretized_points[0].y(), orientation_rad);

    // Handle the last point
    C.row(0) = discretized_points[discretized_points.size() - 3];
    C.row(1) = discretized_points[discretized_points.size() - 2];
    C.row(2) = discretized_points[discretized_points.size() - 1];
    C.row(3) = 2 * C.row(2) - C.row(1);                                    // Mirror the second last point to get a virtual point
    orientation_rad = compute_orientation_usingCardinalSplines(C, 1, tau); // t=1 for the last point
    points_with_orientation.row(discretized_points.size() - 1) = Eigen::RowVector3d(discretized_points.back().x(), discretized_points.back().y(), orientation_rad);

    // Handle the second-to-last point
    orientation_rad = compute_orientation_usingCardinalSplines(C, 0, tau); // t=0 for the second-to-last point
    points_with_orientation.row(discretized_points.size() - 2) = Eigen::RowVector3d(discretized_points[discretized_points.size() - 2].x(), discretized_points[discretized_points.size() - 2].y(), orientation_rad);

    return points_with_orientation;
}

std::tuple<Eigen::Vector2d, double, double> StanleyController::getTargetPoint(const Eigen::Vector3d &x, const Eigen::MatrixXd &r)
{
    // Ensure matrix has at least 3 columns (x, y, orientation)

    Eigen::Vector2d next_target_position = Eigen::Vector2d::Zero(); // Placeholder for the closest point position
    double next_target_orientation = 0.0;                           // Placeholder for orientation
    double next_target_curvature = 0.0;                             // Placeholder for curvature

    double minDist = std::numeric_limits<double>::infinity(); // Set initial min distance as infinity

    // Ensure safe loop bounds
    for (int i = 0; i < r.rows() - 1; ++i)
    {
        // Extract points A and B safely
        Eigen::Vector2d A = r.row(i).head<2>();     // First 2 columns of row i (x, y)
        Eigen::Vector2d B = r.row(i + 1).head<2>(); // First 2 columns of row i+1 (x, y)
        double psi_A = r(i, 2);                     // Orientation at point A
        double psi_B = r(i + 1, 2);                 // Orientation at point B

        Eigen::Vector2d AB = B - A;                // Segment AB
        double d_AB = AB.norm();                   // Length of segment AB
        Eigen::Vector2d n_AB = AB / d_AB;          // Normalize AB
        double t_AB = (x.head<2>() - A).dot(n_AB); // Projection of x onto AB

        // Check if the projected point lies on the segment or outside
        Eigen::Vector2d target_position;
        double target_orientation;
        if (t_AB >= 0 && t_AB <= d_AB)
        {
            target_position = A + t_AB * n_AB; // Point on segment
            // target_orientation = psi_A * (1.0 - t_AB / d_AB) + psi_B * (t_AB / d_AB);
            target_orientation = psi_A + (t_AB / d_AB) * std::atan2(
                                                             std::sin(psi_B - psi_A), std::cos(psi_B - psi_A));
        }
        else if (t_AB < 0)
        {
            target_position = A;
            target_orientation = psi_A;
        }
        else
        {
            target_position = B;
            target_orientation = psi_B;
        }

        double distance = (target_position - x.head<2>()).norm(); // Distance to the trajectory

        if (distance < minDist)
        {
            minDist = distance;
            next_target_position = target_position;
            next_target_orientation = target_orientation;
            // next_target_curvature = (psi_B - psi_A) / d_AB;
            if (d_AB > 1e-3) // Safer against identical or empty trajectories
                next_target_curvature = (psi_B - psi_A) / d_AB;
            else
                next_target_curvature = 0.0;
        }
    }

    return std::make_tuple(next_target_position, next_target_orientation, next_target_curvature);
}

std::tuple<double, double, std::vector<double>> StanleyController::control(double eForw, double e_fbForw, double dForw,
                                                                           double eBack, double e_fbBack, double dBack,
                                                                           std::vector<double> xc)
{
    // Initialize velocity and angular velocity variables
    double long_Velocity = params_.vx; // Default to forward velocity
    double ang_Velocity = 0.0;         // Initialize angular velocity

    // Handle switching between forward and backward movement based on error feedback
    // if (long_Velocity > 0 && e_fbBack < params_.l * 1.5 && e_fbForw > params_.l * 2.5)
    if (long_Velocity > 0 && e_fbBack < params_.l * 0.5 && e_fbForw > params_.l * 0.5)
    {
        long_Velocity = -std::abs(long_Velocity); // Switch to reverse
    }
    // if (long_Velocity < 0 && e_fbBack > params_.l * 1.5 && e_fbForw < -params_.l * 2.5)
    if (long_Velocity < 0 && e_fbBack > params_.l * 0.5 && e_fbForw < -params_.l * 0.5)
    {
        long_Velocity = std::abs(long_Velocity); // Switch to forward
    }

    // Select which direction's error to use (forward or backward)
    double e_fa, theta_e, d, e_fb;
    if (long_Velocity < 0)
    { // Reverse case
        e_fa = -eBack;
        // theta_e = M_PI - eBack;
        theta_e = eBack + M_PI;
        theta_e = std::atan2(std::sin(theta_e), std::cos(theta_e));
        d = dBack;
        e_fb = e_fbBack;
    }
    else
    { // Forward case
        e_fa = eForw;
        theta_e = eForw;
        d = dForw;
        e_fb = e_fbForw;
    }

    // Calculate the speed reduction factor based on distance
    double speedReductionFactor = (1.0 - std::exp(-std::min(d - (params_.dred * 0.05), params_.dred) * params_.velk));
    // speedReductionFactor = std::clamp(speedReductionFactor, 0.3, 1.0);

    long_Velocity *= speedReductionFactor; // Apply speed reduction

    if (std::abs(long_Velocity) > params_.minSpeed)
    {
        // Stanley control law: Adjust steering angle based on lateral error (e_fa) and heading error (theta_e)
        double steering_angle = theta_e + std::atan2(params_.k * e_fa, std::abs(long_Velocity) + params_.ksoft);
        // Saturate the steering angle
        steering_angle = std::min(std::max(steering_angle, -params_.maxDelta), params_.maxDelta);

        // Calculate angular velocity based on steering angle and longitudinal velocity
        ang_Velocity = (1.0 / params_.l) * long_Velocity * std::tan(steering_angle);
        // Saturate angular velocity
        ang_Velocity = std::min(std::max(ang_Velocity, -params_.maxDeltaVel), params_.maxDeltaVel);
    }
    else
    {
        // If speed is below the threshold, stop the vehicle
        long_Velocity = 0.0;
        ang_Velocity = 0.0;
    }

    // Return the control outputs: longitudinal velocity, angular velocity, and updated state vector xc
    return std::make_tuple(long_Velocity, ang_Velocity, xc);
}

std::tuple<Eigen::Vector2d, Eigen::Vector2d, double, double> StanleyController::controlError(const Eigen::Vector3d &x, const Eigen::MatrixXd &r, int direction)
{
    // Extract the current state (position and orientation) of the robot
    double x_ego = x[0];
    double y_ego = x[1];
    double theta = x[2];
    //  Compute the virtual front axle point based on the robot's state and direction
    Eigen::Vector3d x_Front;
    x_Front[0] = x_ego + params_.l * std::cos(theta) * direction;
    x_Front[1] = y_ego + params_.l * std::sin(theta) * direction;
    x_Front[2] = theta;

    // std::stringstream r_stream;
    // r_stream << "r = \n"
    //          << r;
    // RCLCPP_INFO(this->get_logger(), "%s", r_stream.str().c_str());

    // Get the closest point on the reference trajectory (r) to the front axle point
    Eigen::Vector2d nextTargetPos;
    double nextTargetOr, nextTargetCur;
    std::tie(nextTargetPos, nextTargetOr, nextTargetCur) = getTargetPoint(x_Front, r);

    // Adjust orientation for driving in reverse
    if (direction < 0)
    {
        nextTargetOr += M_PI;
    }

    // Calculate the lateral error (e_fa) between the front axle point and the trajectory
    double n1 = std::sin(nextTargetOr);  // Normal vector's x-component
    double n2 = -std::cos(nextTargetOr); // Normal vector's y-component
    double a = n1;
    double b = n2;
    double c = -n1 * nextTargetPos[0] - n2 * nextTargetPos[1];

    // Compute lateral error using line equation (ax + by + c = 0)
    double e_fa = a * x_Front[0] + b * x_Front[1] + c;

    // Calculate the longitudinal error (e_fb) in the direction of the trajectory
    n1 = std::cos(nextTargetOr); // Tangent vector's x-component
    n2 = std::sin(nextTargetOr); // Tangent vector's y-component
    a = n1;
    b = n2;
    c = -n1 * nextTargetPos[0] - n2 * nextTargetPos[1];

    // Longitudinal error in the direction of the trajectory
    double e_fb = a * x_ego + b * y_ego + c;

    // #
    theta = std::atan2(std::sin(theta), std::cos(theta));
    nextTargetOr = std::atan2(std::sin(nextTargetOr), std::cos(nextTargetOr));

    // Compute the heading (orientation) error between the robot's orientation and the reference trajectory
    double theta_e = nextTargetOr - theta;
    theta_e = std::atan2(std::sin(theta_e), std::cos(theta_e)); // Normalize the angle

    // Calculate the Euclidean distance from the robot's current position to the closest point on the trajectory
    // double d = (x_Front.head<2>() - nextTargetPos).norm();

    // double d = (x_ego-nextTargetPos[0], y_ego - nextTargetPos[1]).norm();

    Eigen::Vector2d diff(x_ego - nextTargetPos[0], y_ego - nextTargetPos[1]);
    double d = diff.norm();

    // Return the computed control errors and the distance to the target
    return std::make_tuple(Eigen::Vector2d(e_fa, theta_e), nextTargetPos, d, e_fb);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    std::filesystem::path root_path = std::filesystem::current_path() / "../floor-segmentation/json";
    Parameters parameters(root_path);
    auto node = std::make_shared<StanleyController>(parameters);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
