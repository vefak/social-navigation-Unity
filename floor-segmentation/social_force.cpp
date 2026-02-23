#include "social_force.h"

#include <fstream>
#include <iterator>
#include <limits>
#include <vector>

#include "geometry.h"
#include "planning.h"
#include "transforms.h"
#include "util.h"

namespace social_force {

void Trajectory::save_to_file(const std::filesystem::path& save_path) const {
    std::ofstream os(save_path);

    // If angle information is not provided, output (x, y) at each timestamp
    if (angles.empty()) {
        for (size_t i = 0; i < path.size(); ++i) {
            size_t j = std::min(i + 1, path.size());
            const double dx = path[j].x - path[i].x;
            const double dy = path[j].y - path[i].y;
            double angle = atan2(dy, dx);
            os << path[i].x << " " << path[i].y << " " << angle << " " << v_linear[i].x << " " << v_linear[i].y << " "
               << v_angular[i] << std::endl;
        }

        return;
    }

    // Otherwise plot (x, y, theta) at each timestamp
    assert(path.size() == angles.size());

    for (size_t i = 0; i < path.size(); ++i) {
        os << path[i].x << " " << path[i].y << " " << angles[i] << " " << v_linear[i].x << " " << v_linear[i].y << " "
           << v_angular[i] << std::endl;
    }
}

void Trajectory::adjust_coordinates(const transforms::CoordinateTransform& transform) {
    for (auto& point : path) point = transform.point_transform(point);
    for (auto& velocity : v_linear) velocity = transform.velocity_transform(velocity);
    for (auto& angle : angles) angle = transform.angle_transform(angle);
    for (auto& omega : v_angular) omega = transform.angular_velocity_transform(omega);
}

Trajectory Trajectory::erase_prefix(size_t steps) {
    Trajectory reduced_traj = *this;

    if (path.size() < steps) {
        reduced_traj.path = std::vector<geometry::Point>(path.begin() + steps, path.end());
    }

    if (v_linear.size() < steps) {
        reduced_traj.v_linear = std::vector<geometry::Point>(v_linear.begin() + steps, v_linear.end());
    }

    if (angles.size() < steps) {
        reduced_traj.angles = std::vector<double>(angles.begin() + steps, angles.end());
    }

    if (v_angular.size() < steps) {
        reduced_traj.v_angular = std::vector<double>(v_angular.begin() + steps, v_angular.end());
    }

    return reduced_traj;
}

geometry::Point LocalPlanner::project_onto_old_trajectory(geometry::Point p) {
    if (old_trajectory.empty()) {
        return p;
    }
    return planning::nearest_path_point(old_trajectory, p);
}

planning::Path DubinsInterpolator::dubins_XSX(geometry::Point& start, geometry::Point& goal, double theta_start,
                                              double theta_goal, double turning_radius, int total_samples,
                                              geometry::ROTATION rot_dir) {
    geometry::Point center_start =
        start + turning_radius * geometry::direction_vector(theta_start + rot_dir * M_PI / 2);
    geometry::Point center_goal = goal + turning_radius * geometry::direction_vector(theta_goal + rot_dir * M_PI / 2);

    geometry::Point center_vector = center_goal - center_start;
    geometry::Point parallel_offset = turning_radius * center_vector.rotate(-rot_dir * M_PI / 2).normalize();

    // Points that the XSX path goes through
    geometry::Point anchor_start = center_start + parallel_offset;
    geometry::Point anchor_goal = center_goal + parallel_offset;

    // The pieces of the path
    geometry::Arc arc_start(center_start, turning_radius, start, anchor_start, rot_dir);
    geometry::LineSegment connector(anchor_start, anchor_goal);
    geometry::Arc arc_goal(center_goal, turning_radius, anchor_goal, goal, rot_dir);

    return combine_parts_XSY(arc_start, connector, arc_goal, total_samples);  // Same pattern for XSX and XSY
}

planning::Path DubinsInterpolator::dubins_RSR(geometry::Point& start, geometry::Point& goal, double theta_start,
                                              double theta_goal, double turning_radius, int total_samples) {
    return dubins_XSX(start, goal, theta_start, theta_goal, turning_radius, total_samples, geometry::ROTATION::CW);
}

planning::Path DubinsInterpolator::dubins_LSL(geometry::Point& start, geometry::Point& goal, double theta_start,
                                              double theta_goal, double turning_radius, int total_samples) {
    return dubins_XSX(start, goal, theta_start, theta_goal, turning_radius, total_samples, geometry::ROTATION::CCW);
}

planning::Path DubinsInterpolator::dubins_XYX(geometry::Point& start, geometry::Point& goal, double theta_start,
                                              double theta_goal, double turning_radius, int total_samples,
                                              geometry::ROTATION rot_dir) {
    geometry::Point center_start =
        start + turning_radius * geometry::direction_vector(theta_start + rot_dir * M_PI / 2);
    geometry::Point center_goal = goal + turning_radius * geometry::direction_vector(theta_goal + rot_dir * M_PI / 2);

    geometry::Point center_vector = center_goal - center_start;

    // Check path validity
    if (center_vector.norm() > 4 * turning_radius) {
        planning::Path path;
        path.valid = false;
        return path;
    }

    // Compute connector circle arc length (in radians) via cosine theorem
    double c = center_vector.norm();
    double a = 2 * turning_radius;
    double cos_gamma = (2 * a * a - c * c) / (2 * a * a);
    double gamma = acos(cos_gamma);
    double alpha = (M_PI - gamma) / 2;  // Angle between center vectors

    // Find center of transition circle
    geometry::Point center_aux = center_start + 2 * turning_radius * center_vector.rotate(rot_dir * alpha).normalize();

    // Points that the XYX path goes through
    geometry::Point anchor_start = (center_start + center_aux) / 2;
    geometry::Point anchor_goal = (center_aux + center_goal) / 2;

    // The pieces of the path
    geometry::Arc arc_start(center_start, turning_radius, start, anchor_start, rot_dir);
    geometry::Arc arc_connector(center_aux, turning_radius, anchor_start, anchor_goal,
                                static_cast<geometry::ROTATION>(-rot_dir));  // Opposite direction
    geometry::Arc arc_goal(center_goal, turning_radius, anchor_goal, goal, rot_dir);

    return combine_parts_XYX(arc_start, arc_connector, arc_goal, total_samples);
}

planning::Path DubinsInterpolator::dubins_LRL(geometry::Point& start, geometry::Point& goal, double theta_start,
                                              double theta_goal, double turning_radius, int total_samples) {
    return dubins_XYX(start, goal, theta_start, theta_goal, turning_radius, total_samples, geometry::ROTATION::CW);
}

planning::Path DubinsInterpolator::dubins_RLR(geometry::Point& start, geometry::Point& goal, double theta_start,
                                              double theta_goal, double turning_radius, int total_samples) {
    return dubins_XYX(start, goal, theta_start, theta_goal, turning_radius, total_samples, geometry::ROTATION::CCW);
}

planning::Path DubinsInterpolator::combine_parts_XYX(geometry::Arc arc_start, geometry::Arc arc_connector,
                                                     geometry::Arc arc_goal, int total_samples) {
    // Select number of samples depending on arc lengths
    double num_samples[3];
    double total_length = arc_start.length() + arc_goal.length() + arc_connector.length();
    num_samples[0] = static_cast<int>(total_samples * arc_start.length() / total_length);
    num_samples[1] = static_cast<int>(total_samples * arc_connector.length() / total_length);
    num_samples[2] = total_samples - num_samples[0] - num_samples[1];

    // Generate points
    std::vector<double> theta_arc_start, theta_connector, theta_arc_goal;
    std::vector<geometry::Point> points_arc_start, points_connector, points_arc_goal;

    points_arc_start = arc_start.sample(num_samples[0]);
    points_connector = arc_connector.sample(num_samples[1]);
    points_arc_goal = arc_goal.sample(num_samples[2]);

    theta_arc_start = arc_start.sample_tangent_angle(num_samples[0]);
    theta_connector = arc_connector.sample_tangent_angle(num_samples[1]);
    theta_arc_goal = arc_goal.sample_tangent_angle(num_samples[2]);

    planning::Path path;

    path.points = points_arc_start;
    path.points.insert(path.points.end(), points_connector.begin(), points_connector.end());
    path.points.insert(path.points.end(), points_arc_goal.begin(), points_arc_goal.end());

    path.angles = theta_arc_start;
    path.angles.insert(path.angles.end(), theta_connector.begin(), theta_connector.end());
    path.angles.insert(path.angles.end(), theta_arc_goal.begin(), theta_arc_goal.end());

    path.valid = is_collision_free(path);
    return path;
}

planning::Path DubinsInterpolator::dubins_XSY(geometry::Point& start, geometry::Point& goal, double theta_start,
                                              double theta_goal, double turning_radius, int total_samples,
                                              geometry::ROTATION rot_dir) {
    geometry::Point center_start =
        start + turning_radius * geometry::direction_vector(theta_start + rot_dir * M_PI / 2);
    geometry::Point center_goal = goal + turning_radius * geometry::direction_vector(theta_goal - rot_dir * M_PI / 2);

    geometry::Point center_vector = center_goal - center_start;

    // Check path validity
    if (center_vector.norm() < 2 * turning_radius) {
        planning::Path path;
        path.valid = false;
        return path;
    }

    // Find the length of the connector line segment using the Pythagorean theorem
    double len_connector = sqrt(center_vector.norm() * center_vector.norm() - 4 * turning_radius * turning_radius);
    double alpha = cos(len_connector / center_vector.norm());  // Angle between connector and center vector

    // Construct the connector
    geometry::Point connector_vector = len_connector * center_vector.rotate(rot_dir * alpha).normalize();

    // Points that the XSY path goes through
    geometry::Point anchor_start =
        center_start + turning_radius * connector_vector.rotate(-rot_dir * M_PI / 2).normalize();
    geometry::Point anchor_goal =
        center_goal + turning_radius * connector_vector.rotate(rot_dir * M_PI / 2).normalize();

    // The pieces of the path
    geometry::Arc arc_start(center_start, turning_radius, start, anchor_start, rot_dir);
    geometry::LineSegment connector(anchor_start, anchor_goal);
    geometry::Arc arc_goal(center_goal, turning_radius, anchor_goal, goal, static_cast<geometry::ROTATION>(-rot_dir));

    return combine_parts_XSY(arc_start, connector, arc_goal, total_samples);
}

planning::Path DubinsInterpolator::dubins_LSR(geometry::Point& start, geometry::Point& goal, double theta_start,
                                              double theta_goal, double turning_radius, int total_samples) {
    return dubins_XSY(start, goal, theta_start, theta_goal, turning_radius, total_samples, geometry::ROTATION::CCW);
}
planning::Path DubinsInterpolator::dubins_RSL(geometry::Point& start, geometry::Point& goal, double theta_start,
                                              double theta_goal, double turning_radius, int total_samples) {
    return dubins_XSY(start, goal, theta_start, theta_goal, turning_radius, total_samples, geometry::ROTATION::CW);
}

planning::Path DubinsInterpolator::combine_parts_XSY(geometry::Arc arc_start, geometry::LineSegment connector,
                                                     geometry::Arc arc_goal, int total_samples) {
    // Select number of samples depending on arc lengths
    double num_samples[3];
    double total_length = arc_start.length() + arc_goal.length() + connector.length();
    num_samples[0] = static_cast<int>(total_samples * arc_start.length() / total_length);
    num_samples[1] = static_cast<int>(total_samples * connector.length() / total_length);
    num_samples[2] = total_samples - num_samples[0] - num_samples[1];

    // Generate points
    std::vector<double> theta_arc_start, theta_connector, theta_arc_goal;
    std::vector<geometry::Point> points_arc_start, points_connector, points_arc_goal;

    points_arc_start = arc_start.sample(num_samples[0]);
    points_connector = connector.sample(num_samples[1]);
    points_arc_goal = arc_goal.sample(num_samples[2]);

    theta_arc_start = arc_start.sample_tangent_angle(num_samples[0]);
    theta_connector = std::vector<double>(num_samples[1], (connector.finish - connector.start).angle());
    theta_arc_goal = arc_goal.sample_tangent_angle(num_samples[2]);

    planning::Path path;

    path.points = points_arc_start;
    path.points.insert(path.points.end(), points_connector.begin(), points_connector.end());
    path.points.insert(path.points.end(), points_arc_goal.begin(), points_arc_goal.end());

    path.angles = theta_arc_start;
    path.angles.insert(path.angles.end(), theta_connector.begin(), theta_connector.end());
    path.angles.insert(path.angles.end(), theta_arc_goal.begin(), theta_arc_goal.end());

    path.valid = is_collision_free(path);
    return path;
}

std::pair<std::vector<geometry::Point>, std::vector<double>> DubinsInterpolator::dubins_interpolation(
    geometry::Point start, geometry::Point goal, double theta_start, double theta_goal, double turning_radius,
    int points_to_go) {
    planning::Path candidate_paths[6];

    candidate_paths[0] = dubins_LSL(start, goal, theta_start, theta_goal, turning_radius, points_to_go);
    candidate_paths[1] = dubins_RSR(start, goal, theta_start, theta_goal, turning_radius, points_to_go);
    candidate_paths[2] = dubins_LRL(start, goal, theta_start, theta_goal, turning_radius, points_to_go);
    candidate_paths[3] = dubins_RLR(start, goal, theta_start, theta_goal, turning_radius, points_to_go);
    candidate_paths[4] = dubins_LSR(start, goal, theta_start, theta_goal, turning_radius, points_to_go);
    candidate_paths[5] = dubins_RSL(start, goal, theta_start, theta_goal, turning_radius, points_to_go);

    planning::Path& optimal_path = candidate_paths[0];  // LSL and RSR are always valid
    for (auto& path : candidate_paths) {
        if (path.valid && path.length() < optimal_path.length()) {
            optimal_path = path;
        }
    }

    if (!optimal_path.valid) {
        throw std::logic_error(
            "Not possible to reach configuration (goal, theta_goal) from (start, theta_start) via Dubins paths with "
            "given turning radius.");
    }

    return std::make_pair(optimal_path.points, optimal_path.angles);
}

bool DubinsInterpolator::is_collision_free(planning::Path path) {
    bool valid = true;

    for (const geometry::Point& p : path.points) {
        grid::HexCell* nearest_hex = hex_map.nearest_hexagon(p);
        valid &= (nearest_hex->free);
    }

    return valid;
}

ODEOutput HeadedSFMDynamics::ode_func(double t, const std::vector<double>& x) {
    assert(x.size() == x_dim);
    ODEOutput output;
    std::vector<double> dt_x(x_dim);

    // Extract desired information from state vector
    geometry::Point position{x[0], x[1]};
    double theta = x[2];
    geometry::Point velocity{x[3], x[4]};
    double omega = x[5];

    double goal_potential = potential_field.driving_potential->get_potential(position);
    double map_potential = potential_field.map_potential->get_potential(position);
    double potential = goal_potential + map_potential;

    geometry::Point force_goal = potential_field.driving_potential->get_force(position);
    geometry::Point force_obs = potential_field.map_potential->get_force(position);
    geometry::Point force_global = force_goal + force_obs;

    output.report.force_dynamic = 0;
    output.report.force_static = force_obs.norm();
    output.report.force_total = force_global.norm();

    // Decompose force along motion direction
    geometry::Point r_forward(std::cos(theta), std::sin(theta));
    geometry::Point r_orthogonal(-r_forward.y, r_forward.x);

    // Velocity in the local frame
    geometry::Point v_local = velocity.rotate(-theta);

    // Force in the local frame
    geometry::Point force;
    force.x = r_forward.x * force_global.x + r_forward.y * force_global.y;
    force.y = sf_params.k_o * (r_orthogonal.x * force_obs.x + r_orthogonal.y * force_obs.y) -
              sf_params.k_d * v_local.y;  // N = [k_d] * m / s => [k_d] = kg / s

    // Compute k_theta and k_omega
    double moment_of_inertia = robot.moment_of_inertia();
    double k_theta =
        moment_of_inertia * sf_params.k_lambda * force.norm();  // Nm = kg m^2 * [k_l] * N => [k_l] = 1 / (kg m)
    double k_omega = moment_of_inertia * (1 + sf_params.alpha) *
                     sqrt(sf_params.k_lambda * force.norm() /
                          sf_params.alpha);  // Nms = kg m^2 * sqrt([k_l] * N) => [k_l] = 1 / (N s^2) = 1 / (kg m)

    // Compute torque
    double theta_ref = std::atan2(force_goal.y, force_goal.x);
    double diff_theta = theta - theta_ref;
    if (std::abs(diff_theta) > M_PI) {  // subtraction modulo 2PI
        int correction_sign = (diff_theta > M_PI ? -1 : 1);
        diff_theta = correction_sign * (2 * M_PI - std::abs(diff_theta));
    }

    double torque = -k_theta * diff_theta - k_omega * omega;
    double alpha = torque / moment_of_inertia;

    // Angular velocity and acceleration
    dt_x[2] = omega;
    dt_x[5] = alpha;

    // Compute acceleration, taking into account flow resistance
    double flow_resistance = potential / (robot.v_max * robot.v_max);
    geometry::Point a_local;

    int v_sign = (v_local.x > 0) ? 1 : -1;
    double v_norm = v_local.norm();
    double flow_resistance_factor = v_sign * flow_resistance * v_norm * v_norm;

    // Acceleration in the robot frame
    a_local.x = (force.x - flow_resistance_factor) / robot.mass;
    a_local.y = (force.y) / robot.mass;

    // Acceleration in the global frame
    geometry::Point acceleration = a_local.rotate(theta);
    dt_x[3] = acceleration.x;
    dt_x[4] = acceleration.y;

    dt_x[0] = velocity.x;
    dt_x[1] = velocity.y;

    output.state = dt_x;
    return output;
}

ODEOutput SFMGroupDynamics::ode_func(double t, const std::vector<double>& x) {
    ODEOutput output;
    std::vector<double> dt_x(x.size());

    for (size_t i = 0, id = 0; i < x.size(); i += stride, ++id) {
        // Extract desired information from state vector
        geometry::Point position{x[i], x[i + 1]};
        double theta = x[i + 2];
        geometry::Point velocity{x[i + 3], x[i + 4]};
        double omega = x[i + 5];

        double goal_potential = potential_field.driving_potential->get_potential(position);
        double map_potential = potential_field.map_potential->get_potential(position);
        double local_potential = dynamic_potential->get_potential(position, id);

        double potential = goal_potential + map_potential + local_potential;
        if (i > 0) {
            if (sf_params.influence_auxiliary_agents) {
                potential = map_potential + local_potential;
            } else {
                potential = 0;
            }
        }

        geometry::Point force_goal = potential_field.driving_potential->get_force(position);
        geometry::Point force_obs = potential_field.map_potential->get_force(position);
        geometry::Point force_local = dynamic_potential->get_force(position, id);

        geometry::Point force_global = force_goal + force_obs + force_local;
        if (i > 0) {
            if (sf_params.influence_auxiliary_agents) {
                force_global = force_obs + force_local;
            } else {
                force_global = {0, 0};
            }
        }

        if (i == 0) {
            output.report.force_dynamic = force_local.norm();
            output.report.force_static = force_obs.norm();
            output.report.force_total = force_global.norm();
        }

        // Decompose force along motion direction
        geometry::Point r_forward(std::cos(theta), std::sin(theta));
        geometry::Point r_orthogonal(-r_forward.y, r_forward.x);

        // Velocity in the local frame
        geometry::Point v_local = velocity.rotate(-theta);

        geometry::Point force;
        force.x = r_forward.x * force_global.x + r_forward.y * force_global.y;
        force.y = sf_params.k_o * (r_orthogonal.x * force_obs.x + r_orthogonal.y * force_obs.y) -
                  sf_params.k_d * v_local.y;  // N = [k_d] * m / s => [k_d] = kg / s

        // Compute k_theta and k_omega
        double moment_of_inertia = robot.moment_of_inertia();
        double k_theta =
            moment_of_inertia * sf_params.k_lambda * force.norm();  // Nm = kg m^2 * [k_l] * N => [k_l] = 1 / (kg m)
        double k_omega = moment_of_inertia * (1 + sf_params.alpha) *
                         sqrt(sf_params.k_lambda * force.norm() /
                              sf_params.alpha);  // Nms = kg m^2 * sqrt([k_l] * N) => [k_l] = 1 / (N s^2) = 1 / (kg m)

        // Compute torque
        double theta_ref = std::atan2(force_goal.y, force_goal.x);
        double diff_theta = theta - theta_ref;
        if (std::abs(diff_theta) > M_PI) {  // subtraction modulo 2PI
            int correction_sign = (diff_theta > M_PI ? -1 : 1);
            diff_theta = correction_sign * (2 * M_PI - std::abs(diff_theta));
        }

        double torque = -k_theta * diff_theta - k_omega * omega;
        double alpha = torque / moment_of_inertia;

        // Angular velocity and acceleration
        dt_x[i + 2] = omega;
        dt_x[i + 5] = alpha;

        // Compute acceleration, taking into account flow resistance
        double flow_resistance = potential / (robot.v_max * robot.v_max);
        geometry::Point a_local;

        int v_sign = (v_local.x > 0) ? 1 : -1;
        double v_norm = v_local.norm();
        double flow_resistance_factor = v_sign * flow_resistance * v_norm * v_norm;

        // Acceleration in the local frame
        a_local.x = (force.x - flow_resistance_factor) / robot.mass;
        a_local.y = (force.y) / robot.mass;

        // Acceleration in the global frame
        geometry::Point acceleration = a_local.rotate(theta);
        dt_x[i + 3] = acceleration.x;
        dt_x[i + 4] = acceleration.y;

        // Velocity in the global frame
        dt_x[i] = velocity.x;
        dt_x[i + 1] = velocity.y;
    }

    output.state = dt_x;
    return output;
}

ForceReport Integrator::step(double t_old, const std::vector<double>& x_old, double& t_new,
                             std::vector<double>& x_new) {
    ODEOutput k_1 = dynamics.ode_func(t_old, x_old);
    ODEOutput k_2 = dynamics.ode_func(t_old + t_step / 2, x_old + (t_step / 2) * k_1.state);
    ODEOutput k_3 = dynamics.ode_func(t_old + t_step / 2, x_old + (t_step / 2) * k_2.state);
    ODEOutput k_4 = dynamics.ode_func(t_old + t_step, x_old + t_step * k_3.state);

    t_new = t_old + t_step;
    x_new = x_old + (t_step / 6) * (k_1.state + 2 * k_2.state + 2 * k_3.state + k_4.state);

    return k_1.report;
}

MetricReport SocialForceLocalPlanner::generate_report(const std::vector<double>& x, const std::vector<double>& x_new,
                                                      const ForceReport& force_report) {
    MetricReport report;
    report.cumulative_social_force = force_report.force_total;
    report.cumulative_static_force = force_report.force_static;
    report.cumulative_dynamic_force = force_report.force_dynamic;

    geometry::Point p_robot(x[0], x[1]);
    geometry::Point p_new(x_new[0], x_new[1]);
    report.social_work = force_report.force_total * geometry::dist(p_robot, p_new);

    report.min_distance_from_dynamic = std::numeric_limits<double>::infinity();
    for (size_t i = 2; i < x.size(); i += 1) {
        geometry::Point p_obs(x[i], x[i + 1]);
        report.min_distance_from_dynamic = std::min(report.min_distance_from_dynamic, geometry::dist(p_robot, p_obs));
    }

    geometry::Point p_nearest = potential_field.map_potential->nearest_obstacle_map.find_nearest_obstacle(p_robot);
    report.min_distance_from_static = geometry::dist(p_robot, p_nearest);

    return report;
}

FreeWalkOutput SocialForceLocalPlanner::loop(Integrator& integrator, const std::vector<double>& x_init,
                                             const double t_init, int skip_steps_metric_accumulation) {
    FreeWalkOutput output;

    // Empty path initialization
    output.trajectory.global_plan.found_path = true;
    output.trajectory.global_plan.path = std::vector<geometry::Point>();

    // Loop the RK solver until the termination condition is called
    output.trajectory.path.resize(params.walk_length);
    output.trajectory.angles.resize(params.walk_length);
    output.trajectory.v_linear.resize(params.walk_length);
    output.trajectory.v_angular.resize(params.walk_length);

    // For storing intermediate results
    double t(t_init), t_new;
    std::vector<double> x(x_init), x_new;

    for (size_t i = 0; i < params.walk_length; ++i) {
        // Perform single RK step
        ForceReport force_report = integrator.step(t, x, t_new, x_new);

        // Don't accumulate values in the prefix that will be deleted
        if (i >= skip_steps_metric_accumulation) {
            output.report.accumulate(generate_report(x, x_new, force_report));
        }

        t = t_new;
        x = x_new;

        // Start of the state vector is the robot itself
        // Add new entry to vectors
        output.trajectory.path[i] = {x[0], x[1]};

        // Adjust theta for atan2 range
        double theta = x[2];
        if (theta > M_PI) {
            theta -= 2 * M_PI;
        }
        if (theta < -M_PI) {
            theta += 2 * M_PI;
        }

        output.trajectory.angles[i] = theta;
        output.trajectory.v_linear[i] = geometry::Point(x[3], x[4]);
        output.trajectory.v_angular[i] = x[5];

        // Termination condition: repeat final position until walk has proper length
        if (geometry::dist(goal_state.position, output.trajectory.path[i]) < params.goal_cutoff_distance) {
            output.trajectory.path.resize(params.walk_length);
            output.trajectory.angles.resize(params.walk_length);
            output.trajectory.v_linear.resize(params.walk_length);
            output.trajectory.v_angular.resize(params.walk_length);

            for (size_t j = i + 1; j < params.walk_length; ++j) {
                output.trajectory.path[j] = output.trajectory.path[i];
                output.trajectory.angles[j] = output.trajectory.angles[i];
                output.trajectory.v_linear[j] = geometry::Point(0, 0);
                output.trajectory.v_angular[j] = 0;
            }

            break;
        }

        // Update the local state in dynamic potential
        constexpr size_t stride = 6;

        for (size_t j = 0; j < dynamic_potential->local_state.size(); ++j) {
            dynamic_potential->local_state[j].position = {x[stride * j], x[stride * j + 1]};
            dynamic_potential->local_state[j].orientation = x[stride * j + 2];
            dynamic_potential->local_state[j].velocity = {x[stride * j + 3], x[stride * j + 4]};
            dynamic_potential->local_state[j].angular_velocity = x[stride * j + 5];
        }
    }

    return output;
}

FreeWalkOutput SocialForceLocalPlanner::free_walk(bool erase_prefix) {
    // Get current dynamic snapshot
    robot::State robot_state = graph_db.get_robot_state();
    std::vector<robot::State> dynamic_obstacles = graph_db.get_dynamic_obstacles(robot_state.position);

    if (params.project_starting_point) {
        robot_state.position = project_onto_old_trajectory(robot_state.position);
    }

    // Create local state vector by prepending robot state onto vector of dynamic obstacles
    std::vector<robot::State> local_state = dynamic_obstacles;
    local_state.insert(local_state.begin(), robot_state);
    dynamic_potential->update_state(local_state);

    // Design a state
    constexpr size_t x_dim = 6;
    size_t dim = local_state.size();
    std::vector<double> x_init(x_dim * dim);
    double t_init = 0;

    // Initialization for all agents the state (x, y, theta, v_x, v_y, omega)
    for (size_t i = 0; i < dim; ++i) {
        x_init[x_dim * i] = local_state[i].position.x;
        x_init[x_dim * i + 1] = local_state[i].position.y;
        x_init[x_dim * i + 2] = local_state[i].orientation;
        x_init[x_dim * i + 3] = local_state[i].velocity.x;
        x_init[x_dim * i + 4] = local_state[i].velocity.y;
        x_init[x_dim * i + 5] = local_state[i].angular_velocity;
    }

    // Define the robot dynamics
    SFMGroupDynamics robot_dynamics(potential_field, dynamic_potential, robot, sf_params);

    // Runge Kutta integration
    Integrator integrator(robot_dynamics, params.sampling_period);

    FreeWalkOutput output;

    // We need to know if prefix is erased for computing the metrics
    if (erase_prefix) {
        output = loop(integrator, x_init, t_init, params.skip_steps);
    } else {
        output = loop(integrator, x_init, t_init);
    }

    old_trajectory = output.trajectory.path;  // Update old trajectory

    if (erase_prefix) {
        output.trajectory =
            output.trajectory.erase_prefix(params.skip_steps);  // Remove skip_steps points from the start of the path
    }
    return output;
}

Trajectory SocialForceLocalPlanner::get_aggregated_path() {
    Trajectory output;
    output.global_plan = global_plan;

    // Limit maximum number of iterations
    const int max_walks = params.max_samples / params.walk_length;

    for (int i = 0; i < max_walks; ++i) {
        FreeWalkOutput walk_output = free_walk(false);
        Trajectory& walk = walk_output.trajectory;

        // Append the walk onto the total planner output
        output.path.insert(output.path.end(), walk.path.begin(), walk.path.end());
        output.angles.insert(output.angles.end(), walk.angles.begin(), walk.angles.end());
        output.v_linear.insert(output.v_linear.end(), walk.v_linear.begin(), walk.v_linear.end());
        output.v_angular.insert(output.v_angular.end(), walk.v_angular.begin(), walk.v_angular.end());

        // Update robot position in database
        graph_db.update_robot_position(walk.path.back());
        graph_db.update_robot_orientation(walk.angles.back());
        graph_db.update_robot_velocity(walk.v_linear.back());
        graph_db.update_robot_angular_velocity(walk.v_angular.back());

        // Termination condition
        if (geometry::dist(goal_state.position, output.path.back()) < params.goal_cutoff_distance) {
            break;
        }
    }

    return output;
}

FreeWalkOutput LocalAStar::free_walk(bool erase_prefix) {
    // Get current dynamic snapshot
    robot::State robot_state = graph_db.get_robot_state();

    // We are no longer guaranteed to obtain the same path length
    Trajectory total_trajectory;
    std::vector<robot::State> dynamic_obstacles = graph_db.get_dynamic_obstacles(robot_state.position);

    if (params.project_starting_point) {
        robot_state.position = project_onto_old_trajectory(robot_state.position);
    }

    std::vector<robot::State> local_state = dynamic_obstacles;
    local_state.insert(local_state.begin(), robot_state);

    // Create local state vector by prepending robot state onto vector of dynamic obstacles
    dynamic_potential->update_state(local_state);

    // std::cout << "Stoppage distances: "
    //           << robot.distance_to(robot_state.position, robot_state.orientation, dynamic_obstacles, human) << " "
    //           << robot.distance_to(robot_state.position, robot_state.orientation, lp_params.nearest_obstacle_map) <<
    //           " "
    //           << lp_params.stoppage_distance << std::endl;

    // Stop if dynamic obstacle is too close to robot
    if (robot.distance_to(robot_state.position, robot_state.orientation, dynamic_obstacles, human) <
        lp_params.stoppage_distance) {
        FreeWalkOutput output;

        output.trajectory.path = std::vector<geometry::Point>(params.walk_length, robot_state.position);
        output.trajectory.angles = std::vector<double>(params.walk_length, robot_state.orientation);
        output.trajectory.v_linear = std::vector<geometry::Point>(params.walk_length, {0, 0});
        output.trajectory.v_angular = std::vector<double>(params.walk_length, 0);

        if (erase_prefix) {
            output.trajectory = output.trajectory.erase_prefix(
                params.skip_steps);  // Remove skip_steps points from the start of the path
        }

        output.report = generate_report(output.trajectory, dynamic_obstacles);
        return output;
    }

    if (lp_params.fixed_walk_length) {
        // Build total path by combining several local paths
        int skip_prefix = 0;

        while (total_trajectory.path.size() < params.walk_length) {
            // Termination condition: repeat final position until walk has proper length
            if (geometry::dist(goal_state.position, local_state[0].position) < params.goal_cutoff_distance) {
                size_t starting_length = total_trajectory.path.size();
                total_trajectory.path.resize(params.walk_length);

                for (size_t i = starting_length; i < params.walk_length; ++i) {
                    total_trajectory.path[i] = total_trajectory.path[starting_length - 1];
                }

                break;
            }

            Trajectory trajectory = local_planner(local_state);

            // Make sure there is no crash when trajectory is empty
            if (trajectory.path.empty()) {
                break;
            }

            // Skip first point only after first iteration
            total_trajectory.path.insert(total_trajectory.path.end(), trajectory.path.begin() + skip_prefix,
                                         trajectory.path.end());
            total_trajectory.angles.insert(total_trajectory.angles.end(), trajectory.angles.begin() + skip_prefix,
                                           trajectory.angles.end());
            skip_prefix = 1;

            // Update robot position with last point in previous path
            local_state[0].position = total_trajectory.path.back();
            local_state[0].orientation = total_trajectory.angles.back();
        }

        // Drop the excess time steps
        total_trajectory.path.resize(params.walk_length, total_trajectory.path.back());
        total_trajectory.angles.resize(params.walk_length, total_trajectory.angles.back());
        total_trajectory.v_angular.resize(params.walk_length);
        total_trajectory.v_linear.resize(params.walk_length);
    } else {
        Trajectory trajectory = local_planner(local_state);
        total_trajectory = trajectory;

        // Update robot position with last point in previous path
        local_state[0].position = total_trajectory.path.back();
        local_state[0].orientation = total_trajectory.angles.back();

        total_trajectory.v_angular.resize(trajectory.path.size());
        total_trajectory.v_linear.resize(trajectory.path.size());
    }

    // Path smoothing
    total_trajectory.path = lp_params.path_smoother.get_smoothed_path(total_trajectory.path);

    old_trajectory = total_trajectory.path;  // Update old trajectory

    if (erase_prefix) {
        total_trajectory = total_trajectory.erase_prefix(
            params.skip_steps);  // Remove skip_steps points from the start of the trajectory
    }

    FreeWalkOutput output;
    output.trajectory = total_trajectory;
    output.report = generate_report(output.trajectory, dynamic_obstacles);

    if (output.trajectory.path.back() == geometry::Point{0, 0}) {
        std::cout << "Hit." << std::endl;
    }

    return output;
}

std::vector<geometry::Point> GaussianSmoother::get_smoothed_path(const std::vector<geometry::Point>& local_path) const {
    std::vector<geometry::Point> smoothed_path(local_path.size());

    for (size_t i = 0; i < local_path.size(); ++i) {
        geometry::Point sum = {0, 0};
        double coeff_sum = 0;

        // Window around the current point
        for (int j = -half_window; j <= half_window; ++j) {
            size_t idx = i + j;
            double a_j = coeff[j + half_window];

            if (idx >= 0 && idx < local_path.size()) {
                sum += a_j * local_path[idx];
                coeff_sum += a_j;
            }
        }

        smoothed_path[i] = sum / coeff_sum;
    }

    return smoothed_path;
}

MetricReport LocalAStar::generate_report(const Trajectory& trajectory, std::vector<robot::State> dynamic_obstacles) {
    // Accumulate metric report
    MetricReport report;

    for (int i = 0; i < (int)trajectory.path.size() - 1; ++i) {
        MetricReport snapshot_report;
        geometry::Point p_robot = trajectory.path[i];

        snapshot_report.cumulative_static_force = potential_field.map_potential->get_force(p_robot).norm();
        snapshot_report.cumulative_dynamic_force = dynamic_potential->get_force(p_robot, 0).norm();
        snapshot_report.cumulative_social_force = eval_force(p_robot).norm();

        geometry::Point p_new = trajectory.path[i + 1];
        snapshot_report.social_work = snapshot_report.cumulative_social_force * geometry::dist(p_robot, p_new);

        snapshot_report.min_distance_from_dynamic = std::numeric_limits<double>::infinity();
        for (robot::State& obstacle : dynamic_obstacles) {
            snapshot_report.min_distance_from_dynamic =
                std::min(snapshot_report.min_distance_from_dynamic, geometry::dist(p_robot, obstacle.position));
        }

        geometry::Point p_nearest = potential_field.map_potential->nearest_obstacle_map.find_nearest_obstacle(p_robot);
        snapshot_report.min_distance_from_static = geometry::dist(p_robot, p_nearest);

        report.accumulate(snapshot_report);
    }

    return report;
}

void LocalAStar::path_reconstruction(Trajectory& output, grid::HexCell* start, grid::HexCell* goal,
                                     const std::vector<grid::HexCell*>& backtrack) {
    grid::HexCell* node = goal;
    geometry::Point p_previous(goal->x, goal->y);
    output.path.emplace_back(p_previous);

    while (true) {
        if (node == start) break;
        node = backtrack[node->raw_idx];
        geometry::Point p_current(node->x, node->y);
        auto interpolation = util::arange<geometry::Point>(p_previous, p_current, 2);
        output.path.insert(output.path.end(), interpolation.begin() + 1, interpolation.end());
        p_previous = p_current;
    }

    std::reverse(output.path.begin(), output.path.end());
}

bool LocalAStar::termination_condition(grid::HexCell* cell) {
    if (geometry::dist({cell->x, cell->y}, goal_state.position) < params.goal_cutoff_distance) {
        return true;
    }

    if (lp_params.rim_termination) {
        for (grid::HexCell* neighbor : cell->neighbors) {
            if (neighbor == nullptr) {
                return true;
            }
        }
    }

    return false;
}

Trajectory LocalAStar::local_planner(std::vector<robot::State> local_state) {
    // Construct collision checking function
    auto check_collision_static = [this](geometry::Point p) {
        return this->robot.check_collision(p, 0, this->lp_params.nearest_obstacle_map);
    };

    auto check_collision_dynamic = [this, &local_state](geometry::Point p) {
        // Check against all dynamic obstacles
        for (size_t i = 1; i < local_state.size(); ++i) {
            if (this->robot.check_collision(p, 0, local_state[i], this->human)) {
                return true;
            }
        }

        return false;
    };

    auto check_collision = [&check_collision_static, &check_collision_dynamic](geometry::Point p) {
        return check_collision_static(p) || check_collision_dynamic(p);
    };

    // A bit of a hacky approach but it works
    grid::HexCell beacon;
    geometry::Point p_beacon = compute_beacon(local_state[0].position, lp_params.lookahead);
    beacon.x = p_beacon.x;
    beacon.y = p_beacon.y;
    grid::HexCell* hex_beacon = &beacon;

    // Slide grid center towards direction of movement
    geometry::Point grid_center = local_state[0].position;
    geometry::Point normed_direction_vector = (p_beacon - grid_center).normalize();
    double radius = (lp_params.half_height + lp_params.half_width) / 2;
    grid_center += lp_params.center_slide * radius * normed_direction_vector;

    // Construct local grid
    auto hex_grid_factory = grid::HexGridFactory();
    std::unique_ptr<grid::HexGrid> local_grid_ptr = hex_grid_factory.create_hexagonal_neighborhood(
        grid_center, lp_params.half_height, lp_params.half_width, lp_params.grid_quanta, hex_map, check_collision);
    grid::HexGrid& local_grid = *local_grid_ptr;

    // Potential-driven A* on local grid
    grid::HexCell* hex_start = local_grid.nearest_hexagon(local_state[0].position);

    const auto& heuristic = lp_params.heuristic;

    const double inf = heuristic.params.inf;
    std::set<std::pair<double, grid::HexCell*>> priority_queue;
    std::vector<grid::HexCell*> backtrack(local_grid.size(), nullptr);
    std::vector<double> g_score(local_grid.size(), inf);
    std::vector<double> f_score(local_grid.size(), inf);

    g_score[hex_start->raw_idx] = 0;
    f_score[hex_start->raw_idx] = heuristic(hex_start, hex_beacon);
    priority_queue.insert({f_score[hex_start->raw_idx], hex_start});

    // A* forward pass
    grid::HexCell* checkpoint = nullptr;

    while (!priority_queue.empty()) {
        grid::HexCell* head = priority_queue.begin()->second;
        priority_queue.erase(priority_queue.begin());

        if (termination_condition(head)) {
            checkpoint = head;
            break;
        }

        for (grid::HexCell* neighbor : head->neighbors) {
            if (neighbor != nullptr && neighbor->free) {
                double candidate_score;
                if (lp_params.lightweight) {
                    candidate_score = g_score[head->raw_idx] + 1;

                } else {
                    candidate_score = g_score[head->raw_idx] +
                                      eval_field(geometry::Point(neighbor->x, neighbor->y)) / lp_params.downscale_field;
                }

                if (candidate_score < g_score[neighbor->raw_idx]) {
                    auto old_location = priority_queue.find({f_score[neighbor->raw_idx], neighbor});
                    if (old_location != priority_queue.end()) priority_queue.erase(old_location);
                    backtrack[neighbor->raw_idx] = head;
                    g_score[neighbor->raw_idx] = candidate_score;
                    f_score[neighbor->raw_idx] = candidate_score + heuristic(neighbor, hex_beacon);
                    priority_queue.insert({f_score[neighbor->raw_idx], neighbor});
                }
            }
        }
    }

    if (checkpoint == nullptr || !lp_params.rim_termination) {
        // Find nearest node to beacon in estiamted cost-to-go
        grid::HexCell* closest = &(local_grid.grid_map[0]);
        double best_so_far = inf;

        for (grid::HexCell& hex : local_grid.grid_map) {
            if (hex.free && g_score[hex.raw_idx] < inf) {
                double hval = heuristic(&hex, hex_beacon);

                if (hval < best_so_far) {
                    closest = &hex;
                    best_so_far = hval;
                }
            }
        }

        checkpoint = closest;
    }

    Trajectory trajectory;
    trajectory.global_plan = global_plan;

    // If this is not the case robot was put in a situation where movement is impossible
    if (checkpoint != nullptr && checkpoint->free && g_score[checkpoint->raw_idx] < inf) {
        path_reconstruction(trajectory, hex_start, checkpoint, backtrack);
    } else {
        trajectory.path = {};
    }

    trajectory.angles.resize(trajectory.path.size(), 0);
    return trajectory;
}

geometry::Point LocalPlanner::compute_beacon(geometry::Point p, int lookahead) {
    auto& path = global_plan.path;
    geometry::Point p_nearest = path[0];
    int index = 0;

    for (size_t i = 1; i < path.size(); ++i) {
        if (geometry::dist(p, path[i]) < geometry::dist(p, p_nearest)) {
            p_nearest = path[i];
            index = i;
        }
    }

    index = std::min(index + lookahead, (int)path.size() - 1);  // Look ahead several steps
    return path[index];
}

double LocalAStar::eval_field(geometry::Point position) {
    double goal_potential = potential_field.driving_potential->get_potential(position);
    double map_potential = potential_field.map_potential->get_potential(position);
    double local_potential = dynamic_potential->get_potential(position, 0);

    return goal_potential + map_potential + local_potential;
}

geometry::Point LocalAStar::eval_force(geometry::Point position) {
    geometry::Point goal_force = potential_field.driving_potential->get_force(position);
    geometry::Point map_force = potential_field.map_potential->get_force(position);
    geometry::Point local_force = dynamic_potential->get_force(position, 0);

    return goal_force + map_force + local_force;
}

MetricReport MetricReport::accumulate(const MetricReport& new_report) {
    min_distance_from_static = std::min(min_distance_from_static, new_report.min_distance_from_static);
    min_distance_from_dynamic = std::min(min_distance_from_dynamic, new_report.min_distance_from_dynamic);
    cumulative_social_force += new_report.cumulative_social_force;
    cumulative_static_force += new_report.cumulative_static_force;
    cumulative_dynamic_force += new_report.cumulative_dynamic_force;
    social_work += new_report.social_work;

    return *this;
}

}  // namespace social_force