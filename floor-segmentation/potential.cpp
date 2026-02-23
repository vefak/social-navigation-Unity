#include "potential.h"

namespace potential {

double get_interpolated_potential(geometry::Point p, std::vector<double>& grid_potential, grid::HexGrid& hex_grid,
                                  const double eps = 1e-7) {
    grid::HexCell* nearest_hex = hex_grid.nearest_hexagon(p);
    double dist_nearest = std::min(eps, geometry::dist(p, {nearest_hex->x, nearest_hex->y}));

    double total_potential = grid_potential[nearest_hex->raw_idx] / dist_nearest;
    double total_inv_dist = 1 / dist_nearest;

    for (grid::HexCell* neighbor : nearest_hex->neighbors) {
        if (neighbor != nullptr && neighbor->free) {
            double dist = std::min(eps, geometry::dist(p, {neighbor->x, neighbor->y}));
            total_potential += grid_potential[neighbor->raw_idx] / dist;
            total_inv_dist += 1 / dist;
        }
    }

    total_potential /= total_inv_dist;
    return total_potential;
}

geometry::Point get_interpolated_force(geometry::Point p, std::vector<geometry::Point>& grid_force,
                                       grid::HexGrid& hex_grid, const double eps = 1e-7) {
    grid::HexCell* nearest_hex = hex_grid.nearest_hexagon(p);
    double dist_nearest = std::min(eps, geometry::dist(p, {nearest_hex->x, nearest_hex->y}));

    geometry::Point total_force = grid_force[nearest_hex->raw_idx] / dist_nearest;
    double total_inv_dist = 1 / dist_nearest;

    for (grid::HexCell* neighbor : nearest_hex->neighbors) {
        if (neighbor != nullptr && neighbor->free) {
            double dist = std::min(eps, geometry::dist(p, {neighbor->x, neighbor->y}));
            total_force += grid_force[neighbor->raw_idx] / dist;
            total_inv_dist += 1 / dist;
        }
    }

    total_force /= total_inv_dist;
    return total_force;
}

double GridMapPotential::get_potential(geometry::Point p) {
    return get_interpolated_potential(p, grid_potential, hex_grid);
}

geometry::Point GridMapPotential::get_force(geometry::Point p) {
    return get_interpolated_force(p, grid_force, hex_grid);
}

double ObstacleRepulsionPotential::get_potential(geometry::Point p) {
    geometry::Point p_obs = nearest_obstacle_map.find_nearest_obstacle(p);
    double obs_dist = geometry::dist(p, p_obs);
    return dist_fnc->eval(obs_dist);
}

geometry::Point ObstacleRepulsionPotential::get_force(geometry::Point p) {
    geometry::Point p_obs = nearest_obstacle_map.find_nearest_obstacle(p);
    double obs_dist = geometry::dist(p, p_obs);
    return dist_fnc->grad(obs_dist) * (p_obs - p) / std::max(obs_dist, 1e-6);
}

double VoronoiFieldPotential::get_potential(geometry::Point p) {
    geometry::Point p_obs = nearest_obstacle_map.find_nearest_obstacle(p);
    double obs_dist = geometry::dist(p, p_obs);

    geometry::Point p_vor = graph_util::nearest_voronoi_skeleton_point(p, hex_map, voronoi_map);
    double vor_dist = geometry::dist(p, p_vor);

    if (obs_dist > dist_obs_max) {
        return 0;
    }

    return scale * (alpha / (alpha + obs_dist)) * (vor_dist / (obs_dist + vor_dist)) *
           ((obs_dist - dist_obs_max) * (obs_dist - dist_obs_max) / dist_obs_max_squared);
}

geometry::Point VoronoiFieldPotential::get_force(geometry::Point p) {
    geometry::Point p_obs = nearest_obstacle_map.find_nearest_obstacle(p);
    double obs_dist = geometry::dist(p, p_obs);

    geometry::Point p_vor = graph_util::nearest_voronoi_skeleton_point(p, hex_map, voronoi_map);
    double vor_dist = geometry::dist(p, p_vor);

    // Compute derivatives via chain rule
    double nabla_obs =
        (alpha / (alpha + obs_dist)) * (vor_dist / (obs_dist + vor_dist)) *
        ((obs_dist - dist_obs_max) / dist_obs_max_squared) *
        (-(obs_dist - dist_obs_max) / (alpha + obs_dist) - (obs_dist - dist_obs_max) / (obs_dist + vor_dist) + 2);

    double nabla_vor = (alpha / (alpha + obs_dist)) *
                       ((obs_dist - dist_obs_max) * (obs_dist - dist_obs_max) / dist_obs_max_squared) *
                       (obs_dist / ((obs_dist + vor_dist) * (obs_dist + vor_dist)));

    return -scale * (nabla_obs * (p - p_obs).normalize() + nabla_vor * (p - p_vor).normalize());
}

double GridDrivingPotential::get_potential(geometry::Point p) {
    return get_interpolated_potential(p, grid_potential, hex_grid);
}

geometry::Point GridDrivingPotential::get_force(geometry::Point p) {
    return get_interpolated_force(p, grid_force, hex_grid);
}

double GoalAndPathPotential::get_potential(geometry::Point p) {
    double goal_dist = geometry::dist(p, path.back());
    double valley_dist = geometry::dist(planning::nearest_path_point(path, p), p);

    double goal_potential = goal_attraction * dist_fnc->eval(goal_dist);
    double valley_potential = valley_attraction * dist_fnc->eval(valley_dist);
    return goal_potential + valley_potential;
}

geometry::Point GoalAndPathPotential::get_force(geometry::Point p) {
    double goal_dist = geometry::dist(p, path.back());
    geometry::Point p_nearest = planning::nearest_path_point(path, p);
    double valley_dist = geometry::dist(p_nearest, p);

    geometry::Point force_goal =
        goal_attraction * dist_fnc->grad(goal_dist) * (path.back() - p) / std::max(goal_dist, 1e-6);
    geometry::Point force_valley =
        valley_attraction * dist_fnc->grad(valley_dist) * (p_nearest - p) / std::max(valley_dist, 1e-6);
    return force_goal + force_valley;
}

std::vector<geometry::Point> LookaheadPotential::extract_window(geometry::Point p) {
    // Find nearest waypoint and its index
    geometry::Point p_nearest = path[0];
    int index = 0;

    for (size_t i = 1; i < path.size(); ++i) {
        if (geometry::dist(p, path[i]) < geometry::dist(p, p_nearest)) {
            p_nearest = path[i];
            index = i;
        }
    }

    index = std::min(index + lookahead, (int)path.size() - 1);  // Look ahead several steps

    // Extract window
    int adjusted_window_size = std::min(window_size, (int)path.size() - index);
    std::vector<geometry::Point> window(adjusted_window_size);

    for (size_t i = 0; i < adjusted_window_size && index + i < path.size(); ++i) {
        window[i] = path[index + i];
    }

    return window;
}

double LookaheadPotential::get_potential(geometry::Point p) {
    double potential = 0;
    auto window = extract_window(p);

    for (auto& attractor : window) {
        potential += waypoint_attraction * dist_fnc->eval(geometry::dist(p, attractor));
    }

    return potential / window.size();
}

geometry::Point LookaheadPotential::get_force(geometry::Point p) {
    geometry::Point force;
    auto window = extract_window(p);

    for (auto& attractor : window) {
        double distance = geometry::dist(p, attractor);
        force += waypoint_attraction * dist_fnc->grad(distance) * (attractor - p) / std::max(distance, 1e-6);
    }

    return force / window.size();
}

double WindowPotential::get_potential(geometry::Point p) {
    double potential = 0;

    for (size_t i = counter; i < counter + window_size && i < path.size(); ++i) {
        potential += waypoint_attraction * dist_fnc->eval(geometry::dist(p, path[i]));
    }

    int adjusted_window_size = std::min(window_size, (int)path.size() - counter);

    update_counter(p);

    return potential / adjusted_window_size;
}

geometry::Point WindowPotential::get_force(geometry::Point p) {
    geometry::Point force;

    for (size_t i = counter; i < counter + window_size && i < path.size(); ++i) {
        double distance = geometry::dist(p, path[i]);
        force += waypoint_attraction * dist_fnc->grad(distance) * (path[i] - p) / std::max(distance, 1e-6);
    }

    int adjusted_window_size = std::min(window_size, (int)path.size() - counter);

    update_counter(p);

    return force / adjusted_window_size;
}

// While point is within the cutoff distance of the moving window, move the window
void WindowPotential::update_counter(geometry::Point p) {
    while (true) {
        bool close_to_window = false;

        for (size_t i = counter; i < counter + window_size && i < path.size(); ++i) {
            if (geometry::dist(p, path[i]) < cutoff_dist) {
                close_to_window = true;
                break;
            }
        }

        if (!close_to_window) break;

        ++counter;
    }
}

std::vector<robot::State> DynamicPotential::get_nearby_obstacles(size_t id) {
    std::vector<robot::State> dynamic_obstacles;
    assert(local_state.size() > 0);
    dynamic_obstacles.reserve(local_state.size() - 1);

    for (size_t i = 0; i < local_state.size(); ++i) {
        if (i != id) {
            dynamic_obstacles.push_back(local_state[i]);
        }
    }

    return dynamic_obstacles;
}

double RipplePotential::get_potential(geometry::Point p, size_t id) {
    double potential = 0;

    for (robot::State& obstacle : get_nearby_obstacles(id)) {
        geometry::Point p_obs = obstacle.position;
        double obs_dist = geometry::dist(p, p_obs);
        potential += dist_fnc->eval(obs_dist);
    }

    return potential;
}

geometry::Point RipplePotential::get_force(geometry::Point p, size_t id) {
    geometry::Point force{0, 0};

    for (robot::State& obstacle : get_nearby_obstacles(id)) {
        geometry::Point p_obs = obstacle.position;
        double obs_dist = geometry::dist(p, p_obs);
        force += dist_fnc->grad(obs_dist) * (p_obs - p) / std::max(obs_dist, 1e-6);
    }

    return force;
}

// Formula that computes the directional dilatation
double EggshellPotential::get_contraction(double movement_direction, double offset_angle) {
    double diff = std::abs(movement_direction - offset_angle);
    diff = std::min(diff, 2 * M_PI - diff);

    double contraction = std::cos(diff / 2);  // zero at diff = PI
    contraction = std::min(contraction, cutoff);

    return contraction;
}

double EggshellPotential::get_potential(geometry::Point p, size_t id) {
    double potential = 0;

    for (robot::State& obstacle : get_nearby_obstacles(id)) {
        geometry::Point p_obs = obstacle.position;

        double offset_angle = (p - p_obs).angle();
        double contraction = get_contraction(obstacle.orientation, offset_angle);

        double obs_dist = geometry::dist(p, p_obs);
        potential += dist_fnc->eval(obs_dist) / contraction;
    }

    return potential;
}

geometry::Point EggshellPotential::get_force(geometry::Point p, size_t id) {
    geometry::Point force{0, 0};

    for (robot::State& obstacle : get_nearby_obstacles(id)) {
        geometry::Point p_obs = obstacle.position;

        double offset_angle = (p - p_obs).angle();
        double contraction = get_contraction(obstacle.orientation, offset_angle);

        double obs_dist = geometry::dist(p, p_obs);
        force += (dist_fnc->grad(obs_dist) / contraction) * (p_obs - p) / std::max(obs_dist, 1e-6);
    }

    return force;
}

};  // namespace potential