#pragma once

#include "geometry.h"
namespace social_force {

template <int NumRings>
void LocalOrientedAStar<NumRings>::path_reconstruction(Trajectory& output, grid::OrientedHex<NumRings>* start,
                                                       grid::OrientedHex<NumRings>* goal,
                                                       const std::vector<grid::OrientedHex<NumRings>*>& backtrack) {
    grid::OrientedHex<NumRings>* node = goal;
    geometry::Point p_previous(goal->cell->x, goal->cell->y);
    double theta_previous = goal->orientation;

    output.path.emplace_back(p_previous);
    output.angles.emplace_back(theta_previous);

    while (true) {
        if (node == start) break;
        node = backtrack[node->raw_idx];
        geometry::Point p_current(node->cell->x, node->cell->y);
        double theta_current = node->orientation;

        auto interpolation = util::arange<geometry::Point>(p_previous, p_current, 2);
        auto interpolation_theta = util::arange<double>(theta_previous, theta_current, 2);

        output.path.insert(output.path.end(), interpolation.begin() + 1, interpolation.end());
        output.angles.insert(output.angles.end(), interpolation_theta.begin() + 1, interpolation_theta.end());

        p_previous = p_current;
        theta_previous = theta_current;
    }

    std::reverse(output.path.begin(), output.path.end());
    std::reverse(output.angles.begin(), output.angles.end());
}

template <int NumRings>
bool LocalOrientedAStar<NumRings>::check_collision(grid::OrientedHex<NumRings>* hex,
                                                   const std::vector<robot::State>& local_state) {
    // First level check
    if (hex->occupancy == grid::Occupancy::OCCUPIED) {
        return true;
    }
    if (hex->occupancy == grid::Occupancy::FREE) {
        return false;
    }

    // Second level check (proper)
    bool occupied = false;
    geometry::Point p(hex->cell->x, hex->cell->y);
    double theta = hex->orientation;

    // Check against all dynamic obstacles
    for (size_t i = 1; i < local_state.size(); ++i) {
        if (this->robot.check_collision(p, theta, local_state[i], this->human)) {
            occupied = true;
            break;
        }
    }

    // Check against map
    if (!occupied && this->robot.check_collision(p, theta, this->lp_params.nearest_obstacle_map)) {
        occupied = true;
    }

    if (occupied) {
        hex->occupancy = grid::Occupancy::OCCUPIED;
    } else {
        hex->occupancy = grid::Occupancy::FREE;
    }

    return occupied;
}

template <int NumRings>
bool LocalOrientedAStar<NumRings>::termination_condition(grid::OrientedHex<NumRings>* cell) {
    double dist_goal = geometry::dist({cell->cell->x, cell->cell->y}, goal_state.position);
    if (dist_goal < params.goal_cutoff_distance) {
        return true;
    }

    if (lp_params.rim_termination) {
        for (grid::OrientedHex<NumRings>* neighbor : cell->neighbors) {
            if (neighbor == nullptr) {
                return true;
            }
        }
    }

    return false;
}

template <int NumRings>
Trajectory LocalOrientedAStar<NumRings>::local_planner_callback(std::vector<robot::State> local_state,
                                                                int grid_upscale) {
    // Very mainimal collision checking in HexGrid construction since orientation is important
    auto check_collision_trivial = [this](geometry::Point p) { return !this->hex_map.nearest_hexagon(p)->free; };

    geometry::Point p_beacon = compute_beacon(local_state[0].position, lp_params.lookahead);

    // Slide grid center towards direction of movement
    geometry::Point grid_center = local_state[0].position;
    geometry::Point normed_direction_vector = (p_beacon - grid_center).normalize();
    double radius = (lp_params.half_height + lp_params.half_width) / 2;
    grid_center += lp_params.center_slide * radius * normed_direction_vector;

    // If close enough to goal sample grid around goal position
    bool goal_reach_mode = false;

    if (geometry::dist(local_state[0].position, goal_state.position) < radius) {
        grid_center = goal_state.position;
        goal_reach_mode = true;
    }

    // Construct local grid
    auto hex_grid_factory = grid::HexGridFactory();
    std::unique_ptr<grid::HexGrid> local_grid_ptr = hex_grid_factory.create_hexagonal_neighborhood(
        grid_center, this->lp_params.half_height * grid_upscale, this->lp_params.half_width * grid_upscale,
        this->lp_params.grid_quanta, hex_map, check_collision_trivial);
    grid::HexGrid& local_grid = *local_grid_ptr;

    // Construct OrientedGrid
    grid::SuperHexGrid<NumRings> local_super_hex_grid(local_grid);
    grid::OrientedHexGrid<NumRings> oriented_grid(local_super_hex_grid);

    // Locate start cell
    grid::HexCell* hexagon_start = local_grid.nearest_hexagon(local_state[0].position);
    size_t angle_idx = oriented_grid.nearest_angle_index(local_state[0].orientation);
    size_t start_idx = angle_idx * oriented_grid.stride() + hexagon_start->raw_idx;

    grid::OrientedHex<NumRings>* hex_start = &(oriented_grid.oriented_map[start_idx]);
    check_collision(hex_start, local_state);  // fill occupancy field,

    if (hex_start->occupancy == grid::Occupancy::OCCUPIED) {
        bool perturbations_occupied = true;

        for (grid::HexCell* neighbor : hexagon_start->neighbors) {
            if (neighbor != nullptr) {
                size_t neighbor_idx = angle_idx * oriented_grid.stride() + neighbor->raw_idx;
                grid::OrientedHex<NumRings>* perturbation = &(oriented_grid.oriented_map[neighbor_idx]);

                if (!check_collision(perturbation, local_state)) {
                    hex_start = perturbation;
                    perturbations_occupied = false;
                    break;
                }
            }
        }

        if (perturbations_occupied) {
            bool debug_check = this->robot.check_collision(
                {hex_start->cell->x, hex_start->cell->y}, hex_start->orientation, this->lp_params.nearest_obstacle_map);
            Trajectory trajectory;
            trajectory.global_plan = this->global_plan;
            trajectory.path = {};
            return trajectory;
        }
    }

    // Define beacon cell
    grid::OrientedHex<NumRings>* hex_goal = nullptr;
    grid::OrientedHex<NumRings>* hex_beacon = nullptr;
    grid::SuperHex<NumRings> beacon;              // to avoid memory managment
    grid::OrientedHex<NumRings> oriented_beacon;  // to avoid memory managment

    if (goal_reach_mode) {
        grid::HexCell* hexagon_goal = local_grid.nearest_hexagon(goal_state.position);
        size_t goal_angle_idx = oriented_grid.nearest_angle_index(goal_state.orientation);
        size_t goal_idx = goal_angle_idx * oriented_grid.stride() + hexagon_goal->raw_idx;

        hex_goal = &(oriented_grid.oriented_map[goal_idx]);
        hex_beacon = hex_goal;
    } else {
        // A bit of a hacky approach but it works
        beacon.x = p_beacon.x;
        beacon.y = p_beacon.y;

        geometry::Point p_before = compute_beacon(local_state[0].position, lp_params.lookahead - 1);
        if (p_beacon == p_before) {
            oriented_beacon.orientation = local_state[0].orientation;
        } else {
            oriented_beacon.orientation = (p_beacon - p_before).angle();
        }

        oriented_beacon.cell = &beacon;
        hex_beacon = &oriented_beacon;
    }

    // Initialization
    const auto& heuristic = olp_params.heuristic;
    const double inf = heuristic.params.linear_params.inf;
    std::set<std::pair<double, grid::OrientedHex<NumRings>*>> priority_queue;
    std::vector<grid::OrientedHex<NumRings>*> backtrack(oriented_grid.size(), nullptr);
    std::vector<double> g_score(oriented_grid.size(), inf);
    std::vector<double> f_score(oriented_grid.size(), inf);

    g_score[hex_start->raw_idx] = 0;
    f_score[hex_start->raw_idx] = heuristic(hex_start, hex_beacon);
    priority_queue.insert({f_score[hex_start->raw_idx], hex_start});

    // A* forward pass
    grid::OrientedHex<NumRings>* checkpoint = nullptr;

    while (!priority_queue.empty()) {
        grid::OrientedHex<NumRings>* head = priority_queue.begin()->second;
        priority_queue.erase(priority_queue.begin());

        // If hex_reach_mode then terminate if goal hex is reached
        if (goal_reach_mode && head == hex_goal) {
            checkpoint = hex_goal;
            break;
        }

        // Otherwise check the standard termination condition
        if (!goal_reach_mode && termination_condition(head)) {
            checkpoint = head;
            break;
        }

        for (grid::OrientedHex<NumRings>* neighbor : head->neighbors) {
            if (neighbor != nullptr && !check_collision(neighbor, local_state)) {
                double linear_score, angular_score;

                if (lp_params.lightweight) {
                    linear_score = 1;
                } else {
                    linear_score =
                        eval_field(geometry::Point(neighbor->cell->x, neighbor->cell->y)) / lp_params.downscale_field;
                }

                angular_score =
                    olp_params.angle_cost_function(geometry::angle_dist(head->orientation, neighbor->orientation));

                double candidate_score = g_score[head->raw_idx] + linear_score + angular_score;

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

    if (checkpoint == nullptr || (!lp_params.rim_termination && !goal_reach_mode)) {
        grid::OrientedHex<NumRings>* closest = &(oriented_grid.oriented_map[0]);
        double best_so_far = inf;

        for (grid::OrientedHex<NumRings>& hex : oriented_grid.oriented_map) {
            // hex.raw_idx != hex_start->raw_idx
            if (hex.occupancy == grid::Occupancy::FREE && g_score[hex.raw_idx] < inf) {
                // Compute total heuristic value over guiding window
                double hval = heuristic(&hex, hex_beacon);

                // Second check is important so that orientation doesn't get stuck between local planner runs
                if (hval < best_so_far || (hval <= best_so_far && g_score[hex.raw_idx] < g_score[closest->raw_idx])) {
                    closest = &hex;
                    best_so_far = hval;
                }
            }
        }

        // In case start is closest to beacon, we need to get unstuck
        if (closest == hex_start) {
            return local_planner_callback(local_state, 2 * grid_upscale);
        }

        checkpoint = closest;
    }

    // If this is not the case robot was put in a situation where movement is impossible
    Trajectory trajectory;
    trajectory.global_plan = this->global_plan;

    // If this is not the case robot was put in a situation where movement is impossible
    if (checkpoint != nullptr && checkpoint->occupancy == grid::Occupancy::FREE && g_score[checkpoint->raw_idx] < inf) {
        path_reconstruction(trajectory, hex_start, checkpoint, backtrack);
    } else {
        trajectory.path = {};
    }

    return trajectory;
}

}  // namespace social_force