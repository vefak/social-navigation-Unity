#pragma once

namespace planning {

template <int NumRings>
bool OrientedAStarPlanner<NumRings>::check_collision(OrientedHex<NumRings>* hex) {
    // First (cheaper) check
    if (hex->occupancy == Occupancy::OCCUPIED) {
        return true;
    }
    if (hex->occupancy == Occupancy::FREE) {
        return false;
    }

    // Second check: explicit collision test
    geometry::Point p_hex(hex->cell->x, hex->cell->y);
    bool is_collision = this->robot.check_collision(p_hex, hex->orientation, nearest_obstacle_map);

    // Lazily update the map to avoid repeating expensive collision checks
    if (is_collision) {
        hex->occupancy = Occupancy::OCCUPIED;
    } else {
        hex->occupancy = Occupancy::FREE;
    }

    return is_collision;
}

template <int NumRings>
OrientedGridSearchOutput<NumRings> OrientedAStarPlanner<NumRings>::find_path(
    const OrientedAStarInput<NumRings>& input) {
    OrientedGridSearchOutput<NumRings> output;

    if (this->check_collision(input.hex_start) || this->check_collision(input.hex_goal)) {
        output.found_path = false;
        return output;
    }

    const double inf = input.heuristic.params.linear_params.inf;
    std::set<std::pair<double, OrientedHex<NumRings>*>> priority_queue;
    std::vector<OrientedHex<NumRings>*> backtrack(this->oriented_grid.size(), nullptr);
    std::vector<double> g_score(this->oriented_grid.size(), inf);
    std::vector<double> f_score(this->oriented_grid.size(), inf);

    g_score[input.hex_start->raw_idx] = 0;
    f_score[input.hex_start->raw_idx] = input.heuristic(input.hex_start, input.hex_goal);
    priority_queue.insert({f_score[input.hex_start->raw_idx], input.hex_start});
    output.search_tree.open_nodes.insert(input.hex_start);

    // A* forward pass
    while (!priority_queue.empty()) {
        OrientedHex<NumRings>* head = priority_queue.begin()->second;
        priority_queue.erase(priority_queue.begin());
        output.search_tree.closed_nodes.insert(head);
        output.search_tree.open_nodes.erase(head);
        if (head == input.hex_goal) {
            break;
        }

        for (OrientedHex<NumRings>* neighbor : head->neighbors) {
            if (neighbor != nullptr && !this->check_collision(neighbor)) {
                double linear_cost =
                    geometry::dist({head->cell->x, head->cell->y}, {neighbor->cell->x, neighbor->cell->y});
                double angular_cost =
                    input.angle_cost_function(geometry::angle_dist(head->orientation, neighbor->orientation));

                double candidate_score = g_score[head->raw_idx] + linear_cost + angular_cost;

                if (candidate_score < g_score[neighbor->raw_idx]) {
                    auto old_location = priority_queue.find({f_score[neighbor->raw_idx], neighbor});
                    if (old_location != priority_queue.end()) priority_queue.erase(old_location);
                    backtrack[neighbor->raw_idx] = head;
                    g_score[neighbor->raw_idx] = candidate_score;
                    f_score[neighbor->raw_idx] = candidate_score + input.heuristic(neighbor, input.hex_goal);
                    priority_queue.insert({f_score[neighbor->raw_idx], neighbor});
                    output.search_tree.open_nodes.insert(neighbor);
                }
            }
        }
    }

    // Path reconstruction
    if (g_score[input.hex_goal->raw_idx] < inf) {
        this->path_reconstruction(output, input.hex_start, input.hex_goal, backtrack);
        output.found_path = true;
    } else {
        output.found_path = false;
    }

    return output;
}

template <int NumRings>
double OrientedHeuristic<NumRings>::wall_penalty(OrientedHex<NumRings>* node) const {
    if (params.linear_params.avoid_walls) {
        for (OrientedHex<NumRings>* neighbor : node->neighbors) {
            if (neighbor != nullptr && neighbor->occupancy == Occupancy::OCCUPIED) {
                return params.linear_params.inf;
            }
        }
    }
    return 0;
}

template <int NumRings>
double OrientedHeuristic<NumRings>::valley_potential(OrientedHex<NumRings>* node) const {
    if (params.linear_params.valley_potential) {
        return params.linear_params.potential[node->cell->raw_idx];  // Index of corresponding superhex
    }
    return 0;
}

template <int NumRings>
double OrientedEuclideanDistance<NumRings>::operator()(OrientedHex<NumRings>* node, OrientedHex<NumRings>* goal) const {
    const double unit_distance = this->params.linear_params.tile_size * sqrt(3);
    geometry::Point p_node(node->cell->x, node->cell->y);
    geometry::Point p_goal(goal->cell->x, goal->cell->y);

    double linear_distance = geometry::dist(p_node, p_goal) / unit_distance;
    double angular_distance = geometry::angle_dist(node->orientation, goal->orientation);

    return linear_distance + this->params.angle_weight * angular_distance + this->wall_penalty(node) +
           this->valley_potential(node);
}

template <int NumRings>
double OrientedHexagonalDistance<NumRings>::operator()(OrientedHex<NumRings>* node, OrientedHex<NumRings>* goal) const {
    geometry::IntegerPointCubic p_node_cubic(geometry::IntegerPoint(node->cell->col, node->cell->row));
    geometry::IntegerPointCubic p_goal_cubic(geometry::IntegerPoint(goal->cell->col, goal->cell->row));

    double hexagonal_distance = geometry::hex_dist(p_node_cubic, p_goal_cubic);
    double angular_distance = geometry::angle_dist(node->orientation, goal->orientation);

    return hexagonal_distance + this->params.angle_weight * angular_distance + this->wall_penalty(node) +
           this->valley_potential(node);
}

template <int NumRings>
void OrientedGridBasedPlanner<NumRings>::path_reconstruction(OrientedGridSearchOutput<NumRings>& output,
                                                             OrientedHex<NumRings>* start, OrientedHex<NumRings>* goal,
                                                             std::vector<OrientedHex<NumRings>*>& backtrack,
                                                             int points_per_segment) const {
    OrientedHex<NumRings>* node = goal;
    geometry::Point p_previous(goal->cell->x, goal->cell->y);
    output.path.emplace_back(p_previous);

    while (true) {
        output.cells_on_path.emplace_back(node);
        if (node == start) break;
        node = backtrack[node->raw_idx];
        geometry::Point p_current(node->cell->x, node->cell->y);
        auto interpolation = util::arange<geometry::Point>(p_previous, p_current, points_per_segment);
        output.path.insert(output.path.end(), interpolation.begin() + 1, interpolation.end());
        p_previous = p_current;
    }

    std::reverse(output.path.begin(), output.path.end());
    std::reverse(output.cells_on_path.begin(), output.cells_on_path.end());
}

}  // namespace planning