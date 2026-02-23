#include "planning.h"

#include "geometry.h"

namespace planning {

PlannerOutput load_planner_output(std::filesystem::path path) {
    PlannerOutput output;
    output.found_path = true;
    std::ifstream is(path);

    double x, y;
    while (is >> x >> y) {
        output.path.push_back({x, y});
    }

    return output;
}

double Path::length() {
    double len = 0;
    for (size_t i = 1; i < points.size(); ++i) {
        len += geometry::dist(points[i - 1], points[i]);
    }

    return len;
}

double Heuristic::wall_penalty(HexCell* node) const {
    if (params.avoid_walls) {
        for (HexCell* neighbor : node->neighbors) {
            if (neighbor != nullptr && !neighbor->free) {
                return params.inf;
            }
        }
    }
    return 0;
}

double Heuristic::valley_potential(HexCell* node) const {
    if (params.valley_potential) {
        return params.potential[node->raw_idx];
    }
    return 0;
}

double EuclideanDistance::operator()(HexCell* node, HexCell* goal) const {
    const double unit_distance = params.tile_size * sqrt(3);
    geometry::Point p_node(node->x, node->y);
    geometry::Point p_goal(goal->x, goal->y);
    return geometry::dist(p_node, p_goal) / unit_distance + wall_penalty(node) + valley_potential(node);
}

double HexagonalDistance::operator()(HexCell* node, HexCell* goal) const {
    geometry::IntegerPointCubic p_node_cubic(geometry::IntegerPoint(node->col, node->row));
    geometry::IntegerPointCubic p_goal_cubic(geometry::IntegerPoint(goal->col, goal->row));
    return geometry::hex_dist(p_node_cubic, p_goal_cubic) + wall_penalty(node) + valley_potential(node);
}

void GridBasedPlanner::path_reconstruction(GridSearchOutput& output, HexCell* start, HexCell* goal,
                                           std::vector<HexCell*>& backtrack, int points_per_segment) const {
    HexCell* node = goal;
    geometry::Point p_previous(goal->x, goal->y);
    output.path.emplace_back(p_previous);

    while (true) {
        output.cells_on_path.emplace_back(node);
        if (node == start) break;
        node = backtrack[node->raw_idx];
        geometry::Point p_current(node->x, node->y);
        auto interpolation = util::arange<geometry::Point>(p_previous, p_current, points_per_segment);
        output.path.insert(output.path.end(), interpolation.begin() + 1, interpolation.end());
        p_previous = p_current;
    }

    std::reverse(output.path.begin(), output.path.end());
    std::reverse(output.cells_on_path.begin(), output.cells_on_path.end());
}

GridSearchOutput AStarPlanner::find_path(GridSearchInput input) {
    GridSearchOutput output;

    if (!input.hex_start->free || !input.hex_goal->free) {
        output.found_path = false;
        return output;
    }

    const double inf = input.heuristic.params.inf;
    std::set<std::pair<double, HexCell*>> priority_queue;
    std::vector<HexCell*> backtrack(hex_grid.size(), nullptr);
    std::vector<int> g_score(hex_grid.size(), static_cast<int>(inf));
    std::vector<double> f_score(hex_grid.size(), inf);

    g_score[input.hex_start->raw_idx] = 0;
    f_score[input.hex_start->raw_idx] = input.heuristic(input.hex_start, input.hex_goal);
    priority_queue.insert({f_score[input.hex_start->raw_idx], input.hex_start});
    output.search_tree.open_nodes.insert(input.hex_start);

    // A* forward pass
    while (!priority_queue.empty()) {
        HexCell* head = priority_queue.begin()->second;
        priority_queue.erase(priority_queue.begin());
        output.search_tree.closed_nodes.insert(head);
        output.search_tree.open_nodes.erase(head);
        if (head == input.hex_goal) break;

        for (HexCell* neighbor : head->neighbors) {
            if (neighbor != nullptr && neighbor->free) {
                int candidate_score = g_score[head->raw_idx] + 1;

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
        path_reconstruction(output, input.hex_start, input.hex_goal, backtrack);
        output.found_path = true;
    } else {
        output.found_path = false;
    }

    return output;
}

GridSearchOutput ThetaStarPlanner::find_path(ThetaStarInput input) {
    GridSearchOutput output;

    if (!input.hex_start->free || !input.hex_goal->free) {
        output.found_path = false;
        return output;
    }

    const double inf = input.heuristic.params.inf;
    const double unit_distance = hex_grid.spacing.tile_size * sqrt(3);

    std::set<std::pair<double, HexCell*>> priority_queue;
    std::vector<HexCell*> backtrack(hex_grid.size(), nullptr);
    std::vector<bool> visited(hex_grid.size(), false);
    std::vector<double> g_score(hex_grid.size(), inf);
    std::vector<double> f_score(hex_grid.size(), inf);

    g_score[input.hex_start->raw_idx] = 0;
    f_score[input.hex_start->raw_idx] = input.heuristic(input.hex_start, input.hex_goal);
    priority_queue.insert({f_score[input.hex_start->raw_idx], input.hex_start});
    output.search_tree.open_nodes.insert(input.hex_start);

    // Theta* forward pass
    while (!priority_queue.empty()) {
        HexCell* head = priority_queue.begin()->second;
        priority_queue.erase(priority_queue.begin());
        output.search_tree.closed_nodes.insert(head);
        output.search_tree.open_nodes.erase(head);
        visited[head->raw_idx] = true;
        if (head == input.hex_goal) break;

        for (HexCell* neighbor : head->neighbors) {
            if (neighbor != nullptr && neighbor->free && !visited[neighbor->raw_idx]) {
                HexCell *parent = backtrack[head->raw_idx], *predecessor;
                double candidate_score;

                if (parent != nullptr &&
                    hex_grid.line_of_sight({parent->col, parent->row}, {neighbor->col, neighbor->row})) {
                    geometry::Point p_parent(parent->x, parent->y);
                    geometry::Point p_neighbor(neighbor->x, neighbor->y);
                    candidate_score = g_score[parent->raw_idx] + geometry::dist(p_parent, p_neighbor) / unit_distance;
                    predecessor = parent;
                } else {
                    candidate_score = g_score[head->raw_idx] + 1;
                    predecessor = head;
                }

                if (candidate_score < g_score[neighbor->raw_idx]) {
                    auto old_location = priority_queue.find({f_score[neighbor->raw_idx], neighbor});
                    if (old_location != priority_queue.end()) priority_queue.erase(old_location);
                    backtrack[neighbor->raw_idx] = predecessor;
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
        path_reconstruction(output, input.hex_start, input.hex_goal, backtrack, input.points_per_segment);
        output.found_path = true;
    } else {
        output.found_path = false;
    }

    return output;
}

geometry::Point nearest_path_point(const std::vector<geometry::Point>& path, geometry::Point p) {
    geometry::Point closest;
    double min_distance = 1e7;

    for (size_t i = 1; i < path.size(); ++i) {
        geometry::LineSegment segment(path[i - 1], path[i]);
        geometry::Point candidate = segment.proj(p);

        if (geometry::dist(p, candidate) < min_distance) {
            min_distance = geometry::dist(p, candidate);
            closest = candidate;
        }
    }

    return closest;
}

std::function<double(double)> get_angle_cost_function(std::string cost_function, double angle_cost_multiplier,
                                                      double angle_sensitivity, double angle_deadzone,
                                                      double maximum_angle, double penalty) {
    std::function<double(double)> angle_cost_function;
    maximum_angle -= angle_deadzone;

    if (cost_function == "linear") {
        angle_cost_function = [angle_cost_multiplier, angle_sensitivity, angle_deadzone, maximum_angle,
                               penalty](double angle) {
            if (angle < angle_deadzone) return 0.0;

            return angle_cost_multiplier * angle_sensitivity * (angle - angle_deadzone) +
                   (angle > maximum_angle) * penalty;
        };
    } else if (cost_function == "quadratic") {
        angle_cost_function = [angle_cost_multiplier, angle_sensitivity, angle_deadzone, maximum_angle,
                               penalty](double angle) {
            if (angle < angle_deadzone) return 0.0;

            angle -= angle_deadzone;
            return angle_cost_multiplier * (angle_sensitivity * angle_sensitivity) * (angle * angle) +
                   (angle > maximum_angle) * penalty;
        };
    } else {
        assert(cost_function == "exponential");

        angle_cost_function = [angle_cost_multiplier, angle_sensitivity, angle_deadzone, maximum_angle,
                               penalty](double angle) {
            if (angle < angle_deadzone) return 0.0;

            angle -= angle_deadzone;
            return angle_cost_multiplier * (exp(angle_sensitivity * angle) - 1) + (angle > maximum_angle) * penalty;
        };
    }

    return angle_cost_function;
}

}  // namespace planning
