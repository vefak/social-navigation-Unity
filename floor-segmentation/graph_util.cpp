#include "graph_util.h"

namespace graph_util {

void VoronoiMap::find_regions(HexGrid& hex_grid, ComponentMapHex& component_map, std::vector<int>& component_size) {
    std::vector<bool> visited(hex_grid.size(), false);
    std::queue<HexCell*> hex_queue;

    // Insert all obstacle hexes into queue
    for (ConnectedComponentHex& component : component_map.components) {
        for (HexCell* hex : component.nodes) {
            hex_queue.push(hex);
            root[hex->raw_idx] = &component;
            hex_distance[hex->raw_idx] = 0;
            ++component_size[component.index];
        }
    }

    // Run flood fill for Voronoi components
    while (!hex_queue.empty()) {
        HexCell* head = hex_queue.front();
        hex_queue.pop();

        for (HexCell* neighbor : head->neighbors) {
            if (neighbor != nullptr && neighbor->free && !visited[neighbor->raw_idx]) {
                root[neighbor->raw_idx] = root[head->raw_idx];
                hex_distance[neighbor->raw_idx] = hex_distance[head->raw_idx] + 1;
                visited[neighbor->raw_idx] = true;
                ++component_size[root[neighbor->raw_idx]->index];
                hex_queue.push(neighbor);
            }
        }
    }
}

void VoronoiMap::compute_center_distance(HexGrid& hex_grid) {
    std::vector<bool> visited(hex_grid.size(), false);
    std::queue<HexCell*> hex_queue;

    // Insert all hexes which touch the voronoi skeleton into the queue
    for (HexCell& hex : hex_grid.grid_map) {
        if (hex.free) {
            bool all_same = true;
            for (HexCell* neighbor : hex.neighbors) {
                if (neighbor != nullptr && neighbor->free && root[hex.raw_idx] != root[neighbor->raw_idx]) {
                    all_same = false;
                    break;
                }
            }

            if (!all_same) {
                hex_queue.push(&hex);
                visited[hex.raw_idx] = true;
                center_distance[hex.raw_idx] = 0;
            }
        }
    }

    // Flood fill to compute center distances
    while (!hex_queue.empty()) {
        HexCell* head = hex_queue.front();
        hex_queue.pop();

        for (HexCell* neighbor : head->neighbors) {
            if (neighbor != nullptr && neighbor->free && !visited[neighbor->raw_idx]) {
                center_distance[neighbor->raw_idx] = center_distance[head->raw_idx] + 1;
                visited[neighbor->raw_idx] = true;
                hex_queue.push(neighbor);
            }
        }
    }
}

void VoronoiMap::init(HexGrid& hex_grid, ComponentMapHex& component_map) {
    const int grid_size = hex_grid.size();
    const int num_components = component_map.num_components();

    root.resize(grid_size, nullptr);
    hex_distance.resize(grid_size, -1);
    center_distance.resize(grid_size, -1);
    std::vector<int> component_size(num_components, 0);

    // Find Voronoi regions and compute hexagonal distance from nearest obstacle for each hex
    find_regions(hex_grid, component_map, component_size);

    // Compute distance from voronoi skeleton
    compute_center_distance(hex_grid);

    // Construct Voronoi components bottom-up
    cells.resize(num_components);
    for (size_t i = 0; i < cells.size(); ++i) {
        cells[i].root = &(component_map.components[i]);
        cells[i].nodes.reserve(component_size[i]);
        cells[i].hex_distances.reserve(component_size[i]);
        cells[i].center_distances.reserve(component_size[i]);
    }

    for (int hex_idx = 0; hex_idx < grid_size; ++hex_idx) {
        int comp_idx = root[hex_idx]->index;
        cells[comp_idx].nodes.emplace_back(&(hex_grid.grid_map[hex_idx]));
        cells[comp_idx].hex_distances.emplace_back(hex_distance[hex_idx]);
        cells[comp_idx].center_distances.emplace_back(center_distance[hex_idx]);
    }
}

geometry::Point nearest_obstacle_point(geometry::Point p, HexGrid& hex_grid, const VoronoiMap& voronoi_map,
                                       const double err_tol) {
    // Find nearest hexagon via binary search
    HexCell* nearest_hex = hex_grid.nearest_hexagon(p);

    if (!nearest_hex->free) {
        return p;
    }

    // Consider only distances upto
    int hex_distance = voronoi_map.hex_distance[nearest_hex->raw_idx];
    double radius = (hex_distance + sqrt(3) / 2) * hex_grid.spacing.tile_size * sqrt(3);

    // BFS through all points within radius
    std::queue<HexCell*> hex_queue;
    std::vector<bool> visited(hex_grid.size(), false);

    hex_queue.push(nearest_hex);
    visited[nearest_hex->raw_idx] = true;

    double min_dist = 1e7;
    geometry::Point argmin_dist;

    while (!hex_queue.empty()) {
        HexCell* head = hex_queue.front();
        hex_queue.pop();

        for (HexCell* neighbor : head->neighbors) {
            if (neighbor != nullptr && !visited[neighbor->raw_idx]) {
                geometry::Hexagon candidate_hex(
                    geometry::Point(neighbor->x, neighbor->y),
                    hex_grid.spacing.tile_size);  // Construct hexagon corresponding to candidate hex cell

                double dist_hex = geometry::dist(p, candidate_hex.center);

                // Check hexes with centers within the search radius
                if (dist_hex < radius + err_tol) {
                    visited[neighbor->raw_idx] = true;
                    hex_queue.push(neighbor);

                    if (!neighbor->free) {
                        geometry::Point p_cand = candidate_hex.proj(p);

                        if (geometry::dist(p, p_cand) < min_dist) {
                            min_dist = geometry::dist(p, p_cand);
                            argmin_dist = p_cand;
                        }
                    }
                }
            }
        }
    }

    return argmin_dist;
}

geometry::Point nearest_voronoi_skeleton_point(geometry::Point p, HexGrid& hex_grid, const VoronoiMap& voronoi_map) {
    const double inf = 1e7;
    HexCell* nearest_hex = hex_grid.nearest_hexagon(p);

    // Find Voronoi component of nearest hex
    const VoronoiCell* container_cell = nullptr;

    // Occupied hexes are not included in the Voronoi component, so we find the relevant component differently
    if (nearest_hex->free) {
        for (auto& cell : voronoi_map.cells) {
            if (cell.root == voronoi_map.root[nearest_hex->raw_idx]) {
                container_cell = &cell;
            }
        }
    } else {
        for (auto& cell : voronoi_map.cells) {
            if (cell.root->index == voronoi_map.component_map.component_map[nearest_hex->raw_idx]) {
                container_cell = &cell;
            }
        }
    }

    double min_dist = inf;
    geometry::Point p_nearest;

    if (container_cell != nullptr) {
        // Go through perimiter of the Voronoi_cell
        for (const HexCell* hex : container_cell->nodes) {
            if (hex->free) {
                geometry::Hexagon hexagon(geometry::Point(hex->x, hex->y), hex_grid.spacing.tile_size);

                // Consider all edges of the hex that lie on the Voronoi skeleton
                for (const HexCell* neighbor : hex->neighbors) {
                    if (neighbor != nullptr && neighbor->free &&
                        voronoi_map.root[neighbor->raw_idx] !=
                            container_cell
                                ->root) {  // They belong in different components => common edge is on skeleton

                        geometry::Hexagon neighbor_hexagon(geometry::Point(neighbor->x, neighbor->y),
                                                           hex_grid.spacing.tile_size);

                        geometry::LineSegment common_edge = hexagon.common_edge(neighbor_hexagon);
                        geometry::Point p_proj = common_edge.proj(p);

                        if (geometry::dist(p, p_proj) < min_dist) {
                            min_dist = geometry::dist(p, p_proj);
                            p_nearest = p_proj;
                        }
                    }
                }
            }
        }
    } else {
        // Default to more brute force way to find the desired point
        // NOTE: Due to performance constraints, we avoid full brute force, at the cost of full correctness

        // Choose representative points (room centers) from each Voronoi cell and find closest representative
        for (const VoronoiCell& voronoi_cell : voronoi_map.cells) {
            if (voronoi_cell.root->nodes.empty()) continue;
            HexCell* representative = voronoi_cell.root->nodes[0];

            if (container_cell == nullptr || container_cell->root->nodes.empty()) {
                container_cell = &voronoi_cell;
            } else if (geometry::dist(p, {representative->x, representative->y}) <
                       geometry::dist(p, {container_cell->root->nodes[0]->x, container_cell->root->nodes[0]->y})) {
                container_cell = &voronoi_cell;
            }
        }

        // Go through perimiter of the Voronoi_cell containing closest representative
        for (const HexCell* hex : container_cell->nodes) {
            if (hex->free) {
                geometry::Hexagon hexagon(geometry::Point(hex->x, hex->y), hex_grid.spacing.tile_size);

                // Consider all edges of the hex that lie on the Voronoi skeleton
                for (const HexCell* neighbor : hex->neighbors) {
                    if (neighbor != nullptr && neighbor->free &&
                        voronoi_map.root[neighbor->raw_idx] !=
                            voronoi_map.root[hex->raw_idx]) {  // They belong in different components => common edge is
                                                               // on skeleton

                        geometry::Hexagon neighbor_hexagon(geometry::Point(neighbor->x, neighbor->y),
                                                           hex_grid.spacing.tile_size);

                        geometry::LineSegment common_edge = hexagon.common_edge(neighbor_hexagon);
                        geometry::Point p_proj = common_edge.proj(p);

                        if (geometry::dist(p, p_proj) < min_dist) {
                            min_dist = geometry::dist(p, p_proj);
                            p_nearest = p_proj;
                        }
                    }
                }
            }
        }
    }

    return p_nearest;
}

bool operator<(const HexEdge lhs, const HexEdge rhs) {
    if (lhs.hex != rhs.hex) return lhs.hex < rhs.hex;
    return lhs.side < rhs.side;
}

HexEdge WallMap::find_starting_edge(ConnectedComponentHex& component, std::set<HexEdge>& explored) {
    // Find any node on the shell
    HexEdge edge{nullptr, 0};

    for (HexCell* hex : component.nodes) {
        for (int side = 0; side < 6; ++side) {
            HexCell* neighbor = hex->neighbors[side];
            if (neighbor == nullptr || neighbor->free) {
                HexEdge potential_edge{hex, side};
                if (explored.find(potential_edge) == explored.end()) edge = potential_edge;
            }
        }

        if (edge.hex != nullptr) break;
    }

    return edge;
}

std::vector<HexCell*> WallMap::outer_shell(ConnectedComponentHex& component) {
    std::vector<HexCell*> shell;
    std::set<HexEdge> explored;

    while (true) {
        HexEdge starting_edge = find_starting_edge(component, explored);
        if (starting_edge.hex == nullptr) break;

        // Traverse shell in clockwise order
        HexEdge edge = starting_edge;
        shell.emplace_back(edge.hex);
        explored.insert(edge);

        do {
            edge.side = (edge.side + 1) % 6;
            HexCell* neighbor = edge.hex->neighbors[edge.side];

            if (neighbor != nullptr && !neighbor->free) {
                shell.emplace_back(neighbor);

                // Neighboring hexagon and complementary side
                edge.hex = neighbor;
                edge.side = (edge.side + 3) % 6;
            } else {
                explored.insert(edge);
            }

        } while (edge != starting_edge);
    }

    return shell;
}

std::pair<int, int> LineFitCumulativeTables::bitwise_indices(int start, int endpoint, int parity) const {
    int true_start = (start + ((start & 1) ^ parity)) / 2;
    int true_endpoint = (endpoint - ((endpoint & 1) ^ parity)) / 2;
    return {true_start, true_endpoint};
}

double LineFitCumulativeTables::query_util(const std::array<std::vector<double>, 2>& vec, int start, int endpoint,
                                           int parity) const {
    int true_start, true_endpoint;
    std::tie(true_start, true_endpoint) = bitwise_indices(start, endpoint, parity);
    if (true_start > true_endpoint) return 0;
    return vec[parity][true_endpoint + 1] - vec[parity][true_start];
}

LineFitCumulativeTables::LineFitCumulativeTables(const std::vector<HexCell*>& shell) {
    for (size_t parity = 0; parity < 2; ++parity) {
        const size_t half_size = 1 + (shell.size() + 1 - parity) / 2;
        xx[parity].resize(half_size, 0);
        xy[parity].resize(half_size, 0);
        x[parity].resize(half_size, 0);
        y[parity].resize(half_size, 0);
        yy[parity].resize(half_size, 0);
    }

    double x_min = 1e12, x_max = -1e12;
    double y_min = 1e12, y_max = -1e12;

    for (size_t i = 0; i < shell.size(); ++i) {
        x_min = std::min(x_min, shell[i]->x);
        x_max = std::max(x_max, shell[i]->x);
        y_min = std::min(y_min, shell[i]->y);
        y_max = std::max(y_max, shell[i]->y);
    }

    for (size_t i_raw = 0; i_raw < shell.size(); ++i_raw) {
        size_t parity = (i_raw & 1);
        size_t i = i_raw / 2;
        double x_i = 10 * (shell[i_raw]->x - x_min) / (x_max - x_min);
        double y_i = 10 * (shell[i_raw]->y - y_min) / (y_max - y_min);

        x[parity][i + 1] = x[parity][i] + x_i;
        y[parity][i + 1] = y[parity][i] + y_i;
        xx[parity][i + 1] = xx[parity][i] + x_i * x_i;
        xy[parity][i + 1] = xy[parity][i] + x_i * y_i;
        yy[parity][i + 1] = yy[parity][i] + y_i * y_i;
    }
}

double WallMap::line_fit_error_bitwise(const LineFitCumulativeTables& ct, int start, int finish, int parity) {
    double xx_sum = ct.xx_query(start, finish, parity);
    double x_sum = ct.x_query(start, finish, parity);
    double xy_sum = ct.xy_query(start, finish, parity);
    double y_sum = ct.y_query(start, finish, parity);
    double yy_sum = ct.yy_query(start, finish, parity);
    int n = ct.range_length(start, finish, parity);
    if (n == 1) return 0;

    // The residuals are r = ax + by + c1
    const double eps = 1e-4;
    double error_x, error_y;
    error_x = error_y = 1e12;

    if (std::abs(n * xx_sum - x_sum * x_sum) > eps) {
        double a = (n * xy_sum - x_sum * y_sum) / (n * xx_sum - x_sum * x_sum);
        double b = (y_sum * xx_sum - x_sum * xy_sum) / (n * xx_sum - x_sum * x_sum);
        if (std::abs(a) < eps) a = 0;

        double pure_terms = yy_sum + a * a * xx_sum + b * b * n;
        double mixed_terms = 2 * a * b * x_sum - 2 * a * xy_sum - 2 * b * y_sum;
        error_x = std::max(0.0, pure_terms + mixed_terms);
    }

    if (std::abs(n * yy_sum - y_sum * y_sum) > eps) {
        double a = (n * xy_sum - x_sum * y_sum) / (n * yy_sum - y_sum * y_sum);
        double b = (x_sum * yy_sum - y_sum * xy_sum) / (n * yy_sum - y_sum * y_sum);
        if (std::abs(a) < eps) a = 0;

        double pure_terms = xx_sum + a * a * yy_sum + b * b * n;
        double mixed_terms = 2 * a * b * y_sum - 2 * a * xy_sum - 2 * b * x_sum;
        error_y = std::max(0.0, pure_terms + mixed_terms);
    }

    return std::min(error_x, error_y);
}

double WallMap::line_fit_error(const LineFitCumulativeTables& ct, int start, int finish) {
    if (start == finish) return 0;
    double even_error = line_fit_error_bitwise(ct, start, finish, 0);
    double odd_error = line_fit_error_bitwise(ct, start, finish, 1);
    return even_error + odd_error;
}

void WallMap::wall_segmentation(HexGrid& hex_grid, double jump_penalty, int max_wall_size, bool should_merge_walls) {
    const int components_start = num_components();
    int component_index = components_start;

    for (ConnectedComponentHex component : components) {
        std::vector<HexCell*> shell = outer_shell(component);

        // Traverse shell in order and segment into new regions
        std::vector<double> optimal_assignment(shell.size());
        std::vector<int> backtrack(shell.size());

        // Pre-compute cummulative tables for efficient state transitions
        LineFitCumulativeTables ct(shell);

        for (size_t i = 0; i < shell.size(); ++i) {
            backtrack[i] = -1;
            optimal_assignment[i] = line_fit_error(ct, 0, i);

            int start_id = std::max(0, (int)i - max_wall_size);

            for (size_t j = start_id; j < i; ++j) {
                double unary_penalty = line_fit_error(ct, j + 1, i);
                double pairwise_penalty = jump_penalty;

                if (optimal_assignment[j] + pairwise_penalty + unary_penalty < optimal_assignment[i]) {
                    optimal_assignment[i] = optimal_assignment[j] + pairwise_penalty + unary_penalty;
                    backtrack[i] = j;
                }
            }
        }

        // Reconstruct wall segmentation
        const int component_index_start = component_index;
        int endpoint = shell.size() - 1;

        while (endpoint >= 0) {
            int start = backtrack[endpoint] + 1;

            for (size_t i = start; i <= endpoint; ++i) {
                component_map[shell[i]->raw_idx] = component_index;
            }

            endpoint = start - 1;
            ++component_index;
        }

        // Perform wall merging
        if (should_merge_walls) {
            merge_walls(shell, component_index_start);
        }
    }

    // Merge all components that are on the inside part of the walls
    for (HexCell& node : hex_grid.grid_map) {
        if (component_map[node.raw_idx] < components_start) {
            component_map[node.raw_idx] = 0;
        }
    }

    construct_components_from_map(hex_grid, component_index);
}

void WallMap::merge_walls(std::vector<HexCell*>& shell, int baseline_component_index, double threshold_ratio) {
    // Count number of overlapping hexes for each pair of components
    std::map<std::pair<int, int>, int> component_overlap;
    std::unordered_map<int, int> component_size;
    std::unordered_map<int, int> root_component;

    for (HexCell* hex : shell) {
        int hex_comp_idx = component_map[hex->raw_idx];
        std::unordered_set<int> observed_components;

        for (HexCell* neighbor : hex->neighbors) {
            if (neighbor != nullptr) {
                int neigh_comp_idx = component_map[neighbor->raw_idx];
                if (observed_components.find(neigh_comp_idx) != observed_components.end()) continue;
                observed_components.insert(neigh_comp_idx);

                if (neigh_comp_idx >= baseline_component_index && neigh_comp_idx != hex_comp_idx) {
                    ++component_overlap[{hex_comp_idx, neigh_comp_idx}];
                }
            }
        }

        ++component_size[hex_comp_idx];
        root_component[hex_comp_idx] = hex_comp_idx;
    }

    // For all significantly overlapping regions, perform a merge
    for (auto& comp_pair : component_overlap) {
        int source_comp = comp_pair.first.first;
        int destination_comp = comp_pair.first.second;
        int overlap = comp_pair.second;
        int threshold = threshold_ratio * component_size[source_comp];

        if (overlap >= threshold) {
            root_component[source_comp] = root_component[destination_comp];
        }
    }

    // Update the component map
    for (HexCell* hex : shell) {
        component_map[hex->raw_idx] = root_component[component_map[hex->raw_idx]];
    }
}

UnionFind::UnionFind(int num_nodes) {
    root.resize(num_nodes, nullptr);
    size.resize(num_nodes, 0);
}

void UnionFind::make_set(HexCell* cell) {
    root[cell->raw_idx] = cell;
    size[cell->raw_idx] = 1;
}

// With path compression heuristic
HexCell* UnionFind::find(HexCell* cell) {
    if (root[cell->raw_idx] == cell) return cell;
    return root[cell->raw_idx] = find(root[cell->raw_idx]);
}

// With set size heuristic
void UnionFind::set_union(HexCell* u, HexCell* v) {
    HexCell* u_root = find(u);
    HexCell* v_root = find(v);
    if (u_root == v_root) return;

    if (size[u_root->raw_idx] > size[v_root->raw_idx]) {
        root[v_root->raw_idx] = u_root;
        size[u_root->raw_idx] += size[v_root->raw_idx];
    } else {
        root[u_root->raw_idx] = v_root;
        size[v_root->raw_idx] += size[u_root->raw_idx];
    }
}

/* Room segmentation */

void RoomSegmentation::equidistant_bfs_util(HexCell* start, std::vector<ConnectedComponentHex>& distance_components,
                                            std::vector<int>& distance_component_map) {
    std::queue<HexCell*> node_queue;
    node_queue.push(start);
    distance_component_map[start->raw_idx] = distance_components.size();
    distance_components.back().nodes.emplace_back(start);

    while (!node_queue.empty()) {
        HexCell* head = node_queue.front();
        node_queue.pop();

        for (const auto& neighbor : head->neighbors) {
            if (neighbor != nullptr && neighbor->free &&
                voronoi_map.hex_distance[neighbor->raw_idx] == voronoi_map.hex_distance[start->raw_idx] &&
                distance_component_map[neighbor->raw_idx] == -1) {
                distance_component_map[neighbor->raw_idx] = distance_components.size();
                node_queue.push(neighbor);
                distance_components.back().nodes.emplace_back(neighbor);
            }
        }
    }
}

void RoomSegmentation::find_room_centers() {
    // Find connected regions in free space with constant distance
    std::vector<ConnectedComponentHex> distance_components;
    std::vector<int> distance_component_map;
    distance_component_map.resize(voronoi_map.hex_grid.size(), -1);

    for (HexCell& hex : voronoi_map.hex_grid.grid_map) {
        if (hex.free && distance_component_map[hex.raw_idx] == -1) {
            // Create new component
            distance_components.push_back(ConnectedComponentHex());
            distance_components.back().index = distance_components.size();
            equidistant_bfs_util(&hex, distance_components, distance_component_map);
        }
    }

    // Check which components are local maxima
    for (ConnectedComponentHex& distance_component : distance_components) {
        // Check if component is local maximum
        bool local_max = true;

        for (HexCell* hex : distance_component.nodes) {
            for (HexCell* neighbor : hex->neighbors) {
                if (neighbor == nullptr || (neighbor->free && voronoi_map.hex_distance[hex->raw_idx] <
                                                                  voronoi_map.hex_distance[neighbor->raw_idx])) {
                    local_max = false;
                }
            }
        }

        if (local_max) {
            for (HexCell* hex : distance_component.nodes) {
                features.room_centers.emplace_back(hex);
            }
        }
    }
}

void RoomSegmentation::find_doors() {
    // Find nodes on boundary
    for (HexCell& hex : voronoi_map.hex_grid.grid_map) {
        if (hex.free) {
            for (HexCell* neighbor : hex.neighbors) {
                if (neighbor != nullptr && neighbor->free && room_map[hex.raw_idx] != room_map[neighbor->raw_idx]) {
                    features.doors.emplace_back(&hex);
                }
            }
        }
    }
}

void RoomSegmentation::prune_features(std::vector<HexCell*>& distance_optima) {
    UnionFind union_find(voronoi_map.hex_grid.size());
    std::map<HexCell*, std::vector<HexCell*>> equivalent_features;

    for (HexCell* optimum : distance_optima) {
        union_find.make_set(optimum);
    }

    for (HexCell* optimum : distance_optima) {
        for (HexCell* neighbor : optimum->neighbors) {
            if (neighbor != nullptr && union_find.root[neighbor->raw_idx] != nullptr) {
                union_find.set_union(optimum, neighbor);
            }
        }

        // Handle row-wise pruning (for two horizontally touching but non-neighboring hexes)
        for (int offset : {-1, 1}) {
            if (optimum->col + offset >= 0 && optimum->col + offset < voronoi_map.hex_grid.num_cols) {
                HexCell& row_neighbor = voronoi_map.hex_grid.grid_map[optimum->raw_idx + offset];

                // If common neighbors are obstacles, we do not prune (row-wise neighbors are separated by wall)
                bool common_neighbors_are_obstacles = true;

                for (HexCell* neighbor : optimum->neighbors) {
                    for (HexCell* other_neighbor : row_neighbor.neighbors) {
                        if (neighbor == other_neighbor && neighbor->free) {
                            common_neighbors_are_obstacles = false;
                        }
                    }
                }

                if (union_find.root[row_neighbor.raw_idx] != nullptr && !common_neighbors_are_obstacles) {
                    union_find.set_union(optimum, &row_neighbor);
                }
            }
        }
    }

    for (HexCell* optimum : distance_optima) {
        HexCell* root = union_find.find(optimum);
        equivalent_features[root].emplace_back(optimum);
    }

    // Gather new features from the feature equivalence classes
    distance_optima.clear();
    distance_optima.reserve(equivalent_features.size());

    for (auto& feature_pair : equivalent_features) {
        const size_t equiv_class_size = feature_pair.second.size();
        HexCell* feature = feature_pair.second[equiv_class_size / 2];
        distance_optima.emplace_back(feature);
    }
}

void RoomSegmentation::iterative_deepening_bfs_util(int threshold) {
    // Insert all nodes on the boundary into the queue
    std::queue<HexCell*> node_queue;

    for (HexCell& hex : voronoi_map.hex_grid.grid_map) {
        if (room_map[hex.raw_idx] != -1 && voronoi_map.hex_distance[hex.raw_idx] > threshold) {
            // Check if hex is on the boundary
            bool on_boundary = false;
            for (HexCell* neighbor : hex.neighbors) {
                if (neighbor != nullptr && neighbor->free && room_map[neighbor->raw_idx] == -1) {
                    on_boundary = true;
                }
            }

            if (on_boundary) {
                node_queue.push(&hex);
            }
        }
    }

    // Insert new potential room center points
    for (HexCell* room_center : features.room_centers) {
        if (voronoi_map.hex_distance[room_center->raw_idx] == threshold) {
            room_map[room_center->raw_idx] = rooms.size();
            rooms.push_back(Room());
            rooms.back().index = room_map[room_center->raw_idx];
            rooms.back().cells.emplace_back(room_center);
        }
    }

    // Do BFS with set threshold
    while (!node_queue.empty()) {
        HexCell* head = node_queue.front();
        node_queue.pop();

        for (HexCell* neighbor : head->neighbors) {
            if (neighbor != nullptr && neighbor->free && room_map[neighbor->raw_idx] == -1 &&
                voronoi_map.hex_distance[neighbor->raw_idx] >= threshold) {
                room_map[neighbor->raw_idx] = room_map[head->raw_idx];
                node_queue.push(neighbor);
                rooms[room_map[neighbor->raw_idx]].cells.emplace_back(neighbor);
            }
        }
    }
}

void RoomSegmentation::reorder_features() {
    assert(rooms.size() == features.room_centers.size());
    std::unordered_map<HexCell*, int> center_to_room_id;

    for (size_t i = 0; i < rooms.size(); ++i) {
        Room& room = rooms[i];
        center_to_room_id[room.cells[0]] = i;
    }

    // Reorder room centers
    std::vector<HexCell*> room_centers(features.room_centers.size());

    for (size_t i = 0; i < room_centers.size(); ++i) {
        int room_id = center_to_room_id[features.room_centers[i]];
        room_centers[room_id] = features.room_centers[i];
    }

    features.room_centers = room_centers;
}

void RoomSegmentation::construct_rooms() {
    // Find room centers and prune (for uniqueness)
    find_room_centers();
    prune_features(features.room_centers);

    // Find maximal distance in hexgrid
    auto argmax_dist = std::max_element(
        voronoi_map.hex_grid.grid_map.begin(), voronoi_map.hex_grid.grid_map.end(), [this](HexCell& lhs, HexCell& rhs) {
            return this->voronoi_map.hex_distance[lhs.raw_idx] < this->voronoi_map.hex_distance[rhs.raw_idx];
        });
    int max_dist = voronoi_map.hex_distance[argmax_dist->raw_idx];

    // Iterativelly expand the frontier, starting from room centers
    room_map.resize(voronoi_map.hex_grid.size(), -1);

    for (int threshold = max_dist; threshold >= 0; --threshold) {
        iterative_deepening_bfs_util(threshold);
    }

    find_doors();
    // prune_features(features.doors);
    reorder_features();
}

int RoomGraph::bfs_distance(HexCell* start, HexCell* goal) {
    std::unordered_map<HexCell*, int> distance;
    std::queue<HexCell*> hex_queue;

    hex_queue.push(start);
    distance[start] = 0;

    while (!hex_queue.empty()) {
        HexCell* head = hex_queue.front();
        hex_queue.pop();

        if (head == goal) {
            return distance[head];
        }

        for (HexCell* neighbor : head->neighbors) {
            if (neighbor != nullptr && distance.find(neighbor) == distance.end()) {
                distance[neighbor] = distance[head] + 1;
                hex_queue.push(neighbor);
            }
        }
    }

    throw std::logic_error("Path between the two not found. Shouldn't ever happen if rooms are neighboring.");
}

void RoomGraph::generate_adjacency_list() {
    // Order room centers to be consistent with rooms
    std::vector<HexCell*> room_centers;
    room_centers.resize(rooms.size(), nullptr);
    for (HexCell* center : features.room_centers) {
        room_centers[room_map[center->raw_idx]] = center;
    }

    for (Room& room : rooms) {
        adjacency_list.push_back({});
        std::map<Room*, int> adjacent_distance;

        for (HexCell* hex : room.cells) {
            for (HexCell* neighbor : hex->neighbors) {
                if (neighbor != nullptr && neighbor->free && room_map[hex->raw_idx] != room_map[neighbor->raw_idx]) {
                    int neighbor_room_idx = room_map[neighbor->raw_idx];
                    graph_util::Room* neighbor_room = &rooms[neighbor_room_idx];

                    if (adjacent_distance.find(neighbor_room) == adjacent_distance.end()) {
                        // Find paths lengths between room centers via bfs
                        int distance = bfs_distance(room_centers[room.index], room_centers[neighbor_room_idx]);
                        adjacent_distance[neighbor_room] = distance;
                    }
                }
            }
        }

        for (auto neighbor_pair : adjacent_distance) {
            adjacency_list.back().push_back({&room, neighbor_pair.first, neighbor_pair.second});
        }
    }
}

geometry::Point ExactNearestObstacleMap::find_nearest_obstacle(geometry::Point p) {
    return nearest_obstacle_point(p, hex_grid, voronoi_map);
}

geometry::Point ApproximateNearestObstacleMap::find_nearest_obstacle(geometry::Point p) {
    HexCell* nearest_hex = hex_grid.nearest_hexagon(p);
    return nearest_obstacle[nearest_hex->raw_idx];
}

ApproximateNearestObstacleMap::ApproximateNearestObstacleMap(grid::HexGrid& hex_grid, VoronoiMap& voronoi_map)
    : NearestObstacleMap(hex_grid, voronoi_map) {
    nearest_obstacle.resize(hex_grid.size());

    for (size_t i = 0; i < hex_grid.size(); ++i) {
        HexCell& hex = hex_grid.grid_map[i];

        if (hex.free) {
            geometry::Point p_hex(hex.x, hex.y);
            nearest_obstacle[i] = nearest_obstacle_point(p_hex, hex_grid, voronoi_map);
        } else {
            nearest_obstacle[i] = {hex.x, hex.y};
        }
    }
}

}  // namespace graph_util
