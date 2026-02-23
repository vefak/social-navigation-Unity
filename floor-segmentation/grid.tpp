#pragma once

namespace grid {

template <int N>
double Grid<N>::average_degree() {
    int edge_count = 0, obstacle_count;

    for (auto &cell : grid_map) {
        if (!cell.free) {
            for (auto &n : cell.neighbors) {
                if (n != nullptr) edge_count += !(n->free);
            }
            ++obstacle_count;
        }
    }

    return static_cast<double>(edge_count) / obstacle_count;
}

template <int N>
Grid<N>::Grid(const std::vector<bool> &floor_map, int num_rows, int num_cols) : num_rows(num_rows), num_cols(num_cols) {
    grid_map.resize(num_rows * num_cols);

    for (size_t i = 0; i < num_rows; ++i) {
        for (size_t j = 0; j < num_cols; ++j) {
            auto &g = grid_map[i * num_cols + j];
            g.row = i;
            g.col = j;
            g.raw_idx = i * num_cols + j;
            g.free = floor_map[g.raw_idx];
        }
    }
}

template <int N, int NumRings>
SuperGrid<N, NumRings>::SuperGrid(Grid<N> &floor_grid)
    : Grid<enumerate_neighbors(N, NumRings)>(floor_grid.num_rows, floor_grid.num_cols) {
    for (size_t i = 0; i < floor_grid.size(); ++i) {
        this->grid_map[i].row = floor_grid.grid_map[i].row;
        this->grid_map[i].col = floor_grid.grid_map[i].col;
        this->grid_map[i].raw_idx = floor_grid.grid_map[i].raw_idx;
        this->grid_map[i].x = floor_grid.grid_map[i].x;
        this->grid_map[i].y = floor_grid.grid_map[i].y;
        this->grid_map[i].free = floor_grid.grid_map[i].free;
    }
}

template <int NumRings>
void SuperHexGrid<NumRings>::generate_offsets(
    std::array<geometry::IntegerPoint, enumerate_neighbors(6, NumRings)> &offsets, int parity) {
    constexpr int offset_x[2][N] = {{0, 0, 0, 0, -1, -1}, {0, 1, 1, 0, 0, 0}};
    constexpr int offset_y[N] = {-2, -1, 1, 2, 1, -1};
    constexpr int num_directions = enumerate_neighbors(N, NumRings);

    auto it = offsets.begin();
    std::set<std::pair<int, int>> visited;
    std::queue<std::tuple<std::pair<int, int>, int, int>> offset_queue;

    // Insert all 1-jump connections
    for (size_t i = 0; i < N; ++i) {
        std::pair<int, int> offset = {offset_x[parity][i], offset_y[i]};
        visited.insert(offset);
        offset_queue.push({offset, 1, parity ^ (offset_y[i] & 1)});
        *(it++) = geometry::IntegerPoint(offset.first, offset.second);
    }

    // Form k-jump connections via BFS
    while (!offset_queue.empty()) {
        std::pair<int, int> parent_offset;
        int depth, parent_parity;
        std::tie(parent_offset, depth, parent_parity) = offset_queue.front();
        offset_queue.pop();

        for (int i = 0; i < N; ++i) {
            std::pair<int, int> offset{parent_offset.first + offset_x[parent_parity][i],
                                       parent_offset.second + offset_y[i]};

            if (offset != std::pair<int, int>({0, 0}) && depth < NumRings &&
                std::find(visited.begin(), visited.end(), offset) == visited.end()) {
                offset_queue.push({offset, depth + 1, parent_parity ^ (offset_y[i] & 1)});
                visited.insert(offset);
                *(it++) = geometry::IntegerPoint(offset.first, offset.second);
            }
        }
    }
}

template <int NumRings>
void SuperHexGrid<NumRings>::connect_supergrid() {
    // Compute viable offsets
    std::array<geometry::IntegerPoint, enumerate_neighbors(N, NumRings)> even_offsets, odd_offsets;
    generate_offsets(even_offsets, 0);
    generate_offsets(odd_offsets, 1);

    bool angles_computed = false;

    for (SuperCell<N, NumRings> &super_cell : this->grid_map) {
        std::array<geometry::IntegerPoint, enumerate_neighbors(N, NumRings)> *offsets;

        if (super_cell.row & 1) {
            offsets = &odd_offsets;
        } else {
            offsets = &even_offsets;
        }

        bool all_angles_valid = true;

        for (size_t i = 0; i < offsets->size(); ++i) {
            geometry::IntegerPoint &offset = (*offsets)[i];
            int y = super_cell.row + offset.y;
            int x = super_cell.col + offset.x;

            if (y >= 0 && y < this->num_rows && x >= 0 && x < this->num_cols) {
                super_cell.neighbors[i] = &(this->grid_map[y * this->num_cols + x]);
            } else {
                super_cell.neighbors[i] = nullptr;
                all_angles_valid = false;
            }
        }

        // Extract the angles from a hex that has all of its neighbors
        if (!angles_computed && all_angles_valid) {
            for (size_t i = 0; i < this->angles.size(); ++i) {
                SuperHex<NumRings> *neighbor = super_cell.neighbors[i];

                this->angles[i] =
                    (geometry::Point(neighbor->x, neighbor->y) - geometry::Point(super_cell.x, super_cell.y)).angle();
            }

            angles_computed = true;
        }
    }

    if (!angles_computed) {
        throw std::length_error("Insufficient grid size for computing angles.");
    }
}

template <int NumRings>
void OrientedHexGrid<NumRings>::construct() {
    oriented_map.resize(super_hex_grid.size() * angles.size());

    // Construct oriented hex for each (hex, orientation) pair
    for (size_t i = 0; i < super_hex_grid.size(); ++i) {
        for (size_t j = 0; j < angles.size(); ++j) {
            size_t raw_idx = j * super_hex_grid.size() + i;

            oriented_map[raw_idx].raw_idx = raw_idx;
            oriented_map[raw_idx].angle_idx = j;
            oriented_map[raw_idx].cell = &(super_hex_grid.grid_map[i]);
            oriented_map[raw_idx].orientation = angles[j];
            oriented_map[raw_idx].occupancy = Occupancy::UNKNOWN;

            // If cell is not free the orientation doesn't matter
            if (!super_hex_grid.grid_map[i].free) {
                oriented_map[raw_idx].occupancy = Occupancy::OCCUPIED;
            }
        }
    }

    // Connect the oriented hexes

    for (OrientedHex<NumRings> &hex : oriented_map) {
        for (size_t k = 0; k < hex.cell->neighbors.size(); ++k) {
            SuperHex<NumRings> *super_hex_neighbor = hex.cell->neighbors[k];

            if (super_hex_neighbor != nullptr) {
                // The movement direction towards neighbor corresponds to the angle
                size_t neighbor_raw_idx = k * super_hex_grid.size() + super_hex_neighbor->raw_idx;
                hex.neighbors[k] = &(oriented_map[neighbor_raw_idx]);
            } else {
                hex.neighbors[k] = nullptr;
            }
        }
    }
}

template <int NumRings>
size_t OrientedHexGrid<NumRings>::nearest_angle_index(double angle) {
    angle = geometry::normalize_angle(angle);
    double best_candidate = angles[0];
    size_t index = 0;

    for (size_t i = 1; i < angles.size(); ++i) {
        double candidate_angle = geometry::normalize_angle(angles[i]);

        if (geometry::angle_dist(angle, candidate_angle) < geometry::angle_dist(angle, best_candidate)) {
            best_candidate = candidate_angle;
            index = i;
        }
    }

    return index;
}

}  // namespace grid
