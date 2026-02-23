#include "grid.h"

#include <algorithm>
#include <thread>

#include "util.h"

namespace grid {

std::vector<bool> GridFactory::opencv_image_obstacle_segmentation(const cv::Mat &floor_img) {
    std::vector<bool> grid_map(floor_img.rows * floor_img.cols);

    for (size_t i = 0; i < floor_img.rows; ++i) {
        for (size_t j = 0; j < floor_img.cols; ++j) {
            const cv::Vec3b &bgr = floor_img.at<cv::Vec3b>(i, j);
            grid_map[i * floor_img.cols + j] = (bgr[0] == UCHAR_MAX && bgr[1] == UCHAR_MAX && bgr[2] == UCHAR_MAX);
        }
    }

    return grid_map;  // row-major
}

/* Hexagonal Grid */

// Variables x_offset, y_offset are usef for local grid construction; their value is zero by default
HexGrid::HexGrid(const std::vector<bool> &floor_map, int num_rows, int num_cols, double tile_size, double x_offset,
                 double y_offset)
    : Grid<6>(floor_map, num_rows, num_cols), spacing(tile_size) {
    // Handle the even rows
    x_coo_even = util::linspace<double>(x_offset + spacing.x_offset_even, spacing.x_stride, num_cols);
    y_coo_even = util::linspace<double>(y_offset + spacing.y_offset_even, spacing.y_stride, (num_rows + 1) / 2);

    // Handle the odd rows
    x_coo_odd = util::linspace<double>(x_offset + spacing.x_offset_odd, spacing.x_stride, num_cols);
    y_coo_odd = util::linspace<double>(y_offset + spacing.y_offset_odd, spacing.y_stride, num_rows / 2);

    // Fill out the grid map
    for (size_t i = 0; i < num_rows; ++i) {
        for (size_t j = 0; j < num_cols; ++j) {
            auto &g = grid_map[i * num_cols + j];

            // Set offsets based on row parity
            if (i & 1) {
                g.x = x_offset + spacing.x_offset_odd + j * spacing.x_stride;
                g.y = y_offset + spacing.y_offset_odd + (i / 2) * spacing.y_stride;
            } else {
                g.x = x_offset + spacing.x_offset_even + j * spacing.x_stride;
                g.y = y_offset + spacing.y_offset_even + (i / 2) * spacing.y_stride;
            }
        }
    }

    // Coordinate offsets for generating neighboring nodes
    const int num_directions = 6;
    const int offset_x[2][num_directions] = {{0, 0, 0, 0, -1, -1}, {0, 1, 1, 0, 0, 0}};
    const int offset_y[num_directions] = {-2, -1, 1, 2, 1, -1};

    // Enumerate all neighbors for each hexagon
    for (size_t i = 0; i < num_rows; ++i) {
        for (size_t j = 0; j < num_cols; ++j) {
            auto &g = grid_map[i * num_cols + j];

            for (int k = 0; k < num_directions; ++k) {
                int y = i + offset_y[k];
                int x = j + offset_x[i & 1][k];

                if (y >= 0 && y < num_rows && x >= 0 && x < num_cols) {
                    g.neighbors[k] = &grid_map[y * num_cols + x];
                } else {
                    g.neighbors[k] = nullptr;
                }
            }
        }
    }
}

HexCell *HexGrid::nearest_hexagon(const geometry::Point &point) {
    double x = point.x;
    double y = point.y;

    // Use binary search to find appropriate coordinates
    auto x_even_it = std::lower_bound(x_coo_even.begin(), x_coo_even.end(), x);
    auto y_even_it = std::lower_bound(y_coo_even.begin(), y_coo_even.end(), y);
    auto x_odd_it = std::lower_bound(x_coo_odd.begin(), x_coo_odd.end(), x);
    auto y_odd_it = std::lower_bound(y_coo_odd.begin(), y_coo_odd.end(), y);

    // Adjust the even iterators
    if (x_even_it == x_coo_even.end() ||
        (x_even_it != x_coo_even.begin() && std::abs(x - *(x_even_it - 1)) < std::abs(x - *x_even_it))) {
        --x_even_it;
    }
    if (y_even_it == y_coo_even.end() ||
        (y_even_it != y_coo_even.begin() && std::abs(y - *(y_even_it - 1)) < std::abs(y - *y_even_it))) {
        --y_even_it;
    }

    // Adjust the odd iterators
    if (x_odd_it == x_coo_odd.end() ||
        (x_odd_it != x_coo_odd.begin() && std::abs(x - *(x_odd_it - 1)) < std::abs(x - *x_odd_it))) {
        --x_odd_it;
    }
    if (y_odd_it == y_coo_odd.end() ||
        (y_odd_it != y_coo_odd.begin() && std::abs(y - *(y_odd_it - 1)) < std::abs(y - *y_odd_it))) {
        --y_odd_it;
    }

    // Compare the even and odd distances
    double dist2_even = (x - *x_even_it) * (x - *x_even_it) + (y - *y_even_it) * (y - *y_even_it);
    double dist2_odd = (x - *x_odd_it) * (x - *x_odd_it) + (y - *y_odd_it) * (y - *y_odd_it);

    if (dist2_even < dist2_odd) {
        int y_even = 2 * (y_even_it - y_coo_even.begin());
        int x_even = x_even_it - x_coo_even.begin();
        return &(grid_map[y_even * num_cols + x_even]);
    }

    int y_odd = 2 * (y_odd_it - y_coo_odd.begin()) + 1;
    int x_odd = x_odd_it - x_coo_odd.begin();
    return &(grid_map[y_odd * num_cols + x_odd]);
}

geometry::Point HexGrid::get_true_coo(geometry::IntegerPoint loc) const {
    geometry::Point p_loc;

    if ((loc.y & 1) == 0) {
        p_loc.y = spacing.y_offset_even + spacing.y_stride * (loc.y / 2);
        p_loc.x = spacing.x_offset_even + spacing.x_stride * loc.x;
    } else {
        p_loc.y = spacing.y_offset_odd + spacing.y_stride * (loc.y / 2);
        p_loc.x = spacing.x_offset_odd + spacing.x_stride * loc.x;
    }

    return p_loc;
}

// Checks line of sight between hex grid points
bool HexGrid::line_of_sight(const geometry::IntegerPoint &A, const geometry::IntegerPoint &B) {
    // Closest even and odd hexagons to point A
    int y_even_nearest = 2 * (A.y / 2);
    int y_odd_nearest = 2 * (A.y / 2) + 1;
    int x_even_nearest = A.x;
    int x_odd_nearest = A.x;

    // Initialize starting point for pointer walk algorithm
    auto y_even_it = y_coo_even.begin() + y_even_nearest / 2;
    auto y_odd_it = y_coo_odd.begin() + y_odd_nearest / 2;
    auto x_even_it = x_coo_even.begin() + x_even_nearest;
    auto x_odd_it = x_coo_odd.begin() + x_odd_nearest;

    // Interpolate line using h_dist + 1 points
    int h_dist = geometry::hex_dist(geometry::IntegerPointCubic(A), geometry::IntegerPointCubic(B));
    auto samples = util::arange<geometry::Point>(get_true_coo(A), get_true_coo(B), h_dist + 1);
    geometry::Point prev_sample;
    grid::HexCell *hex_prev;
    bool first_sample = true;

    for (geometry::Point &sample : samples) {
        // Move in any direction while a step can reduce the distance
        for (int r : {-1, 1}) {
            while (y_even_nearest + 2 * r >= 0 && y_even_nearest + 2 * r < num_rows &&
                   std::abs(*y_even_it - sample.y) > std::abs(*(y_even_it + r) - sample.y)) {
                y_even_nearest += 2 * r;
                y_even_it += r;
            }

            while (y_odd_nearest + 2 * r >= 0 && y_odd_nearest + 2 * r < num_rows &&
                   std::abs(*y_odd_it - sample.y) > std::abs(*(y_odd_it + r) - sample.y)) {
                y_odd_nearest += 2 * r;
                y_odd_it += r;
            }

            while (x_even_nearest + r >= 0 && x_even_nearest + r < num_cols &&
                   std::abs(*x_even_it - sample.x) > std::abs(*(x_even_it + r) - sample.x)) {
                x_even_nearest += r;
                x_even_it += r;
            }

            while (x_odd_nearest + r >= 0 && x_odd_nearest + r < num_cols &&
                   std::abs(*x_odd_it - sample.x) > std::abs(*(x_odd_it + r) - sample.x)) {
                x_odd_nearest += r;
                x_odd_it += r;
            }
        }

        // Compare nearest even and nearest odd hexagon
        geometry::Point nearest_even(*x_even_it, *y_even_it);
        geometry::Point nearest_odd(*x_odd_it, *y_odd_it);
        int y_nearest, x_nearest;

        if (geometry::dist(sample, nearest_even) < geometry::dist(sample, nearest_odd)) {
            y_nearest = y_even_nearest;
            x_nearest = x_even_nearest;
        } else {
            y_nearest = y_odd_nearest;
            x_nearest = x_odd_nearest;
        }

        grid::HexCell *hex_nearest = &at({y_nearest, x_nearest});
        if (!hex_nearest->free) return false;

        // Check intersections not captured by the Bresenham algorithm
        if (!first_sample) {
            for (grid::HexCell *neighbor : hex_nearest->neighbors) {
                for (grid::HexCell *prev_neighbor : hex_prev->neighbors) {
                    if (neighbor == prev_neighbor && neighbor != nullptr) {
                        geometry::Point prev_point = get_true_coo({hex_prev->col, hex_prev->row});
                        geometry::Point cur_point = get_true_coo({hex_nearest->col, hex_nearest->row});
                        geometry::Point mid_point = (cur_point + prev_point) / 2;

                        geometry::Point v_midpoint = mid_point - prev_sample;
                        geometry::Point v_line = sample - prev_sample;
                        geometry::Point v_hex = get_true_coo({neighbor->col, neighbor->row}) - prev_sample;

                        if (geometry::cross(v_line, v_hex) * geometry::cross(v_line, v_midpoint) < 0 && !neighbor->free)
                            return false;
                    }
                }
            }
        }

        first_sample = false;
        hex_prev = hex_nearest;
        prev_sample = sample;
    }

    return true;
}

double HexGridFactory::hex_overlap(geometry::Hexagon hex, SquareGrid &unit_grid) {
    double occupied_space = 0;

    // Find relevant region for search
    int y_min = hex.center.y - hex.radius * sqrt(3) / 2;
    int y_max = hex.center.y + hex.radius * sqrt(3) / 2 + 1;
    int x_min = hex.center.x - hex.radius;
    int x_max = hex.center.x + hex.radius + 1;
    y_min = std::max(0, y_min);
    y_max = std::min(static_cast<int>(unit_grid.num_rows) - 1, y_max);
    x_min = std::max(0, x_min);
    x_max = std::min(static_cast<int>(unit_grid.num_cols) - 1, x_max);

    for (int i = y_min; i <= y_max; ++i) {
        for (int j = x_min; j <= x_max; ++j) {
            if (!unit_grid[{i, j}].free) {
                occupied_space += geometry::intersection(geometry::UnitSquare(i, j), hex).area();
            }
        }
    }

    return (hex.area() - occupied_space) / hex.area();
}

std::unique_ptr<HexGrid> HexGridFactory::create_hexagonal_grid_from_square_grid(SquareGrid unit_grid, double tile_size,
                                                                                double threshold) {
    HexSpacing spacing(tile_size);

    // Construct coordinate vectors for the relevant ranges
    auto x_coo_even = util::arange<double>(
        spacing.x_offset_even, static_cast<double>(unit_grid.num_cols) + spacing.x_stride / 2, spacing.x_stride);
    auto y_coo_even = util::arange<double>(
        spacing.y_offset_even, static_cast<double>(unit_grid.num_rows) + spacing.y_stride / 2, spacing.y_stride);
    auto x_coo_odd = util::arange<double>(
        spacing.x_offset_odd, static_cast<double>(unit_grid.num_cols) + spacing.x_stride / 2, spacing.x_stride);
    auto y_coo_odd = util::arange<double>(
        spacing.y_offset_odd, static_cast<double>(unit_grid.num_rows) + spacing.y_stride / 2, spacing.y_stride);

    size_t hex_rows = y_coo_even.size() + y_coo_odd.size();
    size_t hex_cols = std::max(x_coo_even.size(), x_coo_odd.size());
    std::vector<bool> hex_grid_map(hex_rows * hex_cols);

    // Find percentage of occupied area in intersection between hexagons and the square grid
    const int num_threads = std::max((int)std::thread::hardware_concurrency(), 1);
    std::vector<std::thread> threads;

    for (int t = 0; t < num_threads; ++t) {
        threads.emplace_back(std::thread([&y_coo_even, &x_coo_even, &y_coo_odd, &x_coo_odd, &hex_grid_map, &unit_grid,
                                          tile_size, threshold, t, num_threads, hex_rows, hex_cols, this] {
            for (int i = t; i < y_coo_even.size(); i += num_threads) {
                for (auto j = 0; j < x_coo_even.size(); ++j) {
                    geometry::Point p(x_coo_even[j], y_coo_even[i]);
                    hex_grid_map[2 * i * hex_cols + j] =
                        this->hex_overlap(geometry::Hexagon(p, tile_size), unit_grid) > threshold;
                }
            }

            for (int i = t; i < y_coo_odd.size(); i += num_threads) {
                for (auto j = 0; j < x_coo_odd.size(); ++j) {
                    geometry::Point p(x_coo_odd[j], y_coo_odd[i]);
                    hex_grid_map[(2 * i + 1) * hex_cols + j] =
                        this->hex_overlap(geometry::Hexagon(p, tile_size), unit_grid) > threshold;
                }
            }
        }));
    }

    for (auto &th : threads) th.join();

    return std::unique_ptr<HexGrid>(new HexGrid(hex_grid_map, hex_rows, hex_cols, tile_size));
}

std::unique_ptr<HexGrid> HexGridFactory::create_hexagonal_grid_from_image(const cv::Mat &floor_img, double tile_size,
                                                                          double threshold) {
    std::vector<bool> floor_map = opencv_image_obstacle_segmentation(floor_img);
    SquareGrid unit_grid(floor_map, floor_img.rows, floor_img.cols, 1);
    return create_hexagonal_grid_from_square_grid(unit_grid, tile_size, threshold);
}

std::unique_ptr<HexGrid> HexGridFactory::create_hexagonal_neighborhood(
    geometry::Point p, int half_height, int half_width, double grid_quanta, HexGrid &hex_grid,
    std::function<bool(geometry::Point)> check_collision) {
    HexSpacing spacing(grid_quanta);
    double x_min, x_max, y_min, y_max;
    x_min = p.x - half_width;
    x_max = p.x + half_width;
    y_min = p.y - half_height;
    y_max = p.y + half_height;

    // Construct coordinate vectors for the relevant ranges
    auto x_coo_even =
        util::arange<double>(x_min + spacing.x_offset_even, x_max + spacing.x_stride / 2, spacing.x_stride);
    auto y_coo_even =
        util::arange<double>(y_min + spacing.y_offset_even, y_max + spacing.y_stride / 2, spacing.y_stride);
    auto x_coo_odd = util::arange<double>(x_min + spacing.x_offset_odd, x_max + spacing.x_stride / 2, spacing.x_stride);
    auto y_coo_odd = util::arange<double>(y_min + spacing.y_offset_odd, y_max + spacing.y_stride / 2, spacing.y_stride);

    size_t hex_rows = y_coo_even.size() + y_coo_odd.size();
    size_t hex_cols = std::max(x_coo_even.size(), x_coo_odd.size());
    std::vector<bool> grid_map(hex_rows * hex_cols);

    // Find percentage of occupied area in intersection between hexagons and the square grid
    const int num_threads = std::max((int)std::thread::hardware_concurrency(), 1);
    std::vector<std::thread> threads;

    for (int t = 0; t < num_threads; ++t) {
        threads.emplace_back(std::thread([&y_coo_even, &x_coo_even, &y_coo_odd, &x_coo_odd, &grid_map, &hex_grid, t,
                                          num_threads, hex_rows, hex_cols, this, check_collision] {
            for (int i = t; i < y_coo_even.size(); i += num_threads) {
                for (auto j = 0; j < x_coo_even.size(); ++j) {
                    geometry::Point p(x_coo_even[j], y_coo_even[i]);
                    grid_map[2 * i * hex_cols + j] = !check_collision(p);
                }
            }

            for (int i = t; i < y_coo_odd.size(); i += num_threads) {
                for (auto j = 0; j < x_coo_odd.size(); ++j) {
                    geometry::Point p(x_coo_odd[j], y_coo_odd[i]);
                    grid_map[(2 * i + 1) * hex_cols + j] = !check_collision(p);
                }
            }
        }));
    }

    for (auto &th : threads) th.join();

    return std::unique_ptr<HexGrid>(new HexGrid(grid_map, hex_rows, hex_cols, grid_quanta, x_min, y_min));
}

/* Square Grid */

SquareGrid::SquareGrid(const std::vector<bool> &floor_map, int num_rows, int num_cols, int tile_size)
    : Grid<4>(floor_map, num_rows, num_cols), tile_size(tile_size) {
    // Fill out the grid map
    for (size_t i = 0; i < num_rows; ++i) {
        for (size_t j = 0; j < num_cols; ++j) {
            auto &g = grid_map[i * num_cols + j];
            g.x = (j + 0.5) * tile_size;
            g.y = (i + 0.5) * tile_size;
        }
    }

    // Coordinate offsets for generating neighboring nodes
    const int num_directions = 4;
    const int offset_x[4] = {0, 1, 0, -1};
    const int offset_y[4] = {-1, 0, 1, 0};

    // Enumerate all neighbors for each square
    for (size_t i = 0; i < num_rows; ++i) {
        for (size_t j = 0; j < num_cols; ++j) {
            auto &g = grid_map[i * num_cols + j];

            for (int k = 0; k < num_directions; ++k) {
                int y = i + offset_y[k];
                int x = j + offset_x[k];

                if (y >= 0 && y < num_rows && x >= 0 && x < num_cols) {
                    g.neighbors[k] = &grid_map[y * num_cols + x];
                } else {
                    g.neighbors[k] = nullptr;
                }
            }
        }
    }
}

std::unique_ptr<SquareGrid> SquareGridFactory::create_square_grid_from_image(const cv::Mat &floor_image, int tile_size,
                                                                             double threshold) {
    size_t square_rows = (floor_image.rows + tile_size - 1) / tile_size;
    size_t square_cols = (floor_image.cols + tile_size - 1) / tile_size;
    std::vector<bool> square_grid_map(square_rows * square_cols);
    std::vector<bool> floor_map = opencv_image_obstacle_segmentation(floor_image);

    const int num_threads = std::max((int)std::thread::hardware_concurrency(), 1);
    std::vector<std::thread> threads;

    for (int t = 0; t < num_threads; ++t) {
        threads.emplace_back(std::thread([tile_size, threshold, t, num_threads, square_rows, square_cols, &floor_map,
                                          &square_grid_map, &floor_image] {
            for (int grid_row = t; grid_row < square_rows; grid_row += num_threads) {
                for (int grid_col = 0; grid_col < square_cols; ++grid_col) {
                    int free_count = 0, total_count = 0;

                    for (int tile_row = 0; tile_row < tile_size; ++tile_row) {
                        for (int tile_col = 0; tile_col < tile_size; ++tile_col) {
                            int i_cell = tile_size * grid_row + tile_row;
                            int j_cell = tile_size * grid_col + tile_col;

                            if (i_cell < floor_image.rows && j_cell < floor_image.cols) {
                                free_count += floor_map[i_cell * floor_image.cols + j_cell];
                                ++total_count;
                            }
                        }
                    }

                    square_grid_map[grid_row * square_cols + grid_col] = ((double)free_count / total_count > threshold);
                }
            }
        }));
    }

    for (auto &th : threads) th.join();

    return std::unique_ptr<SquareGrid>(new SquareGrid(square_grid_map, square_rows, square_cols, tile_size));
}

}  // namespace grid
