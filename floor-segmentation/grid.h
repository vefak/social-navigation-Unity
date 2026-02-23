#pragma once
#include <cmath>
#include <functional>
#include <memory>
#include <opencv2/opencv.hpp>
#include <unordered_set>
#include <vector>

#include "geometry.h"

namespace grid {

// Defines generic cell
template <int N>
struct Cell {
    int row, col, raw_idx;
    double x, y;
    bool free;
    std::array<Cell<N> *, N> neighbors;

    constexpr int num_directions() const { return N; }
};

typedef Cell<6> HexCell;
typedef Cell<4> SquareCell;

constexpr int enumerate_neighbors(int N, int NumRings) { return N * NumRings * (NumRings + 1) / 2; }

template <int N, int NumRings>
using SuperCell = Cell<enumerate_neighbors(N, NumRings)>;

template <int NumRings>
using SuperHex = SuperCell<6, NumRings>;

// Add a third option for lazy collision detection
enum class Occupancy { FREE, OCCUPIED, UNKNOWN };

template <int N, int NumRings>
struct OrientedCell {
    SuperCell<N, NumRings> *cell;
    double orientation;
    int angle_idx, raw_idx;
    Occupancy occupancy;
    std::array<OrientedCell<N, NumRings> *, enumerate_neighbors(N, NumRings)> neighbors;
};

template <int NumRings>
using OrientedHex = OrientedCell<6, NumRings>;

// Define generic grid
template <int N>
class Grid {
   public:
    std::vector<Cell<N>> grid_map;
    size_t num_rows, num_cols;

    Grid(int num_rows, int num_cols) : num_rows(num_rows), num_cols(num_cols), grid_map(num_rows * num_cols) {}

    Grid(const std::vector<bool> &floor_map, int num_rows, int num_cols);

    double average_degree();

    int size() const { return grid_map.size(); }

    Cell<N> &at(std::pair<int, int> indices) { return grid_map[indices.first * num_cols + indices.second]; }

    Cell<N> &operator[](std::pair<int, int> indices) { return this->at(indices); }
};

// Define grid of supercells
template <int N, int NumRings>
struct SuperGrid : public Grid<enumerate_neighbors(N, NumRings)> {
    std::array<double, enumerate_neighbors(N, NumRings)> angles;  // Stores angles of viable directions

    SuperGrid(Grid<N> &floor_grid);
};

struct GridFactory {
    /*
    Segment the floor image into obstacles and free space. From the floor plan image, white pixels
    are treated as free space and the non-white pixels as obstacles.
    */
    std::vector<bool> opencv_image_obstacle_segmentation(const cv::Mat &floor_img);
};

/* Hexagonal Grid */

// Single hexagon in hexgrid
struct HexSpacing {
    double tile_size;
    double x_offset_even, x_offset_odd, x_stride;
    double y_offset_even, y_offset_odd, y_stride;

    HexSpacing(double tile_size) : tile_size(tile_size) {
        x_offset_even = tile_size;
        x_offset_odd = tile_size * 5 / 2;
        y_offset_even = tile_size * sqrt(3) / 2;
        y_offset_odd = tile_size * sqrt(3);
        x_stride = 3 * tile_size;
        y_stride = sqrt(3) * tile_size;
    }
};

// Hexgrid in "flat top" configuration
// Dimensions 2*rows x cols
// Even rows and odd rows shifted from each other by 1.5 x tile_size.
class HexGrid : public Grid<6> {
    // Get true coordinates of hexagon center from grid coordinates.
    geometry::Point get_true_coo(geometry::IntegerPoint loc) const;

   public:
    HexSpacing spacing;
    std::vector<double> x_coo_even, y_coo_even, x_coo_odd, y_coo_odd;

    // Occupancy map in row-major order
    HexGrid(const std::vector<bool> &floor_map, int num_rows, int num_cols, double tile_size, double x_offset = 0,
            double y_offset = 0);

    HexCell *nearest_hexagon(const geometry::Point &point);

    bool line_of_sight(const geometry::IntegerPoint &A, const geometry::IntegerPoint &B);
};

// Define grid of supercells
template <int NumRings>
struct SuperHexGrid : public SuperGrid<6, NumRings> {
    static constexpr int N = 6;

    void generate_offsets(std::array<geometry::IntegerPoint, enumerate_neighbors(6, NumRings)> &offsets, int parity);

    void connect_supergrid();

    SuperHexGrid(HexGrid &hex_grid) : SuperGrid<N, NumRings>(hex_grid) { connect_supergrid(); }
};

template <int NumRings>
struct OrientedHexGrid {
    static constexpr int N = 6;  // Hexagon
    SuperHexGrid<NumRings> &super_hex_grid;
    std::vector<OrientedHex<NumRings>> oriented_map;
    std::array<double, enumerate_neighbors(N, NumRings)> angles;

    size_t size() { return oriented_map.size(); }

    size_t stride() { return super_hex_grid.size(); }

    size_t nearest_angle_index(double angle);

    void construct();

    OrientedHexGrid(SuperHexGrid<NumRings> &super_hex_grid)
        : super_hex_grid(super_hex_grid), angles(super_hex_grid.angles) {
        construct();
    }
};

/* Square Grid */

class SquareGrid : public Grid<4> {
    int tile_size;

   public:
    SquareGrid(const std::vector<bool> &floor_map, int num_rows, int num_cols, int tile_size);

    friend class ComponentMap;
};

// Factory classes for the two types of grids

class HexGridFactory : public GridFactory {
    // Compute the total area of the free space contained in the hexagon centered at p
    double hex_overlap(geometry::Hexagon hex, SquareGrid &unit_grid);

   public:
    std::unique_ptr<HexGrid> create_hexagonal_grid_from_image(const cv::Mat &floor_img, double tile_size,
                                                              double threshold);

    std::unique_ptr<HexGrid> create_hexagonal_grid_from_square_grid(SquareGrid unit_grid, double tile_size,
                                                                    double threshold);

    std::unique_ptr<HexGrid> create_hexagonal_neighborhood(geometry::Point p, int half_height, int half_width,
                                                           double grid_quanta, HexGrid &hex_grid,
                                                           std::function<bool(geometry::Point)> check_collision);
};

class SquareGridFactory : public GridFactory {
   public:
    std::unique_ptr<SquareGrid> create_square_grid_from_image(const cv::Mat &floor_image, int tile_size,
                                                              double threshold);
};

}  // namespace grid

#include "grid.tpp"
