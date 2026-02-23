#pragma once
#include <functional>
#include <opencv2/opencv.hpp>

#include "geometry.h"
#include "graph_util.h"
#include "grid.h"
#include "planning.h"
#include "potential.h"
#include "social_force.h"

namespace plot {

struct NearestHexes {
    geometry::Point first_nearest, second_nearest;
    double first_distance, second_distance;
    bool first_is_free, second_is_free, vertical;
    int y_first, x_first, y_second, x_second;
};

class Figure {
    grid::HexGrid &hex_grid;
    unsigned char *data;
    int x_max, y_max, column_dim, row_dim, channel_dim, skip_dim, upscale;
    std::tuple<double, double, double> default_rgb;

    // Shift iterators until they point to nearest hexagon to (i, j)
    void iterator_adjust(double i, double j, std::vector<double>::iterator &x_even_it,
                         std::vector<double>::iterator &y_even_it, std::vector<double>::iterator &x_odd_it,
                         std::vector<double>::iterator &y_odd_it);

    // Find first and second nearest hexes
    NearestHexes find_nearest_hexes(double i, double j, std::vector<double>::iterator x_even_it,
                                    std::vector<double>::iterator y_even_it, std::vector<double>::iterator x_odd_it,
                                    std::vector<double>::iterator y_odd_it);

    // Compute convex combination ratio for anti-aliasing
    double anti_aliasing_ratio(NearestHexes nh, const geometry::Point current_point, double force = 3);

    // Iterate and apply callback function
    void multithreaded_iterate_util(std::function<void(int, int, geometry::Point, NearestHexes)> pixelwise_callback);

    // Render hex-grid (obstacles & free-space)
    void render_initial();
    void render_initial(graph_util::ComponentMapHex &component_map);

    // Write data to image
    cv::Mat write_to_image();

    // Plot circles with given centers
    template <class Container>
    void plot_circles(const Container &container, double shrink, std::tuple<double, double, double> rgb,
                      double dampen = 1, bool dampen_endpoints = false);

    void init_params() {
        x_max = std::ceil(std::max(hex_grid.x_coo_even.back(), hex_grid.x_coo_odd.back()));
        y_max = std::ceil(std::max(hex_grid.y_coo_even.back(), hex_grid.y_coo_odd.back()));

        column_dim = upscale * x_max + 1;
        row_dim = upscale * y_max + 1;
        channel_dim = 3;  // BGR
        skip_dim = channel_dim * column_dim;

        data = new unsigned char[row_dim * column_dim * channel_dim];
    }

    // Generate random color tuples
    std::vector<std::tuple<double, double, double>> generate_rgb(size_t num_tuples);

   public:
    Figure(grid::HexGrid &hex_grid, int upscale = 4) : hex_grid(hex_grid), upscale(upscale) {
        init_params();
        render_initial();
    }

    Figure(grid::HexGrid &hex_grid, graph_util::ComponentMapHex &component_map, int upscale = 4,
           std::tuple<double, double, double> default_rgb = {0.9, 0.9, 0.9})
        : hex_grid(hex_grid), upscale(upscale), default_rgb(default_rgb) {
        init_params();
        render_initial(component_map);
    }

    // Pass by value for implementing default behavior more easily
    void plot(const graph_util::VoronoiMap &voronoi_map, std::vector<bool> interior_mask = std::vector<bool>());

    void plot(const graph_util::RoomFeaturePoints &features, std::vector<int> marker_points = {});

    void plot(const graph_util::RoomSegmentation &room_segmentation, std::vector<int> marked_rooms = {},
              double dampen = 0.4);

    // Display the path in red (write directly to image)
    void plot(const planning::PlannerOutput &output, double shrink = 0.4);

    void plot(const social_force::Trajectory &trajectory, double shrink = 0.2);

    void plot(const planning::SearchTree &tree, double shrink = 0.4);

    void plot(const geometry::Point p, double shrink = 0.4);

    void show(std::string fig_title = "");

    void savefig(std::string path);

    ~Figure() { delete[] data; }
};

class SquareFigure {
    const grid::SquareGrid &square_grid;

    cv::Mat write_to_image();

   public:
    void show(std::string fig_title = "");

    void savefig(std::string path);

    SquareFigure(const grid::SquareGrid &square_grid) : square_grid(square_grid) {}
};

class PotentialFieldPlotter {
    int figure_height, figure_width;
    std::filesystem::path path;
    std::string name, extension;
    double stride;
    bool logplot, flip_color_scheme;

   public:
    void save(potential::PotentialField &potential_field, potential::DynamicPotential &dynamic_potential);

    PotentialFieldPlotter(int figure_height, int figure_width, std::filesystem::path path, std::string name,
                          std::string extension, double stride, bool logplot, bool flip_color_scheme)
        : figure_height(figure_height),
          figure_width(figure_width),
          path(path),
          name(name),
          extension(extension),
          stride(stride),
          logplot(logplot),
          flip_color_scheme(flip_color_scheme) {}
};

}  // namespace plot

#include "plot.tpp"
