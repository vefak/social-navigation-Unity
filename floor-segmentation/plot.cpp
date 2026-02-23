#include "plot.h"

#include <algorithm>
#include <random>
#include <thread>

namespace plot {

void Figure::iterator_adjust(double i, double j, std::vector<double>::iterator &x_even_it,
                             std::vector<double>::iterator &y_even_it, std::vector<double>::iterator &x_odd_it,
                             std::vector<double>::iterator &y_odd_it) {
    // Adjust iterators (pointer walk)
    while (!(std::next(x_even_it) == this->hex_grid.x_coo_even.end()) &&
           std::abs(*x_even_it - j) > std::abs(*std::next(x_even_it) - j)) {
        ++x_even_it;
    }
    while (!(std::next(x_odd_it) == this->hex_grid.x_coo_odd.end()) &&
           std::abs(*x_odd_it - j) > std::abs(*std::next(x_odd_it) - j)) {
        ++x_odd_it;
    }
    while (!(std::next(y_even_it) == this->hex_grid.y_coo_even.end()) &&
           std::abs(*y_even_it - i) > std::abs(*std::next(y_even_it) - i)) {
        ++y_even_it;
    }
    while (!(std::next(y_odd_it) == this->hex_grid.y_coo_odd.end()) &&
           std::abs(*y_odd_it - i) > std::abs(*std::next(y_odd_it) - i)) {
        ++y_odd_it;
    }
}

NearestHexes Figure::find_nearest_hexes(double i, double j, std::vector<double>::iterator x_even_it,
                                        std::vector<double>::iterator y_even_it, std::vector<double>::iterator x_odd_it,
                                        std::vector<double>::iterator y_odd_it) {
    geometry::Point current_point(j, i);
    geometry::Point nearest_even(*x_even_it, *y_even_it);
    geometry::Point nearest_odd(*x_odd_it, *y_odd_it);
    double even_dist = geometry::dist(current_point, nearest_even);
    double odd_dist = geometry::dist(current_point, nearest_odd);

    // Find second nearest point. If nearest is even, 2nd nearest is odd
    // or a neighbor of the nearest even point in the same column.
    NearestHexes nh;
    std::vector<double>::iterator y_it;

    if (even_dist < odd_dist) {
        nh.y_first = 2 * (y_even_it - this->hex_grid.y_coo_even.begin());
        nh.x_first = x_even_it - this->hex_grid.x_coo_even.begin();
        nh.y_second = 2 * (y_odd_it - this->hex_grid.y_coo_odd.begin()) + 1;
        nh.x_second = x_odd_it - this->hex_grid.x_coo_odd.begin();
        y_it = y_even_it;

        nh.first_nearest = nearest_even;
        nh.second_nearest = nearest_odd;
    } else {
        nh.y_first = 2 * (y_odd_it - this->hex_grid.y_coo_odd.begin()) + 1;
        nh.x_first = x_odd_it - this->hex_grid.x_coo_odd.begin();
        nh.y_second = 2 * (y_even_it - this->hex_grid.y_coo_even.begin());
        nh.x_second = x_even_it - this->hex_grid.x_coo_even.begin();
        y_it = y_odd_it;

        nh.first_nearest = nearest_odd;
        nh.second_nearest = nearest_even;
    }

    nh.first_is_free = hex_grid[{nh.y_first, nh.x_first}].free;
    nh.second_is_free = hex_grid[{nh.y_second, nh.x_second}].free;

    // Check other candidate point (up or down)
    geometry::Point direction_vector = current_point - nh.first_nearest;
    double norm = geometry::dist(nh.first_nearest, current_point);
    nh.vertical = false;  // Horizontal if the second nearest is of opposite parity

    if (nh.y_first > 1 && norm > 1e-4) {
        geometry::Point other_candidate(nh.first_nearest.x, *std::prev(y_it));
        double angle = direction_vector.angle();

        if (angle >= -2 * M_PI / 3 && angle <= -M_PI / 3) {
            nh.second_nearest = other_candidate;
            nh.second_is_free = this->hex_grid[{nh.y_first - 2, nh.x_first}].free;
            nh.vertical = true;
            nh.y_second = nh.y_first - 2;
            nh.x_second = nh.x_first;
        }
    }

    if (nh.y_first < hex_grid.num_rows - 2 && norm > 1e-4) {
        geometry::Point other_candidate(nh.first_nearest.x, *std::next(y_it));
        double angle = direction_vector.angle();

        if (angle >= M_PI / 3 && angle <= 2 * M_PI / 3) {
            nh.second_nearest = other_candidate;
            nh.second_is_free = this->hex_grid[{nh.y_first + 2, nh.x_first}].free;
            nh.vertical = true;
            nh.y_second = nh.y_first + 2;
            nh.x_second = nh.x_first;
        }
    }

    nh.first_distance = geometry::dist(current_point, nh.first_nearest);
    nh.second_distance = geometry::dist(current_point, nh.second_nearest);

    return nh;
}

double Figure::anti_aliasing_ratio(NearestHexes nh, const geometry::Point current_point, double force) {
    double alpha = 0.5 + std::min(0.5, force * std::abs(nh.first_distance - nh.second_distance) / (2 * sqrt(2)));

    if (nh.vertical) {
        double y_midpoint = (nh.first_nearest.y + nh.second_nearest.y) / 2;
        alpha = 0.5 + std::min(0.5, force * std::abs(current_point.y - y_midpoint));
    }

    return alpha;
}

void Figure::multithreaded_iterate_util(
    std::function<void(int, int, geometry::Point, NearestHexes)> pixelwise_callback) {
    const int num_threads = std::max((int)std::thread::hardware_concurrency(), 1);
    std::vector<std::thread> threads;

    for (int t = 0; t < num_threads; ++t) {
        threads.emplace_back(std::thread([this, t, num_threads, pixelwise_callback] {
            auto y_odd_it = this->hex_grid.y_coo_odd.begin();
            auto y_even_it = this->hex_grid.y_coo_even.begin();

            int i_start = (this->row_dim - 1) * t / num_threads;
            int i_end = (this->row_dim - 1) * (t + 1) / num_threads;

            for (int i_us = i_start; i_us <= i_end; ++i_us) {
                auto x_even_it = this->hex_grid.x_coo_even.begin();
                auto x_odd_it = this->hex_grid.x_coo_odd.begin();

                for (int j_us = 0; j_us < this->column_dim; ++j_us) {
                    // True coordinates of square center
                    double i = (0.5 + (double)i_us) / this->upscale;
                    double j = (0.5 + (double)j_us) / this->upscale;
                    geometry::Point current_point(j, i);

                    iterator_adjust(i, j, x_even_it, y_even_it, x_odd_it, y_odd_it);

                    NearestHexes nh = find_nearest_hexes(i, j, x_even_it, y_even_it, x_odd_it, y_odd_it);

                    // Operation that we are applying pixel-wise
                    pixelwise_callback(i_us, j_us, current_point, nh);
                }
            }
        }));
    }

    for (auto &th : threads) th.join();
    threads.clear();
}

void Figure::render_initial() {
    multithreaded_iterate_util([&](int i_us, int j_us, geometry::Point cur_point, NearestHexes nh) {
        // Compute non-boolean intensity for anti-aliasing
        double alpha = anti_aliasing_ratio(nh, cur_point);

        // Combine first and second nearest and write pixel to figure
        if (nh.first_is_free == nh.second_is_free) {
            for (int c = 0; c < this->channel_dim; ++c) {
                data[i_us * this->skip_dim + j_us * this->channel_dim + c] = nh.first_is_free * UCHAR_MAX;
            }
        } else {
            double alpha_raw = nh.first_is_free ? alpha : 1 - alpha;
            for (int c = 0; c < this->channel_dim; ++c) {
                data[i_us * this->skip_dim + j_us * this->channel_dim + c] =
                    static_cast<unsigned char>(UCHAR_MAX * alpha_raw);
            }
        }
    });
}

std::vector<std::tuple<double, double, double>> Figure::generate_rgb(size_t num_tuples) {
    // Randomly generate RGB values for each connected component
    std::mt19937 rng;
    std::uniform_real_distribution<double> distr(0, 1);
    std::vector<std::tuple<double, double, double>> rgb(num_tuples);

    for (auto &color : rgb) {
        std::get<0>(color) = distr(rng);
        std::get<1>(color) = distr(rng);
        std::get<2>(color) = distr(rng);
    }

    rgb[0] = default_rgb;  // Ensure that the "interior" obstacle hexes are the same color
    return rgb;
}

void Figure::render_initial(graph_util::ComponentMapHex &component_map) {
    auto rgb = generate_rgb(component_map.num_components());

    multithreaded_iterate_util([&](int i_us, int j_us, geometry::Point cur_point, NearestHexes nh) {
        // Compute non-boolean intensity for anti-aliasing
        double alpha = anti_aliasing_ratio(nh, cur_point);

        // Combine first and second nearest and write pixel to figure
        std::tuple<double, double, double> rgb_first, rgb_second;

        int comp_idx_first = component_map.component_map[nh.y_first * this->hex_grid.num_cols + nh.x_first];
        int comp_idx_second = component_map.component_map[nh.y_second * this->hex_grid.num_cols + nh.x_second];

        rgb_first = nh.first_is_free ? std::tuple<double, double, double>(1, 1, 1) : rgb[comp_idx_first];
        rgb_second = nh.second_is_free ? std::tuple<double, double, double>(1, 1, 1) : rgb[comp_idx_second];

        std::tuple<double, double, double> avg_rgb;
        std::get<0>(avg_rgb) = alpha * std::get<0>(rgb_first) + (1 - alpha) * std::get<0>(rgb_second);
        std::get<1>(avg_rgb) = alpha * std::get<1>(rgb_first) + (1 - alpha) * std::get<1>(rgb_second);
        std::get<2>(avg_rgb) = alpha * std::get<2>(rgb_first) + (1 - alpha) * std::get<2>(rgb_second);

        data[i_us * this->skip_dim + j_us * this->channel_dim] = std::get<0>(avg_rgb) * UCHAR_MAX;
        data[i_us * this->skip_dim + j_us * this->channel_dim + 1] = std::get<1>(avg_rgb) * UCHAR_MAX;
        data[i_us * this->skip_dim + j_us * this->channel_dim + 2] = std::get<2>(avg_rgb) * UCHAR_MAX;
    });
}

void Figure::plot(const graph_util::VoronoiMap &voronoi_map, std::vector<bool> interior_mask) {
    if (interior_mask.empty()) {
        interior_mask.resize(voronoi_map.hex_grid.size(), true);
    }

    multithreaded_iterate_util([&](int i_us, int j_us, geometry::Point cur_point, NearestHexes nh) {
        // Handle the Voronoi plotting (in blue)
        const double force = 1;
        double gamma = 1.0 - std::min(0.99, force * std::abs(nh.first_distance - nh.second_distance) / sqrt(2));

        if (nh.vertical) {
            double y_midpoint = (nh.first_nearest.y + nh.second_nearest.y) / 2;
            gamma = 1.0 - std::min(0.99, force * 2 * std::abs(cur_point.y - y_midpoint));
        }

        if (voronoi_map.root[nh.y_first * static_cast<int>(hex_grid.num_cols) + nh.x_first] !=
                voronoi_map.root[nh.y_second * static_cast<int>(hex_grid.num_cols) + nh.x_second] &&
            nh.first_is_free && interior_mask[nh.y_first * static_cast<int>(hex_grid.num_cols) + nh.x_first]) {
            for (int c = 0; c < this->channel_dim; ++c) {
                data[i_us * this->skip_dim + j_us * this->channel_dim + c] *= (1 - gamma);
            }
            data[i_us * this->skip_dim + j_us * this->channel_dim] += gamma * UCHAR_MAX;
        }
    });
}

void Figure::plot(const graph_util::RoomSegmentation &room_segmentation, std::vector<int> marked_rooms, double dampen) {
    auto rgb = generate_rgb(room_segmentation.rooms.size());

    // Emphasize the rooms of interest
    std::vector<bool> is_marked(rgb.size(), false);
    for (int marker : marked_rooms) {
        is_marked[marker] = true;
    }

    multithreaded_iterate_util([&](int i_us, int j_us, geometry::Point cur_point, NearestHexes nh) {
        // Compute non-boolean intensity for anti-aliasing
        double alpha = anti_aliasing_ratio(nh, cur_point);

        // Combine first and second nearest and write pixel to figure
        std::tuple<double, double, double> rgb_first, rgb_second;

        int room_idx_first = room_segmentation.room_map[nh.y_first * this->hex_grid.num_cols + nh.x_first];
        int room_idx_second = room_segmentation.room_map[nh.y_second * this->hex_grid.num_cols + nh.x_second];

        rgb_first = (nh.first_is_free && room_idx_first != -1) ? rgb[room_idx_first]
                                                               : std::tuple<double, double, double>(0, 0, 0);
        rgb_second = (nh.second_is_free && room_idx_second != -1) ? rgb[room_idx_second]
                                                                  : std::tuple<double, double, double>(0, 0, 0);

        if (nh.first_is_free && room_idx_first != -1) {
            std::tuple<double, double, double> avg_rgb;
            std::get<0>(avg_rgb) = alpha * std::get<0>(rgb_first) + (1 - alpha) * std::get<0>(rgb_second);
            std::get<1>(avg_rgb) = alpha * std::get<1>(rgb_first) + (1 - alpha) * std::get<1>(rgb_second);
            std::get<2>(avg_rgb) = alpha * std::get<2>(rgb_first) + (1 - alpha) * std::get<2>(rgb_second);

            int r_prop = (dampen * std::get<0>(avg_rgb) + (1 - dampen)) * UCHAR_MAX;
            int g_prop = (dampen * std::get<1>(avg_rgb) + (1 - dampen)) * UCHAR_MAX;
            int b_prop = (dampen * std::get<2>(avg_rgb) + (1 - dampen)) * UCHAR_MAX;

            if (is_marked[room_idx_first]) {
                double beta = (1 + dampen) / 2;
                if (room_idx_first == marked_rooms[0] || room_idx_first == marked_rooms.back()) {
                    beta = 1;
                }
                r_prop = beta * UCHAR_MAX;
                g_prop = 0;
                b_prop = 0;
            }

            data[i_us * this->skip_dim + j_us * this->channel_dim] =
                (data[i_us * this->skip_dim + j_us * this->channel_dim] + b_prop) / 2;
            data[i_us * this->skip_dim + j_us * this->channel_dim + 1] =
                (data[i_us * this->skip_dim + j_us * this->channel_dim + 1] + g_prop) / 2;
            data[i_us * this->skip_dim + j_us * this->channel_dim + 2] =
                (data[i_us * this->skip_dim + j_us * this->channel_dim + 2] + r_prop) / 2;
        }
    });
}

void Figure::plot(const planning::PlannerOutput &output, double shrink) {
    plot_circles(output.path, shrink, {1, 0, 0});
}

void Figure::plot(const social_force::Trajectory &trajectory, double shrink) {
    plot_circles(trajectory.path, shrink, {1, 0, 0});
}

void Figure::plot(geometry::Point p, double shrink) {
    std::vector<geometry::Point> points{p};
    plot_circles(points, shrink, {0, 1, 0});
}

void Figure::plot(const planning::SearchTree &tree, double shrink) {
    // Plot closed set points
    std::vector<geometry::Point> closed_points(tree.closed_nodes.size());
    auto it_closed = tree.closed_nodes.begin();

    for (size_t i = 0; it_closed != tree.closed_nodes.end(); ++i, ++it_closed) {
        closed_points[i] = geometry::Point((*it_closed)->x, (*it_closed)->y);
    }

    plot_circles(closed_points, shrink, {0, 1, 0});

    // Plot open set points
    std::vector<geometry::Point> open_points(tree.open_nodes.size());
    auto it_open = tree.open_nodes.begin();

    for (size_t i = 0; it_open != tree.open_nodes.end(); ++i, ++it_open) {
        open_points[i] = geometry::Point((*it_open)->x, (*it_open)->y);
    }

    plot_circles(open_points, shrink, {0, 0, 1});
}

cv::Mat Figure::write_to_image() {
    cv::Mat colored_img(this->row_dim, this->column_dim, CV_8UC3, cv::Scalar(0, 0, 0));
    const int num_threads = std::max((int)std::thread::hardware_concurrency(), 1);
    std::vector<std::thread> threads;

    for (int t = 0; t < num_threads; ++t) {
        threads.push_back(std::thread([&colored_img, this, t, num_threads] {
            for (int i = t; i < this->row_dim; i += num_threads) {
                for (int j = 0; j < this->column_dim; ++j) {
                    cv::Vec3b &color = colored_img.at<cv::Vec3b>(i, j);
                    color[0] = data[i * this->skip_dim + j * this->channel_dim];
                    color[1] = data[i * this->skip_dim + j * this->channel_dim + 1];
                    color[2] = data[i * this->skip_dim + j * this->channel_dim + 2];
                }
            }
        }));
    }

    for (auto &th : threads) th.join();
    return colored_img;
}

void Figure::savefig(std::string path) {
    auto colored_img = write_to_image();
    cv::imwrite(path, colored_img);
}

void Figure::show(std::string fig_title) {
    auto colored_img = write_to_image();
    cv::imshow(fig_title, colored_img);
}

// Marking points allows us to display a path in the room graph
void Figure::plot(const graph_util::RoomFeaturePoints &features, std::vector<int> marker_points) {
    std::vector<geometry::Point> doors, room_centers_marked, room_centers;

    for (grid::HexCell *door : features.doors) {
        doors.push_back({door->x, door->y});
    }

    // Mark markers
    std::vector<bool> is_marked(features.room_centers.size());
    for (auto &marker : marker_points) {
        is_marked[marker] = true;
    }

    for (size_t i = 0; i < features.room_centers.size(); ++i) {
        grid::HexCell *room_center = features.room_centers[i];
        if (is_marked[i]) {
            room_centers_marked.push_back({room_center->x, room_center->y});
        }
        room_centers.push_back({room_center->x, room_center->y});
    }

    plot_circles(doors, 0.6, {0, 1, 0});                           // Green
    plot_circles(room_centers_marked, 2.0, {0, 0, 1}, 0.5, true);  // Transparent Blue
    plot_circles(room_centers, 0.6, {1, 0, 0});                    // Red
}

// Square figure

cv::Mat SquareFigure::write_to_image() {
    const int num_rows = square_grid.num_rows;
    const int num_cols = square_grid.num_cols;
    unsigned char *data = new unsigned char[num_rows * num_cols];

    for (int i = 0; i < num_rows; ++i) {
        for (int j = 0; j < num_cols; ++j) {
            data[i * num_cols + j] = square_grid.grid_map[i * num_cols + j].free * UCHAR_MAX;
        }
    }

    return cv::Mat(num_rows, num_cols, CV_8UC1, data);
}

void SquareFigure::savefig(std::string path) {
    auto colored_img = write_to_image();
    cv::imwrite(path, colored_img);
}

void SquareFigure::show(std::string fig_title) {
    auto colored_img = write_to_image();
    cv::imshow(fig_title, colored_img);
}

void PotentialFieldPlotter::save(potential::PotentialField &potential_field,
                                 potential::DynamicPotential &dynamic_potential) {
    const int num_threads = std::max((int)std::thread::hardware_concurrency(), 1);

    // Figure out image dimensions based on stride
    int num_rows = figure_height / stride;
    int num_cols = figure_width / stride;

    // Reusable buffers
    constexpr size_t N = 4;
    cv::Mat potential_field_image[N];
    double *data[N];

    for (size_t i = 0; i < N; ++i) {
        potential_field_image[i] = cv::Mat(num_rows, num_cols, CV_8UC3, cv::Scalar(0, 0, 0));
        data[i] = new double[num_rows * num_cols];
    }

    std::vector<std::thread> threads(num_threads);

    // Compute potentials
    for (int t = 0; t < num_threads; ++t) {
        threads[t] = std::thread([&, t] {
            for (int i = t; i < num_rows; i += num_threads) {
                for (int j = 0; j < num_cols; ++j) {
                    geometry::Point p(0.5 + j * stride, 0.5 + i * stride);

                    double map_potential = potential_field.map_potential->get_potential(p);
                    double driving_potential = potential_field.driving_potential->get_potential(p);
                    double object_potential = dynamic_potential.get_potential(p, 0);
                    double potential = map_potential + driving_potential + object_potential;

                    if (logplot) {
                        data[0][i * num_cols + j] = std::log(std::max(1e-6, map_potential));
                        data[1][i * num_cols + j] = std::log(std::max(1e-6, driving_potential));
                        data[2][i * num_cols + j] = std::log(std::max(1e-6, object_potential));
                        data[3][i * num_cols + j] = std::log(std::max(1e-6, potential));
                    } else {
                        data[0][i * num_cols + j] = map_potential;
                        data[1][i * num_cols + j] = driving_potential;
                        data[2][i * num_cols + j] = object_potential;
                        data[3][i * num_cols + j] = potential;
                    }
                }
            }
        });
    }

    for (auto &th : threads) th.join();

    // For the purpose of normalization find max accross each potential map
    double max_potential[N], min_potential[N], range[N];
    for (size_t i = 0; i < N; ++i) {
        max_potential[i] = *std::max_element(data[i], data[i] + num_rows * num_cols);
        min_potential[i] = *std::min_element(data[i], data[i] + num_rows * num_cols);
        range[i] = max_potential[i] - min_potential[i];
    }

    for (int t = 0; t < num_threads; ++t) {
        threads[t] = std::thread([&, t] {
            for (size_t k = 0; k < N; ++k) {
                for (int i = t; i < num_rows; i += num_threads) {
                    for (int j = 0; j < num_cols; ++j) {
                        // Fill in the color data
                        cv::Vec3b &color = potential_field_image[k].at<cv::Vec3b>(i, j);

                        color[0] = (data[k][i * num_cols + j] - min_potential[k]) * UCHAR_MAX / range[k];
                        color[1] = (data[k][i * num_cols + j] - min_potential[k]) * UCHAR_MAX / range[k];
                        color[2] = (data[k][i * num_cols + j] - min_potential[k]) * UCHAR_MAX / range[k];

                        if (flip_color_scheme) {
                            // Color scheme is inverted compared to the potential (darker = higher potential)
                            for (int c = 0; c < 3; ++c) {
                                color[c] = UCHAR_MAX - color[c];
                            }
                        }
                    }
                }
            }
        });
    }

    for (auto &th : threads) th.join();

    // Save the snapshots
    cv::imwrite(path / ("map_" + name + "." + extension), potential_field_image[0]);
    cv::imwrite(path / ("driving_" + name + "." + extension), potential_field_image[1]);
    cv::imwrite(path / ("dynamic_" + name + "." + extension), potential_field_image[2]);
    cv::imwrite(path / (name + "." + extension), potential_field_image[3]);

    for (auto &buffer : data) {
        delete[] buffer;
    }
}

}  // namespace plot
