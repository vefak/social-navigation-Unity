#pragma once

namespace plot {

template <class Container>
void Figure::plot_circles(const Container &container, double shrink, std::tuple<double, double, double> rgb,
                          double dampen, bool dampen_endpoints) {
    const double radius = shrink * hex_grid.spacing.tile_size * upscale * std::sqrt(3) / 2;

    for (auto p_it = container.begin(); p_it != container.end(); ++p_it) {
        geometry::Point current_point = *p_it;
        current_point.x *= upscale;
        current_point.y *= upscale;

        int x_low = std::max(0, static_cast<int>(current_point.x - radius));
        int x_high = std::min(static_cast<int>(current_point.x + radius + 1), upscale * x_max);
        int y_low = std::max(0, static_cast<int>(current_point.y - radius));
        int y_high = std::min(static_cast<int>(current_point.y + radius + 1), upscale * y_max);

        for (int y = y_low; y <= y_high; ++y) {
            for (int x = x_low; x <= x_high; ++x) {
                geometry::Point neighbor(x, y);
                double dist = geometry::dist(current_point, neighbor);
                const double square_diag = sqrt(2);

                if (dist < radius + square_diag / 2) {
                    double alpha = 1 - std::max(0.0, (dist - radius + 0.5 * square_diag) / square_diag);
                    if (dampen_endpoints) {
                        alpha *= dampen;
                    } else if (p_it != container.begin() && std::next(p_it) != container.end()) {
                        alpha *= dampen;
                    }
                    data[y * skip_dim + x * channel_dim] =
                        (1 - alpha) * data[y * skip_dim + x * channel_dim] + alpha * std::get<2>(rgb) * UCHAR_MAX;
                    data[y * skip_dim + x * channel_dim + 1] =
                        (1 - alpha) * data[y * skip_dim + x * channel_dim + 1] + alpha * std::get<1>(rgb) * UCHAR_MAX;
                    data[y * skip_dim + x * channel_dim + 2] =
                        (1 - alpha) * data[y * skip_dim + x * channel_dim + 2] + alpha * std::get<0>(rgb) * UCHAR_MAX;
                }
            }
        }
    }
}

}  // namespace plot
