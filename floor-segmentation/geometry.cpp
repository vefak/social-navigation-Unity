#include "geometry.h"

#include <stdexcept>

namespace geometry {

std::vector<UnitEdge> UnitSquare::get_edges() const {
    std::vector<UnitEdge> edges(4);
    for (int i = 0; i < 4; ++i) edges[i] = UnitEdge(*this, static_cast<SquareSide>(i));
    return edges;
}

Polygon::Polygon(UnitSquare usq) {
    points.resize(4);
    points[0] = Point(usq.col, usq.row);
    points[1] = Point(usq.col, usq.row + 1);
    points[2] = Point(usq.col + 1, usq.row + 1);
    points[3] = Point(usq.col + 1, usq.row);
}

Polygon::Polygon(Hexagon hex) {
    points.resize(6);
    for (int i = 0; i < 6; ++i) {
        double angle_rad = M_PI / 6 + i * (M_PI / 3);
        points[i] = Point(hex.center.x + hex.radius * cos(angle_rad), hex.center.y + hex.radius * sin(angle_rad));
    }
}

double Polygon::area() const {
    if (points.empty()) return 0;

    double area = 0;
    Point prev_point = points.back();

    for (const Point& cur_point : points) {
        area += prev_point.x * cur_point.y - cur_point.x * prev_point.y;
        prev_point = cur_point;
    }

    return std::abs(area) / 2;
}

// Check if point lies in the (square-containing) half-plane bounded by edge e
bool is_inside(Point p, UnitEdge e, const double err_tol = 1e-6) {
    switch (e.side) {
        case SquareSide::LEFT:
            return p.x >= e.usq.col - err_tol;
        case SquareSide::BOTTOM:
            return p.y <= e.usq.row + 1 + err_tol;
        case SquareSide::RIGHT:
            return p.x <= e.usq.col + 1 + err_tol;
        case SquareSide::TOP:
            return p.y >= e.usq.row - err_tol;
        default:
            throw std::logic_error("");
    }
}

// Line intersection between line segment and unit square edge
Point line_square_edge_intersection(LineSegment line, UnitEdge e, const double err_tol = 1e-6) {
    if (e.side == SquareSide::LEFT || e.side == SquareSide::RIGHT) {
        if (abs(line.finish.x - line.start.x) < err_tol) return line.finish;  // vertical case

        int x_side = (e.side == SquareSide::RIGHT) ? e.usq.col + 1 : e.usq.col;
        double t = (x_side - line.start.x) / (line.finish.x - line.start.x);

        return Point(x_side, line.start.y + (line.finish.y - line.start.y) * t);
    }

    if (abs(line.finish.y - line.start.y) < err_tol) return line.finish;  // horizontal case

    int y_side = (e.side == SquareSide::BOTTOM) ? e.usq.row + 1 : e.usq.row;
    double t = (y_side - line.start.y) / (line.finish.y - line.start.y);

    return Point(line.start.x + t * (line.finish.x - line.start.x), y_side);
}

Polygon intersection(UnitSquare usq, Hexagon hex) {
    // Set output polygon to the initial hexagon
    Polygon initial_polygon(hex);
    Polygon output_polygon = initial_polygon;

    // Use the Sutherland-Hodgman algorithm to clip the hexagon
    for (const UnitEdge e : usq.get_edges()) {
        Polygon input_polygon(std::move(output_polygon));
        output_polygon.points.clear();

        // CCW pass
        Point prev_point = input_polygon.points.back();

        for (const Point& cur_point : input_polygon.points) {
            Point intersection = line_square_edge_intersection(LineSegment(prev_point, cur_point), e);

            if (is_inside(cur_point, e)) {
                if (!is_inside(prev_point, e)) {
                    output_polygon.points.emplace_back(intersection);
                }
                output_polygon.points.emplace_back(cur_point);
            } else if (is_inside(prev_point, e)) {
                output_polygon.points.emplace_back(intersection);
            }

            prev_point = cur_point;
        }

        if (output_polygon.points.empty()) break;
    }

    // If the polygon is empty, either the intial hex or the square are the overlapping area,
    // or there is no overlap.
    if (output_polygon.points.empty()) {
        bool all_top = true, all_bottom = true, all_left = true, all_right = true;

        for (const auto& point : initial_polygon.points) {
            if (is_inside(point, UnitEdge(usq, SquareSide::LEFT))) all_left = false;
            if (is_inside(point, UnitEdge(usq, SquareSide::BOTTOM))) all_bottom = false;
            if (is_inside(point, UnitEdge(usq, SquareSide::RIGHT))) all_right = false;
            if (is_inside(point, UnitEdge(usq, SquareSide::TOP))) all_top = false;
        }

        if (all_top || all_bottom || all_left || all_right) return Polygon();

        if (initial_polygon.area() < 1.0) return initial_polygon;

        return Polygon(usq);
    }

    return output_polygon;
}

// Projection of point onto line segment
Point LineSegment::proj(Point p) const {
    Point A = start;
    Point B = finish;
    Point C = p;

    Point AB = (B - A).normalize();  // vector AB normed
    Point AB_perp(AB.y, -AB.x);      // normed orthogonal vector

    // Check that projection of C onto AB is between endpoints A & B
    double CAxCD = cross(C - A, AB_perp);  // D is intersection point => CD ~ AB_perp
    double CBxCD = cross(C - B, AB_perp);

    if (CAxCD * CBxCD > 0) {  // same sign
        if (geometry::dist(C, A) < geometry::dist(C, B)) {
            return A;
        } else {
            return B;
        }
    } else {
        // We have to find the intersection between the lines to find the point
        double distance = std::abs((B - C) * AB_perp);
        geometry::Point D = C + distance * AB_perp;

        // Flip sign if D is further from A than C (projection should always be closer)
        if (geometry::dist(D, A) > geometry::dist(C, A)) {
            D = C - distance * AB_perp;
        }

        return D;
    }
}

// Projection of point onto hexagon
Point Hexagon::proj(Point p) const {
    Point vertex[7];  // cyclic for ease of computation
    vertex[0] = vertex[6] = center + Point(radius, 0);

    for (size_t i = 1, angle = M_PI / 3; i < 6; ++i, angle += M_PI / 3) {
        vertex[i] = center + radius * Point(cos(angle), sin(angle));
    }

    // Construct the sides from the angles and compute the distance
    double min_dist = 1e7;
    Point p_proj;

    for (size_t i = 0; i < 6; ++i) {
        LineSegment segment(vertex[i], vertex[i + 1]);
        Point candidate = segment.proj(p);

        if (dist(p, candidate) < min_dist) {
            p_proj = candidate;
            min_dist = dist(p, p_proj);
        }
    }

    return p_proj;
}

// Finds common edge between two skeletons
LineSegment Hexagon::common_edge(const Hexagon& hex) const {
    constexpr double fov = 0.1;  // The amount of leeway given when detecting angle

    // Precompute hexagon vertices and angles connecting hexagon to edge centers
    double angle[6];
    Point vertex[7];  // cyclic for ease of computation

    angle[0] = M_PI / 6;
    vertex[0] = vertex[6] = center + Point(radius, 0);

    for (size_t i = 1; i < 6; ++i) {
        angle[i] = angle[i - 1] + M_PI / 3;
        double phi = angle[i] - M_PI / 6;
        vertex[i] = center + radius * Point(cos(phi), sin(phi));
    }

    // Find phase of vector that connects hexagon centers
    Point v_center = hex.center - center;
    double phase = v_center.angle();

    // Adjust range from [-pi, pi] to [0, 2 pi]
    if (phase < 0) phase += 2 * M_PI;

    // Find the appropriate edge
    for (int i = 0; i < 6; ++i) {
        if (abs(phase - angle[i]) < fov) {
            return LineSegment(vertex[i], vertex[i + 1]);
        }
    }

    throw std::logic_error("Was unable to find common edge between hexagons.");
}

std::vector<Point> LineSegment::sample(int num_samples) {
    std::vector<Point> samples(num_samples);

    for (int i = 0; i < num_samples; ++i) {
        double lambda = static_cast<double>(i) / (num_samples - 1);
        samples[i] = lambda * start + (1 - lambda) * finish;
    }

    return samples;
}

double Arc::length() {
    double theta_start = (start - center).angle();
    double theta_goal = (finish - center).angle();

    double d_theta_ccw = (theta_goal - theta_start) + (theta_goal < theta_start ? 2 * M_PI : 0);
    double d_theta_cw = 2 * M_PI - d_theta_ccw;

    return (direction == ROTATION::CCW) ? radius * d_theta_ccw : radius * d_theta_cw;
}

std::vector<Point> Arc::sample(int num_samples) {
    std::vector<Point> samples(num_samples);
    double theta_start = (start - center).angle();
    double theta_finish = (finish - center).angle();

    // Make sure rotation is consistent with direction
    if (theta_finish < theta_start && direction == ROTATION::CCW) {
        theta_finish += 2 * M_PI;
    }

    if (theta_finish > theta_start && direction == ROTATION::CW) {
        theta_finish -= 2 * M_PI;
    }

    for (int i = 0; i < num_samples; ++i) {
        double lambda = static_cast<double>(i) / (num_samples - 1);
        double theta = lambda * theta_start + (1 - lambda) * theta_finish;
        samples[i] = center + radius * geometry::direction_vector(theta);
    }

    return samples;
}

std::vector<double> Arc::sample_tangent_angle(int num_samples) {
    std::vector<double> tangent_angles(num_samples);
    double theta_start = (start - center).angle();
    double theta_finish = (finish - center).angle();

    // Make sure rotation is consistent with direction
    if (theta_finish < theta_start && direction == ROTATION::CCW) {
        theta_finish += 2 * M_PI;
    }

    if (theta_finish > theta_start && direction == ROTATION::CW) {
        theta_finish -= 2 * M_PI;
    }

    for (int i = 0; i < num_samples; ++i) {
        double lambda = static_cast<double>(i) / (num_samples - 1);
        double theta = lambda * theta_start + (1 - lambda) * theta_finish;
        tangent_angles[i] = normalize_angle(theta + (direction == CCW ? M_PI / 2 : -M_PI / 2));
    }

    return tangent_angles;
}

double normalize_angle(double theta) {
    while (theta < 0) theta += 2 * M_PI;
    while (theta >= 2 * M_PI) theta -= 2 * M_PI;
    return theta;
}

double angle_dist(double alpha, double beta) {
    alpha = normalize_angle(alpha);
    beta = normalize_angle(beta);

    double diff = std::abs(alpha - beta);
    return std::min(diff, 2 * M_PI - diff);
}

std::ostream& operator<<(std::ostream& os, const Point& p) {
    os << "(" << p.x << ", " << p.y << ")";
    return os;
}

}  // namespace geometry
