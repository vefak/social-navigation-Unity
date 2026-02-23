#pragma once
#include <cmath>
#include <ostream>
#include <vector>

namespace geometry {

// Point in 2D space
class Point {
   private:
    static constexpr double err_tol = 1e-8;

   public:
    double x, y;

    Point() : x(0), y(0) {}

    Point(double x, double y) : x(x), y(y) {}

    Point operator+(const Point p) const { return Point(x + p.x, y + p.y); }

    Point operator-(const Point p) const { return Point(x - p.x, y - p.y); }

    Point operator*(double k) const { return Point(k * x, k * y); }

    double operator*(const Point p) const { return x * p.x + y * p.y; }

    Point operator/(double k) const { return Point(x / k, y / k); }

    bool operator==(const Point p) const { return (std::abs(x - p.x) < err_tol && std::abs(y - p.y) < err_tol); }

    Point& operator+=(const Point p) {
        x += p.x;
        y += p.y;
        return *this;
    }

    Point& operator*=(double k) {
        x *= k;
        y *= k;
        return *this;
    }

    Point& operator/=(double k) {
        x /= k;
        y /= k;
        return *this;
    }

    double angle() const { return atan2(y, x); }

    double norm() const { return sqrt(x * x + y * y); }

    Point normalize() const {
        double amplitude = std::max(norm(), err_tol);
        return Point(x / amplitude, y / amplitude);
    }

    Point rotate(double theta) const {
        double R[2][2] = {{cos(theta), -sin(theta)}, {sin(theta), cos(theta)}};
        return Point(R[0][0] * x + R[0][1] * y, R[1][0] * x + R[1][1] * y);
    }
};

inline Point direction_vector(double theta) { return Point(cos(theta), sin(theta)); }

double normalize_angle(double theta);

double angle_dist(double alpha, double beta);

inline Point operator*(double k, Point p) { return p * k; }

inline double dist(const Point& A, const Point& B) {
    return std::sqrt((A.x - B.x) * (A.x - B.x) + (A.y - B.y) * (A.y - B.y));
}

// The z-coordinate of the inner product
inline double cross(Point A, Point B) { return A.x * B.y - B.x * A.y; }

std::ostream& operator<<(std::ostream& os, const Point& p);

struct LineSegment {
    Point start, finish;
    LineSegment(Point start, Point finish) : start(start), finish(finish) {}

    Point proj(Point p) const;

    double length() { return (finish - start).norm(); }

    std::vector<Point> sample(int num_samples);
};

enum ROTATION { CCW = 1, CW = -1 };

struct Arc {
    Point start, finish;
    Point center;
    double radius;
    ROTATION direction;

    Arc(Point center, double radius, Point start, Point finish, ROTATION direction)
        : center(center), radius(radius), start(start), finish(finish), direction(direction) {}

    double length();

    std::vector<Point> sample(int num_samples);

    std::vector<double> sample_tangent_angle(int num_samples);
};

struct Hexagon {
    Point center;
    double radius;

    Hexagon(Point center, double radius) : center(center), radius(radius) {}

    double area() const { return radius * radius * sqrt(3) * 1.5; }

    Point proj(Point p) const;

    LineSegment common_edge(const Hexagon& hex) const;
};

struct UnitEdge;

// 1x1 grid square at (row, col)
struct UnitSquare {
    int row, col;

    UnitSquare() = default;

    UnitSquare(int row, int col) : row(row), col(col) {}

    std::vector<UnitEdge> get_edges() const;
};

enum SquareSide { LEFT = 0, BOTTOM = 1, RIGHT = 2, TOP = 3 };

struct UnitEdge {
    UnitSquare usq;
    SquareSide side;

    UnitEdge() = default;

    UnitEdge(UnitSquare usq, SquareSide side) : usq(usq), side(side) {}
};

// Points listed in CCW order
struct Polygon {
    std::vector<Point> points;

    Polygon() {}

    Polygon(int num_faces) { points.resize(num_faces); }

    Polygon(UnitSquare usq);

    Polygon(Hexagon hex);

    // Find the area of the clipped polygon via triangulation
    double area() const;
};

// Intersection area between hexagon and unit grid square
Polygon intersection(UnitSquare usq, Hexagon hex);

// Integer valued point (grid point)
struct IntegerPoint {
    int x, y;

    IntegerPoint() : x(0), y(0) {}

    IntegerPoint(int x, int y) : x(x), y(y) {}
};

// Define integer valued point via cubic representation
struct IntegerPointCubic {
    int q, r, s;

    IntegerPointCubic() : q(0), r(0), s(0) {}

    IntegerPointCubic(int q, int r, int s) : q(q), r(r), s(s) {}

    IntegerPointCubic(IntegerPoint p) {
        int col = 2 * p.x + (p.y & 1);
        int row = p.y / 2;
        q = col;
        r = row - (col - (col & 1)) / 2;
        s = -q - r;
    }
};

inline int hex_dist(IntegerPointCubic A, IntegerPointCubic B) {
    return (std::abs(A.q - B.q) + std::abs(A.r - B.r) + std::abs(A.s - B.s)) / 2;
}

struct Circle {
    geometry::Point center;
    double radius;
};

}  // namespace geometry
