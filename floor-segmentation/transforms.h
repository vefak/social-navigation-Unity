#pragma once
#include <functional>

#include "geometry.h"
#include "robot.h"

namespace transforms {

// Define arbitrary bijective functor
template <typename T>
class VariableTransform {
    std::function<T(T)> f, f_inv;

   public:
    VariableTransform() {}

    VariableTransform(std::function<T(T)> f, std::function<T(T)> f_inv) : f(f), f_inv(f_inv) {}

    VariableTransform inverse() const { return VariableTransform<T>(f_inv, f); }

    VariableTransform compose(const VariableTransform& transform) const {
        VariableTransform result;
        std::function<T(T)> g{f}, g_inv{f_inv}, h{transform.f}, h_inv{transform.f_inv};
        result.f = [=](T obj) { return g(h(obj)); };
        result.f_inv = [=](T obj) { return h_inv(g_inv(obj)); };  // Reverse order
        return result;
    }

    T operator()(T arg) const { return f(arg); }
};

// Define transformations between coordinate systems (x, y, theta)
struct CoordinateTransform {
    VariableTransform<geometry::Point> point_transform, velocity_transform;
    VariableTransform<double> angle_transform, angular_velocity_transform;

    CoordinateTransform();

    CoordinateTransform(const CoordinateTransform& transform);

    CoordinateTransform compose(const CoordinateTransform& transform) const;

    CoordinateTransform inverse() const;

    // For applying transforms on robot state
    robot::State operator()(robot::State state) const;
};

using IdentityTransform = CoordinateTransform;  // Base class implements identity transformation

class PointToPixel : public CoordinateTransform {
   private:
    size_t num_rows, num_cols;
    double x_min, x_max, y_min, y_max;

   public:
    PointToPixel(size_t num_rows, size_t num_cols, double x_min, double x_max, double y_min, double y_max);
};

class FlipYAxisImage : public CoordinateTransform {
   private:
    size_t num_rows;

   public:
    FlipYAxisImage(int num_rows);
};

class FlipAngle : public CoordinateTransform {
   public:
    FlipAngle();
};

class OffsetAngle : public CoordinateTransform {
   private:
    double offset;

   public:
    OffsetAngle(double offset);
};

// Non-invertible Transforms
struct NonInvertibleTransform {
    std::function<geometry::Point(geometry::Point)> point_transform, velocity_transform;
    std::function<double(double)> angle_transform, angular_velocity_transform;

    NonInvertibleTransform();

    template <typename T>
    NonInvertibleTransform compose(const T& transform) const {
        NonInvertibleTransform result;
        std::function<geometry::Point(geometry::Point)> f_point{point_transform}, f_velocity{velocity_transform};
        std::function<double(double)> f_angle{angle_transform}, f_angular_velocity{angular_velocity_transform};

        result.point_transform = [=](geometry::Point point) { return f_point(transform.point_transform(point)); };
        result.velocity_transform = [=](geometry::Point velocity) {
            return f_velocity(transform.velocity_transform(velocity));
        };

        result.angle_transform = [=](double angle) { return f_angle(transform.angle_transform(angle)); };
        result.angular_velocity_transform = [=](double angle) {
            return f_angular_velocity(transform.angular_velocity_transform(angle));
        };

        return result;
    }

    // For applying transforms on robot state
    robot::State operator()(robot::State state) const;
};

class NormAngle : public NonInvertibleTransform {
   public:
    NormAngle();
};

}  // namespace transforms