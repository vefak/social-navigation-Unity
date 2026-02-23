#include "transforms.h"

namespace transforms {

// Invertible transforms
CoordinateTransform::CoordinateTransform() {
    auto point_identity = [](geometry::Point point) { return point; };
    auto scalar_identity = [](double scalar) { return scalar; };

    point_transform = VariableTransform<geometry::Point>(point_identity, point_identity);
    velocity_transform = VariableTransform<geometry::Point>(point_identity, point_identity);
    angle_transform = VariableTransform<double>(scalar_identity, scalar_identity);
    angular_velocity_transform = VariableTransform<double>(scalar_identity, scalar_identity);
}

CoordinateTransform::CoordinateTransform(const CoordinateTransform& transform) {
    point_transform = transform.point_transform;
    velocity_transform = transform.velocity_transform;
    angle_transform = transform.angle_transform;
    angular_velocity_transform = transform.angular_velocity_transform;
}

CoordinateTransform CoordinateTransform::compose(const CoordinateTransform& transform) const {
    CoordinateTransform result;
    result.point_transform = point_transform.compose(transform.point_transform);
    result.velocity_transform = velocity_transform.compose(transform.velocity_transform);
    result.angle_transform = angle_transform.compose(transform.angle_transform);
    result.angular_velocity_transform = angular_velocity_transform.compose(transform.angular_velocity_transform);
    return result;
}

CoordinateTransform CoordinateTransform::inverse() const {
    CoordinateTransform result;
    result.point_transform = point_transform.inverse();
    result.velocity_transform = velocity_transform.inverse();
    result.angle_transform = angle_transform.inverse();
    result.angular_velocity_transform = angular_velocity_transform.inverse();
    return result;
}

robot::State CoordinateTransform::operator()(robot::State state) const {
    return robot::State{.position = point_transform(state.position),
                        .velocity = velocity_transform(state.velocity),
                        .orientation = angle_transform(state.orientation),
                        .angular_velocity = angular_velocity_transform(state.angular_velocity)};
}

PointToPixel::PointToPixel(size_t num_rows, size_t num_cols, double x_min, double x_max, double y_min, double y_max)
    : CoordinateTransform(),
      num_rows(num_rows),
      num_cols(num_cols),
      x_min(x_min),
      x_max(x_max),
      y_min(y_min),
      y_max(y_max) {
    point_transform = VariableTransform<geometry::Point>(
        [x_min, x_max, y_min, y_max, num_rows, num_cols](geometry::Point point) {
            const double x_scale = x_max - x_min;
            const double y_scale = y_max - y_min;

            point.x = (point.x - x_min) * (num_cols / x_scale);
            point.y = (point.y - y_min) * (num_rows / y_scale);

            return point;
        },
        [x_min, x_max, y_min, y_max, num_rows, num_cols](geometry::Point point) {
            const double x_scale = x_max - x_min;
            const double y_scale = y_max - y_min;

            point.x = x_min + point.x * (x_scale / num_cols);
            point.y = y_min + point.y * (y_scale / num_rows);

            return point;
        });
    velocity_transform = VariableTransform<geometry::Point>(
        [x_min, x_max, y_min, y_max, num_rows, num_cols](geometry::Point velocity) {
            const double x_scale = x_max - x_min;
            const double y_scale = y_max - y_min;

            velocity.x = velocity.x * (num_cols / x_scale);
            velocity.y = velocity.y * (num_rows / y_scale);

            return velocity;
        },
        [x_min, x_max, y_min, y_max, num_rows, num_cols](geometry::Point velocity) {
            const double x_scale = x_max - x_min;
            const double y_scale = y_max - y_min;

            velocity.x = velocity.x * (x_scale / num_cols);
            velocity.y = velocity.y * (y_scale / num_rows);

            return velocity;
        });
}

FlipYAxisImage::FlipYAxisImage(int num_rows) : CoordinateTransform(), num_rows(num_rows) {
    auto flip_y_axis = [num_rows](geometry::Point point) {
        point.y = num_rows - point.y - 1;
        return point;
    };
    point_transform = VariableTransform<geometry::Point>(flip_y_axis, flip_y_axis);
}

FlipAngle::FlipAngle() : CoordinateTransform() {
    auto negate = [](double scalar) { return -scalar; };
    angle_transform = VariableTransform<double>(negate, negate);
    angular_velocity_transform = VariableTransform<double>(negate, negate);
}

OffsetAngle::OffsetAngle(double offset) : CoordinateTransform(), offset(offset) {
    angle_transform = VariableTransform<double>([offset](double scalar) { return scalar + offset; },
                                                [offset](double scalar) { return scalar - offset; });
}

// Non-invertable transforms
NonInvertibleTransform::NonInvertibleTransform() {
    auto point_identity = [](geometry::Point point) { return point; };
    auto scalar_identity = [](double scalar) { return scalar; };

    point_transform = point_identity;
    velocity_transform = point_identity;
    angle_transform = scalar_identity;
    angle_transform = scalar_identity;
}

robot::State NonInvertibleTransform::operator()(robot::State state) const {
    return robot::State{.position = point_transform(state.position),
                        .velocity = velocity_transform(state.velocity),
                        .orientation = angle_transform(state.orientation),
                        .angular_velocity = angular_velocity_transform(state.angular_velocity)};
}

NormAngle::NormAngle() : NonInvertibleTransform() {
    // Norm angle within [-pi, pi]
    angle_transform = [](double angle) {
        while (angle < -M_PI) angle += 2 * M_PI;
        while (angle > M_PI) angle -= 2 * M_PI;
        return angle;
    };
}

}  // namespace transforms