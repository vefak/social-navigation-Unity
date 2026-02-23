#include "robot.h"

#include <limits>

#include "geometry.h"

namespace robot {

bool CircleRobot::check_collision(geometry::Point position, double theta,
                                  graph_util::NearestObstacleMap& nearest_obstacle_map) const {
    geometry::Point p_nearest = nearest_obstacle_map.find_nearest_obstacle(position);
    return geometry::dist(position, p_nearest) < robot_radius;
}

bool RectangularRobot::check_collision(geometry::Point position, double theta,
                                       graph_util::NearestObstacleMap& nearest_obstacle_map) const {
    for (const geometry::Circle& circle : circle_skeleton) {
        geometry::Point p_circle = position + circle.center.rotate(theta) * length;
        geometry::Point p_nearest = nearest_obstacle_map.find_nearest_obstacle(p_circle);

        if (geometry::dist(p_circle, p_nearest) < circle.radius * length) {
            return true;
        }
    }

    return false;
}

bool CircleRobot::check_collision(geometry::Point position, double theta, State state, const Human& human) const {
    double offset_angle = (position - state.position).angle();
    double contraction = human.get_contraction(state.orientation, offset_angle);

    double distance = geometry::dist(position, state.position);
    double adjusted_distance = (distance - robot_radius) / contraction;

    return adjusted_distance < human.collision_radius;
}

bool RectangularRobot::check_collision(geometry::Point position, double theta, State state, const Human& human) const {
    for (const geometry::Circle& circle : circle_skeleton) {
        geometry::Point p_circle = position + circle.center.rotate(theta) * length;

        double offset_angle = (p_circle - state.position).angle();
        double contraction = human.get_contraction(state.orientation, offset_angle);

        double distance = geometry::dist(p_circle, state.position);
        double adjusted_distance = (distance - circle.radius * length) / contraction;

        if (adjusted_distance < human.collision_radius) {
            return true;
        }
    }

    return false;
}

double CircleRobot::distance_to(geometry::Point position, double theta, const std::vector<State>& dynamic_obstacles,
                                const Human& human) const {
    double min_dist = std::numeric_limits<double>::infinity();

    for (const State& state : dynamic_obstacles) {
        double distance = geometry::dist(position, state.position);
        double adjusted_distance = distance - robot_radius - human.collision_radius;
        min_dist = std::min(min_dist, adjusted_distance);
    }

    return min_dist;
}

double CircleRobot::distance_to(geometry::Point position, double theta,
                                graph_util::NearestObstacleMap& nearest_obstacle_map) const {
    geometry::Point p_obs = nearest_obstacle_map.find_nearest_obstacle(position);
    return geometry::dist(position, p_obs) - robot_radius;
}

double RectangularRobot::distance_to(geometry::Point position, double theta,
                                     const std::vector<State>& dynamic_obstacles, const Human& human) const {
    double min_dist = std::numeric_limits<double>::infinity();

    for (const State& state : dynamic_obstacles) {
        for (const geometry::Circle& circle : circle_skeleton) {
            geometry::Point p_circle = position + circle.center.rotate(theta) * length;
            double distance = geometry::dist(p_circle, state.position);
            double adjusted_distance = distance - circle.radius * length - human.collision_radius;
            min_dist = std::min(min_dist, adjusted_distance);
        }
    }

    return min_dist;
}

double RectangularRobot::distance_to(geometry::Point position, double theta,
                                     graph_util::NearestObstacleMap& nearest_obstacle_map) const {
    double min_dist = std::numeric_limits<double>::infinity();

    for (const geometry::Circle& circle : circle_skeleton) {
        geometry::Point p_circle = position + circle.center.rotate(theta) * length;
        geometry::Point p_obs = nearest_obstacle_map.find_nearest_obstacle(p_circle);

        double distance = geometry::dist(p_circle, p_obs);
        double adjusted_distance = distance - circle.radius * length;

        min_dist = std::min(min_dist, adjusted_distance);
    }

    return min_dist;
}

double Human::get_contraction(double movement_direction, double offset_angle) const {
    double diff = std::abs(movement_direction - offset_angle);
    diff = std::min(diff, 2 * M_PI - diff);

    double contraction = std::sin(diff / 2);  // zero at diff = 0
    contraction = std::max(contraction, contraction_cutoff);

    return contraction;
}

}  // namespace robot