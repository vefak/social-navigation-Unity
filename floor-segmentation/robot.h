#pragma once
#include "geometry.h"
#include "graph_util.h"
#include "grid.h"

namespace robot {
// Store information regarding the robot

struct State {
    geometry::Point position, velocity;
    double orientation, angular_velocity;
};

struct Human {
    double collision_radius;
    double contraction_cutoff;

    // Formula that computes the directional dilatation
    double get_contraction(double movement_direction, double offset_angle) const;
};

struct Robot {
    double turning_radius, mass, v_max;

    virtual double moment_of_inertia() const = 0;

    virtual bool check_collision(geometry::Point position, double theta,
                                 graph_util::NearestObstacleMap& nearest_obstacle_map) const = 0;

    // Angle from chord length and radius
    double max_turning_angle(double distance) { return 2 * asin(distance / (2 * turning_radius)); }

    // As per distance function similar to the "eggshell" potential
    virtual bool check_collision(geometry::Point position, double theta, State state, const Human& human) const = 0;

    virtual double distance_to(geometry::Point position, double theta, const std::vector<State>& dynamic_obstacles,
                               const Human& human) const = 0;

    virtual double distance_to(geometry::Point position, double theta,
                               graph_util::NearestObstacleMap& nearest_obstacle_map) const = 0;

    Robot(double turning_radius, double mass, double v_max)
        : turning_radius(turning_radius), mass(mass), v_max(v_max) {}
};

struct CircleRobot : public Robot {
    double robot_radius;

    virtual double moment_of_inertia() const override { return 0.5 * mass * robot_radius * robot_radius; }

    bool check_collision(geometry::Point position, double theta,
                         graph_util::NearestObstacleMap& nearest_obstacle_map) const override;

    bool check_collision(geometry::Point position, double theta, State state, const Human& human) const override;

    double distance_to(geometry::Point position, double theta, const std::vector<State>& dynamic_obstacles,
                       const Human& human) const override;

    double distance_to(geometry::Point position, double theta,
                       graph_util::NearestObstacleMap& nearest_obstacle_map) const override;

    CircleRobot(double robot_radius, double turning_radius, double mass, double v_max)
        : Robot(turning_radius, mass, v_max), robot_radius(robot_radius) {}
};

struct RectangularRobot : public Robot {
    double length, width;

    // Approximate collision box via multiple circles
    std::vector<geometry::Circle> circle_skeleton;

    double moment_of_inertia() const override { return mass * (length * length + width * width) / 12; }

    bool check_collision(geometry::Point position, double theta,
                         graph_util::NearestObstacleMap& nearest_obstacle_map) const override;

    bool check_collision(geometry::Point position, double theta, State state, const Human& human) const override;

    double distance_to(geometry::Point position, double theta, const std::vector<State>& dynamic_obstacles,
                       const Human& human) const override;

    double distance_to(geometry::Point position, double theta,
                       graph_util::NearestObstacleMap& nearest_obstacle_map) const override;

    RectangularRobot(double length, double width, double turning_radius, double mass, double v_max,
                     std::vector<geometry::Circle> circle_skeleton)
        : Robot(turning_radius, mass, v_max), length(length), width(width), circle_skeleton(circle_skeleton) {}
};

}  // namespace robot