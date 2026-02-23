#pragma once
#include "graph_util.h"
#include "grid.h"
#include "planning.h"
#include "robot.h"

namespace potential {

// Define various functions of distance used in potential functions
struct DistanceFunction {
    virtual double eval(double distance) = 0;

    virtual double grad(double distance) = 0;
};

struct Conic : DistanceFunction {  // Identity maping over distance
    double eval(double distance) override { return distance; }

    double grad(double distance) override { return 1; }
};

struct Quadratic : DistanceFunction {
    double eval(double distance) override { return distance * distance; }

    double grad(double distance) override { return 2 * distance; }
};

struct Exponential : DistanceFunction {
    double A, B;

    double eval(double distance) override { return A * exp(-distance * distance / B); }

    double grad(double distance) override { return A * exp(-distance * distance / B) * (-2 * distance) / B; }

    Exponential(double A, double B) : A(A), B(B) {}
};

// Map-induced potential field component
struct MapPotential {
    grid::HexGrid& hex_map;
    graph_util::VoronoiMap& voronoi_map;
    graph_util::NearestObstacleMap& nearest_obstacle_map;

    MapPotential(grid::HexGrid& hex_map, graph_util::VoronoiMap& voronoi_map,
                 graph_util::NearestObstacleMap& nearest_obstacle_map)
        : hex_map(hex_map), voronoi_map(voronoi_map), nearest_obstacle_map(nearest_obstacle_map) {}

    virtual double get_potential(geometry::Point p) = 0;

    virtual geometry::Point get_force(geometry::Point p) = 0;
};

struct GridMapPotential : public MapPotential {
    std::shared_ptr<MapPotential> potential_field;
    grid::HexGrid& hex_grid;

    std::vector<double> grid_potential;
    std::vector<geometry::Point> grid_force;

    double get_potential(geometry::Point p) override;

    geometry::Point get_force(geometry::Point p) override;

    GridMapPotential(std::shared_ptr<MapPotential> potential_field, grid::HexGrid& hex_grid)
        : MapPotential(potential_field->hex_map, potential_field->voronoi_map, potential_field->nearest_obstacle_map),
          potential_field(potential_field),
          hex_grid(hex_grid) {
        grid_potential.resize(hex_grid.size());
        grid_force.resize(hex_grid.size());

        for (grid::HexCell& hex : hex_grid.grid_map) {
            if (hex.free) {
                geometry::Point p(hex.x, hex.y);
                grid_potential[hex.raw_idx] = potential_field->get_potential(p);
                grid_force[hex.raw_idx] = potential_field->get_force(p);
            }
        }
    }
};

struct ObstacleRepulsionPotential : MapPotential {
    std::shared_ptr<DistanceFunction> dist_fnc;

    ObstacleRepulsionPotential(grid::HexGrid& hex_map, graph_util::VoronoiMap& voronoi_map,
                               graph_util::NearestObstacleMap& nearest_obstacle_map,
                               std::shared_ptr<DistanceFunction> dist_fnc)
        : MapPotential(hex_map, voronoi_map, nearest_obstacle_map), dist_fnc(dist_fnc) {}

    double get_potential(geometry::Point p) override;

    geometry::Point get_force(geometry::Point p) override;
};

struct VoronoiFieldPotential : MapPotential {
    double alpha, dist_obs_max, dist_obs_max_squared, scale;

    VoronoiFieldPotential(grid::HexGrid& hex_map, graph_util::VoronoiMap& voronoi_map,
                          graph_util::NearestObstacleMap& nearest_obstacle_map, double scale, double alpha,
                          double dist_obs_max)
        : MapPotential(hex_map, voronoi_map, nearest_obstacle_map),
          scale(scale),
          alpha(alpha),
          dist_obs_max(dist_obs_max),
          dist_obs_max_squared(dist_obs_max * dist_obs_max) {}

    double get_potential(geometry::Point p) override;

    geometry::Point get_force(geometry::Point p) override;
};

// Define a goal-induced potential field yielded by the SFM
struct DrivingPotential {
    bool is_evolving;
    std::shared_ptr<DistanceFunction> dist_fnc;

    virtual double get_potential(geometry::Point p) = 0;

    virtual geometry::Point get_force(geometry::Point p) = 0;

    DrivingPotential(bool is_evolving, std::shared_ptr<DistanceFunction> dist_fnc)
        : is_evolving(is_evolving), dist_fnc(dist_fnc) {}
};

struct GridDrivingPotential : public DrivingPotential {
    std::shared_ptr<DrivingPotential> potential_field;
    grid::HexGrid& hex_grid;

    std::vector<double> grid_potential;
    std::vector<geometry::Point> grid_force;

    double get_potential(geometry::Point p) override;

    geometry::Point get_force(geometry::Point p) override;

    GridDrivingPotential(std::shared_ptr<DrivingPotential> potential_field, grid::HexGrid& hex_grid)
        : DrivingPotential(potential_field->is_evolving, potential_field->dist_fnc),
          potential_field(potential_field),
          hex_grid(hex_grid) {
        grid_potential.resize(hex_grid.size());
        grid_force.resize(hex_grid.size());

        for (grid::HexCell& hex : hex_grid.grid_map) {
            if (hex.free) {
                geometry::Point p(hex.x, hex.y);
                grid_potential[hex.raw_idx] = potential_field->get_potential(p);
                grid_force[hex.raw_idx] = potential_field->get_force(p);
            }
        }
    }
};

struct GoalAndPathPotential : DrivingPotential {
    std::vector<geometry::Point> path;
    double goal_attraction, valley_attraction;

    double get_potential(geometry::Point p) override;

    geometry::Point get_force(geometry::Point p) override;

    GoalAndPathPotential(std::vector<geometry::Point> path, double goal_attraction, double valley_attraction,
                         std::shared_ptr<DistanceFunction> dist_fnc)
        : DrivingPotential(false, dist_fnc),
          path(path),
          goal_attraction(goal_attraction),
          valley_attraction(valley_attraction) {}
};

struct LookaheadPotential : DrivingPotential {
    std::vector<geometry::Point> path;
    double waypoint_attraction;
    int window_size, lookahead;

    std::vector<geometry::Point> extract_window(geometry::Point p);

    double get_potential(geometry::Point p) override;

    geometry::Point get_force(geometry::Point p) override;

    LookaheadPotential(std::vector<geometry::Point> path, double waypoint_attraction, int window_size, int lookahead,
                       std::shared_ptr<DistanceFunction> dist_fnc)
        : DrivingPotential(true, dist_fnc),
          path(path),
          waypoint_attraction(waypoint_attraction),
          window_size(window_size),
          lookahead(lookahead) {}
};

struct WindowPotential : DrivingPotential {
    std::vector<geometry::Point> path;
    double waypoint_attraction, cutoff_dist;
    int window_size, counter;

    void update_counter(geometry::Point p);

    double get_potential(geometry::Point p) override;

    geometry::Point get_force(geometry::Point p) override;

    WindowPotential(std::vector<geometry::Point> path, double waypoint_attraction, double cutoff_dist, int window_size,
                    std::shared_ptr<DistanceFunction> dist_fnc)
        : DrivingPotential(true, dist_fnc),
          path(path),
          waypoint_attraction(waypoint_attraction),
          cutoff_dist(cutoff_dist),
          window_size(window_size),
          counter(0) {}
};

// Potential based on dynamic obstacles
struct DynamicPotential {
    std::shared_ptr<DistanceFunction> dist_fnc;
    std::vector<robot::State> local_state;

    virtual double get_potential(geometry::Point p, size_t id) = 0;

    virtual geometry::Point get_force(geometry::Point p, size_t id) = 0;

    std::vector<robot::State> get_nearby_obstacles(size_t id);

    void update_state(std::vector<robot::State> local_state) { this->local_state = local_state; }

    DynamicPotential(std::shared_ptr<DistanceFunction> dist_fnc) : dist_fnc(dist_fnc), local_state() {}
};

// Uniformily repulsive away from obstacles
struct RipplePotential : public DynamicPotential {
    double get_potential(geometry::Point p, size_t id) override;
    geometry::Point get_force(geometry::Point p, size_t id) override;

    using DynamicPotential::DynamicPotential;
};

struct EggshellPotential : public DynamicPotential {
    double cutoff;

    double get_contraction(double movement_orientation, double offset_angle);

    double get_potential(geometry::Point p, size_t id) override;
    geometry::Point get_force(geometry::Point p, size_t id) override;

    EggshellPotential(std::shared_ptr<DistanceFunction> dist_fnc, double cutoff)
        : DynamicPotential(dist_fnc), cutoff(cutoff) {}
};

// List of potential field snapshots through SFM iterations
struct PotentialField {
    std::shared_ptr<MapPotential> map_potential;
    std::shared_ptr<DrivingPotential> driving_potential;
};

};  // namespace potential