#pragma once
#include <fstream>
#include <functional>
#include <string>

#include "database.h"
#include "geometry.h"
#include "graph_util.h"
#include "grid.h"
#include "robot.h"
#include "util.h"

namespace planning {

using namespace grid;

// Abstract definition of planner

struct Path {
    bool valid;
    std::vector<geometry::Point> points;
    std::vector<double> angles;

    double length();
};

struct PlannerOutput {
    bool found_path;
    std::vector<geometry::Point> path;
};

PlannerOutput load_planner_output(std::filesystem::path path);

geometry::Point nearest_path_point(const std::vector<geometry::Point>& path, geometry::Point p);

struct PlannerInput {
    geometry::Point start, goal;

    PlannerInput(geometry::Point start, geometry::Point goal) : start(start), goal(goal) {}
};

// A* planning

struct SearchTree {
    std::set<HexCell*> closed_nodes;
    std::set<HexCell*> open_nodes;
};

template <int NumRings>
struct OrientedSearchTree {
    std::set<OrientedHex<NumRings>*> closed_nodes;
    std::set<OrientedHex<NumRings>*> open_nodes;
};

struct GridSearchOutput : public PlannerOutput {
    std::vector<HexCell*> cells_on_path;
    SearchTree search_tree;
};

template <int NumRings>
struct OrientedGridSearchOutput : public PlannerOutput {
    std::vector<OrientedHex<NumRings>*> cells_on_path;
    OrientedSearchTree<NumRings> search_tree;
};

struct HeuristicParams {
    double inf, tile_size;
    bool avoid_walls;       // If true, heuristic value is infinity if next to wall
    bool valley_potential;  // If true, use attractive potential towards Voronoi skeleton
    std::vector<int> potential;
};

// Returns zero by default
class Heuristic {
   protected:
    double wall_penalty(HexCell* node) const;
    double valley_potential(HexCell* node) const;

   public:
    HeuristicParams params;

    virtual double operator()(HexCell* node, HexCell* goal) const {
        return wall_penalty(node) + valley_potential(node);
    };

    Heuristic(HeuristicParams params) : params(params) {}
};

// Euclidean distance to goal
struct EuclideanDistance : public Heuristic {
    double operator()(HexCell* node, HexCell* goal) const override;

    EuclideanDistance(HeuristicParams params) : Heuristic(params) {}
};

// Hexagonal distance to goal
struct HexagonalDistance : public Heuristic {
    double operator()(HexCell* node, HexCell* goal) const override;

    HexagonalDistance(HeuristicParams params) : Heuristic(params) {}
};

struct OrientedHeuristicParams {
    HeuristicParams linear_params;
    double angle_weight;
};

template <int NumRings>
class OrientedHeuristic {
   protected:
    double wall_penalty(OrientedHex<NumRings>* node) const;
    double valley_potential(OrientedHex<NumRings>* node) const;

   public:
    OrientedHeuristicParams params;

    virtual double operator()(OrientedHex<NumRings>* node, OrientedHex<NumRings>* goal) const {
        return wall_penalty(node) + valley_potential(node);
    };

    OrientedHeuristic(OrientedHeuristicParams params) : params(params) {}
};

// Euclidean distance to goal
template <int NumRings>
struct OrientedEuclideanDistance : public OrientedHeuristic<NumRings> {
    double operator()(OrientedHex<NumRings>* node, OrientedHex<NumRings>* goal) const override;

    using OrientedHeuristic<NumRings>::OrientedHeuristic;
};

// Hexagonal distance to goal
template <int NumRings>
struct OrientedHexagonalDistance : public OrientedHeuristic<NumRings> {
    double operator()(OrientedHex<NumRings>* node, OrientedHex<NumRings>* goal) const override;

    using OrientedHeuristic<NumRings>::OrientedHeuristic;
};

struct GridSearchInput : public PlannerInput {
    HexCell *hex_start, *hex_goal;
    const Heuristic& heuristic;

    GridSearchInput(HexCell* hex_start, HexCell* hex_goal, const Heuristic& heuristic)
        : hex_start(hex_start),
          hex_goal(hex_goal),
          heuristic(heuristic),
          PlannerInput(geometry::Point(hex_start->x, hex_start->y), geometry::Point(hex_goal->x, hex_goal->y)) {}

    GridSearchInput(geometry::Point start, geometry::Point goal, HexGrid& hex_grid, const Heuristic& heuristic)
        : heuristic(heuristic), PlannerInput(start, goal) {
        hex_start = hex_grid.nearest_hexagon(start);
        hex_goal = hex_grid.nearest_hexagon(goal);
    }
};

class GridBasedPlanner {
   protected:
    void path_reconstruction(GridSearchOutput& output, HexCell* start, HexCell* goal, std::vector<HexCell*>& backtrack,
                             int points_per_segment = 2) const;

   public:
    HexGrid& hex_grid;

    GridBasedPlanner(HexGrid& hex_grid) : hex_grid(hex_grid) {}
};

struct AStarPlanner : public GridBasedPlanner {
    using GridBasedPlanner::GridBasedPlanner;

    GridSearchOutput find_path(GridSearchInput input);
};

template <int NumRings>
struct OrientedAStarInput : public PlannerInput {
    static constexpr int N = 6;
    OrientedHex<NumRings>*hex_start, *hex_goal;
    const OrientedHeuristic<NumRings>& heuristic;
    std::function<double(double)> angle_cost_function;
    double theta_start, theta_goal;

    OrientedAStarInput(OrientedHex<NumRings>* hex_start, OrientedHex<NumRings>* hex_goal,
                       const OrientedHeuristic<NumRings>& heuristic, std::function<double(double)> angle_cost_function)
        : PlannerInput({hex_start->cell->x, hex_start->cell->y}, {hex_goal->cell->x, hex_goal->cell->y}),
          hex_start(hex_start),
          hex_goal(hex_goal),
          heuristic(heuristic),
          angle_cost_function(angle_cost_function),
          theta_start(hex_start->orientation),
          theta_goal(hex_goal->orientation) {}

    OrientedAStarInput(geometry::Point start, geometry::Point goal, HexGrid& hex_grid,
                       const OrientedHeuristic<NumRings>& heuristic, std::function<double(double)> angle_cost_function,
                       OrientedHexGrid<NumRings>& oriented_grid, double theta_start, double theta_goal)
        : PlannerInput(start, goal), heuristic(heuristic), angle_cost_function(angle_cost_function) {
        // Due to Hex - OrientedHex type mismatch we have to use index logic
        HexCell* hexagon_start = hex_grid.nearest_hexagon(start);
        HexCell* hexagon_goal = hex_grid.nearest_hexagon(goal);

        size_t theta_start_idx = oriented_grid.nearest_angle_index(theta_start);
        size_t theta_goal_idx = oriented_grid.nearest_angle_index(theta_goal);

        this->theta_start = oriented_grid.angles[theta_start_idx];
        this->theta_goal = oriented_grid.angles[theta_goal_idx];

        size_t hex_start_idx = theta_start_idx * oriented_grid.stride() + hexagon_start->raw_idx;
        size_t hex_goal_idx = theta_goal_idx * oriented_grid.stride() + hexagon_goal->raw_idx;

        hex_start = oriented_grid.oriented_map[hex_start_idx];
        hex_goal = oriented_grid.oriented_map[hex_goal_idx];
    }
};

template <int NumRings>
struct OrientedGridBasedPlanner {
   protected:
    void path_reconstruction(OrientedGridSearchOutput<NumRings>& output, OrientedHex<NumRings>* start,
                             OrientedHex<NumRings>* goal, std::vector<OrientedHex<NumRings>*>& backtrack,
                             int points_per_segment = 2) const;

   public:
    OrientedHexGrid<NumRings>& oriented_grid;

    OrientedGridBasedPlanner(OrientedHexGrid<NumRings>& oriented_grid) : oriented_grid(oriented_grid) {}
};

template <int NumRings>
struct OrientedAStarPlanner : public OrientedGridBasedPlanner<NumRings> {
    graph_util::NearestObstacleMap& nearest_obstacle_map;
    robot::Robot& robot;

    // Evaluates collisions lazily
    bool check_collision(OrientedHex<NumRings>* hex);

    // HexGrid and VornoiMap are needed for collision checking
    OrientedAStarPlanner(OrientedHexGrid<NumRings>& oriented_grid, robot::Robot& robot,
                         graph_util::NearestObstacleMap& nearest_obstacle_map)
        : OrientedGridBasedPlanner<NumRings>(oriented_grid), robot(robot), nearest_obstacle_map(nearest_obstacle_map) {}

    OrientedGridSearchOutput<NumRings> find_path(const OrientedAStarInput<NumRings>& input);
};

// Theta* planning

struct ThetaStarInput : public GridSearchInput {
    int points_per_segment;

    ThetaStarInput(HexCell* hex_start, HexCell* hex_goal, const Heuristic& heuristic, int points_per_segment = 20)
        : GridSearchInput(hex_start, hex_goal, heuristic), points_per_segment(points_per_segment) {}

    ThetaStarInput(geometry::Point start, geometry::Point goal, HexGrid& hex_grid, const Heuristic& heuristic,
                   int points_per_segment = 20)
        : GridSearchInput(start, goal, hex_grid, heuristic), points_per_segment(points_per_segment) {}
};

struct ThetaStarPlanner : public GridBasedPlanner {
    using GridBasedPlanner::GridBasedPlanner;

    GridSearchOutput find_path(ThetaStarInput input);
};

std::function<double(double)> get_angle_cost_function(std::string cost_function, double angle_cost_multiplier,
                                                      double angle_sensitivity, double angle_deadzone,
                                                      double maximum_angle, double penalty);

}  // namespace planning

#include "planning.tpp"