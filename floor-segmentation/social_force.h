#pragma once
#include <filesystem>

#include "geometry.h"
#include "graph_util.h"
#include "grid.h"
#include "planning.h"
#include "potential.h"
#include "robot.h"
#include "util.h"

namespace social_force {

struct Trajectory {
    planning::PlannerOutput global_plan;
    std::vector<geometry::Point> path, v_linear;
    std::vector<double> angles, v_angular;

    // Transform path coordinates and velocities into different coordinate system
    void adjust_coordinates(const transforms::CoordinateTransform& transform);

    void save_to_file(const std::filesystem::path& save_path) const;

    Trajectory erase_prefix(size_t steps);
};

struct LocalPlannerParams {
    int max_samples, lookahead;
    double sampling_period, goal_cutoff_distance;
    size_t walk_length, skip_steps;
    bool project_starting_point;
};

struct SocialForceParams {
    double alpha;
    double k_d, k_o, k_lambda;
    bool influence_auxiliary_agents;
};

struct MetricReport {
    double min_distance_from_static;
    double min_distance_from_dynamic;
    double cumulative_social_force;
    double cumulative_static_force;
    double cumulative_dynamic_force;
    double social_work;

    MetricReport accumulate(const MetricReport& new_report);

    MetricReport()
        : min_distance_from_static(std::numeric_limits<double>::infinity()),
          min_distance_from_dynamic(std::numeric_limits<double>::infinity()),
          cumulative_social_force(0),
          cumulative_static_force(0),
          cumulative_dynamic_force(0),
          social_work(0) {}
};

struct FreeWalkOutput {
    Trajectory trajectory;
    MetricReport report;

    FreeWalkOutput() : report() {}
};

struct LocalPlanner {
    robot::State goal_state;
    database::GraphDatabase& graph_db;
    planning::PlannerOutput& global_plan;
    LocalPlannerParams params;
    std::vector<geometry::Point> old_trajectory;

    // Iterate local planner until goal is reached (static obstacles only mode)
    virtual Trajectory get_aggregated_path() = 0;

    virtual FreeWalkOutput free_walk(bool erase_prefix = true) = 0;

    geometry::Point project_onto_old_trajectory(geometry::Point p);

    geometry::Point compute_beacon(geometry::Point p, int lookahead);

    LocalPlanner(database::GraphDatabase& graph_db, robot::State goal_state, planning::PlannerOutput& global_plan,
                 const LocalPlannerParams& params)
        : graph_db(graph_db), goal_state(goal_state), global_plan(global_plan), params(params) {}
};

class DubinsInterpolator {
   private:
    grid::HexGrid& hex_map;

    // Helper function for Dubins subcases
    planning::Path dubins_XSX(geometry::Point& start, geometry::Point& goal, double theta_start, double theta_goal,
                              double turning_radius, int total_samples, geometry::ROTATION rot_dir);

    planning::Path dubins_XYX(geometry::Point& start, geometry::Point& goal, double theta_start, double theta_goal,
                              double turning_radius, int total_samples, geometry::ROTATION rot_dir);

    planning::Path dubins_XSY(geometry::Point& start, geometry::Point& goal, double theta_start, double theta_goal,
                              double turning_radius, int total_samples, geometry::ROTATION rot_dir);

    // Dubins subcases
    planning::Path dubins_LSL(geometry::Point& start, geometry::Point& goal, double theta_start, double theta_goal,
                              double turning_radius, int total_samples);
    planning::Path dubins_RSR(geometry::Point& start, geometry::Point& goal, double theta_start, double theta_goal,
                              double turning_radius, int total_samples);
    planning::Path dubins_LRL(geometry::Point& start, geometry::Point& goal, double theta_start, double theta_goal,
                              double turning_radius, int total_samples);
    planning::Path dubins_RLR(geometry::Point& start, geometry::Point& goal, double theta_start, double theta_goal,
                              double turning_radius, int total_samples);
    planning::Path dubins_LSR(geometry::Point& start, geometry::Point& goal, double theta_start, double theta_goal,
                              double turning_radius, int total_samples);
    planning::Path dubins_RSL(geometry::Point& start, geometry::Point& goal, double theta_start, double theta_goal,
                              double turning_radius, int total_samples);

    // Helper functions for constructing path out of arcs and line segments
    planning::Path combine_parts_XSY(geometry::Arc arc_start, geometry::LineSegment connector, geometry::Arc arc_goal,
                                     int total_samples);

    planning::Path combine_parts_XYX(geometry::Arc arc_start, geometry::Arc arc_connector, geometry::Arc arc_goal,
                                     int total_samples);

    // Check for collisions to ensure validity of Dubins path
    bool is_collision_free(planning::Path path);

   public:
    std::pair<std::vector<geometry::Point>, std::vector<double>> dubins_interpolation(
        geometry::Point start, geometry::Point goal, double theta_start, double theta_goal, double turning_radius,
        int points_to_go);

    DubinsInterpolator(grid::HexGrid& hex_map) : hex_map(hex_map) {}
};

struct ForceReport {
    double force_dynamic;
    double force_static;
    double force_total;
};

struct ODEOutput {
    std::vector<double> state;
    ForceReport report;
};

// Defines dynamics for the Runge Kutta solver
struct Dynamics {
    /* System: x'' = f(t, x)

    Expand into:
    y' = f(x, t)
    x' = y

    */
    virtual ODEOutput ode_func(double t, const std::vector<double>& x) = 0;
};

struct HeadedSFMDynamics : public Dynamics {
    potential::PotentialField& potential_field;
    const robot::Robot& robot;
    const SocialForceParams& sf_params;
    static constexpr size_t x_dim = 6;

    ODEOutput ode_func(double t, const std::vector<double>& x) override;

    HeadedSFMDynamics(potential::PotentialField& potential_field, const robot::Robot& robot,
                      const SocialForceParams& sf_params)
        : potential_field(potential_field), robot(robot), sf_params(sf_params) {}
};

struct SFMGroupDynamics : public Dynamics {
    potential::PotentialField& potential_field;
    std::shared_ptr<potential::DynamicPotential> dynamic_potential;
    const robot::Robot& robot;
    const SocialForceParams& sf_params;
    static constexpr size_t stride = 6;
    bool influence_auxiliary_agents;

    ODEOutput ode_func(double t, const std::vector<double>& x) override;

    SFMGroupDynamics(potential::PotentialField& potential_field,
                     std::shared_ptr<potential::DynamicPotential> dynamic_potential, const robot::Robot& robot,
                     const SocialForceParams& sf_params)
        : potential_field(potential_field), dynamic_potential(dynamic_potential), robot(robot), sf_params(sf_params) {}
};

struct Integrator {
    ForceReport step(double t_old, const std::vector<double>& x_old, double& t_new, std::vector<double>& x_new);

    Dynamics& dynamics;
    double t_step;

    Integrator(Dynamics& dynamics, double t_step) : dynamics(dynamics), t_step(t_step) {}
};

// Local planner based on leap frog integration of SFM dynamics
class SocialForceLocalPlanner : public LocalPlanner {
   protected:
    grid::HexGrid& hex_map;
    const robot::Robot& robot;
    SocialForceParams sf_params;
    potential::PotentialField& potential_field;
    std::shared_ptr<potential::DynamicPotential> dynamic_potential;

   public:
    virtual Trajectory get_aggregated_path() override;

    MetricReport generate_report(const std::vector<double>& x, const std::vector<double>& x_new,
                                 const ForceReport& force_report);

    FreeWalkOutput loop(Integrator& integrator, const std::vector<double>& x_init, const double t_init,
                        int skip_steps_metric_accumulation = 0);

    virtual FreeWalkOutput free_walk(bool erase_prefix = true) override;

    SocialForceLocalPlanner(database::GraphDatabase& graph_db, robot::State goal_state,
                            planning::PlannerOutput& global_plan, const LocalPlannerParams& params,
                            grid::HexGrid& hex_map, potential::PotentialField& potential_field,
                            std::shared_ptr<potential::DynamicPotential> dynamic_potential, const robot::Robot& robot,
                            const SocialForceParams& sf_params)
        : LocalPlanner(graph_db, goal_state, global_plan, params),
          hex_map(hex_map),
          potential_field(potential_field),
          dynamic_potential(dynamic_potential),
          robot(robot),
          sf_params(sf_params) {}
};

// Post-processing local path via path smoothing
struct PathSmoother {
    virtual std::vector<geometry::Point> get_smoothed_path(const std::vector<geometry::Point>& local_path) const = 0;

    PathSmoother() {}
};

struct IdentitySmoother : PathSmoother {
    std::vector<geometry::Point> get_smoothed_path(const std::vector<geometry::Point>& local_path) const override {
        return local_path;
    }

    IdentitySmoother() {}
};

struct GaussianSmoother : PathSmoother {
    int half_window;
    double sigma;
    std::vector<double> coeff;

    std::vector<geometry::Point> get_smoothed_path(const std::vector<geometry::Point>& local_path) const override;

    GaussianSmoother(int half_window, double sigma) : half_window(half_window), sigma(sigma) {
        coeff.resize(2 * half_window + 1);

        for (int i = -half_window; i <= half_window; ++i) {
            coeff[i + half_window] = exp(-i * i / (2 * sigma * sigma));
        }
    }
};

struct LocalAStarParams {
    bool lightweight;
    bool fixed_walk_length;
    bool rim_termination;
    int half_height;
    int half_width;
    double center_slide;
    int lookahead;
    double grid_quanta;
    double downscale_field;
    double stoppage_distance;
    const planning::Heuristic& heuristic;
    const PathSmoother& path_smoother;
    graph_util::NearestObstacleMap& nearest_obstacle_map;

    LocalAStarParams(bool lightweight, bool fixed_walk_length, bool rim_termination, int half_height, int half_width,
                     double center_slide, int lookahead, double grid_quanta, double downscale_field,
                     double stoppage_distance, const planning::Heuristic& heuristic, const PathSmoother& path_smoother,
                     graph_util::NearestObstacleMap& nearest_obstacle_map)
        : lightweight(lightweight),
          fixed_walk_length(fixed_walk_length),
          rim_termination(rim_termination),
          half_height(half_height),
          half_width(half_width),
          center_slide(center_slide),
          lookahead(lookahead),
          grid_quanta(grid_quanta),
          downscale_field(downscale_field),
          stoppage_distance(stoppage_distance),
          heuristic(heuristic),
          path_smoother(path_smoother),
          nearest_obstacle_map(nearest_obstacle_map) {}
};

struct LocalAStar : public SocialForceLocalPlanner {
    const robot::Human& human;
    const LocalAStarParams lp_params;

    MetricReport generate_report(const Trajectory& trajectory, std::vector<robot::State> dynamic_obstacles);

    void path_reconstruction(Trajectory& output, grid::HexCell* start, grid::HexCell* goal,
                             const std::vector<grid::HexCell*>& backtrack);

    double eval_field(geometry::Point position);

    geometry::Point eval_force(geometry::Point position);

    virtual Trajectory local_planner(std::vector<robot::State> local_state);

    FreeWalkOutput free_walk(bool erase_prefix = true) override;

    bool termination_condition(grid::HexCell* cell);

    LocalAStar(database::GraphDatabase& graph_db, robot::State goal_state, planning::PlannerOutput& global_plan,
               const LocalPlannerParams& params, grid::HexGrid& hex_map, potential::PotentialField& potential_field,
               std::shared_ptr<potential::DynamicPotential> dynamic_potential, const robot::Robot& robot,
               const robot::Human& human, const SocialForceParams& sf_params, const LocalAStarParams& lp_params)
        : SocialForceLocalPlanner(graph_db, goal_state, global_plan, params, hex_map, potential_field,
                                  dynamic_potential, robot, sf_params),
          human(human),
          lp_params(lp_params) {}
};

template <int NumRings>
struct OrientedLocalAStarParams {
    const planning::OrientedHeuristic<NumRings>& heuristic;
    std::function<double(double)> angle_cost_function;

    OrientedLocalAStarParams(const planning::OrientedHeuristic<NumRings>& heuristic,
                             std::function<double(double)> angle_cost_function)
        : heuristic(heuristic), angle_cost_function(angle_cost_function) {}
};

template <int NumRings>
struct LocalOrientedAStar : public LocalAStar {
    const OrientedLocalAStarParams<NumRings>& olp_params;

    bool termination_condition(grid::OrientedHex<NumRings>* cell);

    Trajectory local_planner_callback(std::vector<robot::State> local_state, int grid_upscale);

    Trajectory local_planner(std::vector<robot::State> local_state) override {
        return local_planner_callback(local_state, 1);
    }

    void path_reconstruction(Trajectory& output, grid::OrientedHex<NumRings>* start, grid::OrientedHex<NumRings>* goal,
                             const std::vector<grid::OrientedHex<NumRings>*>& backtrack);

    // Lazy evaluation
    bool check_collision(grid::OrientedHex<NumRings>* hex, const std::vector<robot::State>& local_state);

    LocalOrientedAStar(database::GraphDatabase& graph_db, robot::State goal_state, planning::PlannerOutput& global_plan,
                       const LocalPlannerParams& params, grid::HexGrid& hex_map,
                       potential::PotentialField& potential_field,
                       std::shared_ptr<potential::DynamicPotential> dynamic_potential, const robot::Robot& robot,
                       const robot::Human& human, const SocialForceParams& sf_params, const LocalAStarParams& lp_params,
                       const OrientedLocalAStarParams<NumRings>& olp_params)
        : LocalAStar(graph_db, goal_state, global_plan, params, hex_map, potential_field, dynamic_potential, robot,
                     human, sf_params, lp_params),
          olp_params(olp_params) {}
};

}  // namespace social_force

#include "social_force.tpp"