#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <string>

#include "database.h"
#include "geometry.h"
#include "graph_util.h"
#include "grid.h"
#include "nlohmann/json.hpp"
#include "planning.h"
#include "plot.h"
#include "potential.h"
#include "save.h"
#include "social_force.h"
#include "util.h"

/*
TODO:
- Testing
    - Problem: Local A* crashing
        - Crashing in ROS/Unify every time even with no dynamic obstacles
        - Works fine when running from C++
        - Solution: debuggine needed. Debugging ROS is horrible.

    - Make sure SFM configuration is okay in test
        - parameters are okay both ROS and C++
        - proximity stoppage
        - obstacle repulsion is reasonable

    - Writing paper


- Finalization

    - Update the config file with any changes

    - commit everything cleanly for the Nth time


DONE:
    - Fix collision issue in local planner >>>
        - proximity stoppage >>> seems to be working
        - check collision issue  >>> Error was due to parameter accidentally multiplied by zero (local_planner.cpp)

        - Local A* with N=1 not converging
            - Goes outside of the map somehow
            - With same parameters and no relevant changes doesn't work. Why?
            - Mysterious (0, 0) appearing in path and cycling behavior!

            - Answer: Due to planner not having anywhere to move
                - Less of a problem with N=2 than N=1.
                - Occurs when so close to barrier that moving very slightly causes collision.
                - Fix 1: Change way initial location is connected to local grid. Try perturbations.
                - Fix 2.1: Make sure closest hex to beacon is not the starting one (moving backward is necessary).
                - Fix 2.2 (chosen): If closest hex is the start, double the grid size and try again.

    - test multiple start-end configurations >>>
        - NOTE: Shrunk the table
        - Very hard for large table to navigate the narrow passages of the hospital environment
*/

void update_timestamp(std::chrono::time_point<std::chrono::high_resolution_clock>& start,
                      std::chrono::time_point<std::chrono::high_resolution_clock>& finish, bool display_runtime) {
    finish = std::chrono::high_resolution_clock::now();

    if (display_runtime) {
        std::chrono::duration<double> elapsed_time = finish - start;
        std::cout << "Elapsed time: " << elapsed_time.count() << " [s]" << std::endl;
    }
}

int main() {
    // Load floor plan
    const std::filesystem::path root = "..";
    const std::filesystem::path input_dir = root / "input";
    const std::filesystem::path output_dir = root / "output";
    const std::filesystem::path json_dir = root / "json";

    // Input configuration parsing
    std::ifstream in_stream(json_dir / "config.json");
    nlohmann::json data = nlohmann::json::parse(in_stream);

    const std::filesystem::path database_dir = data["json"]["databasePath"];  // Default location for neo4j importing

    // Load floor image
    cv::Mat floor_img = cv::imread(input_dir / data["fileNames"]["inputMapName"]);

    // Construct hex grid
    int tile_size = data["gridCompute"]["tileSize"];
    double threshold = data["gridCompute"]["detectionThreshold"];

    if (data["gridCompute"]["displayRuntime"]) {
        std::cout << "Computing hexagonal grid from floor plan..." << std::endl;
    }

    std::chrono::time_point<std::chrono::high_resolution_clock> start, finish;
    start = std::chrono::high_resolution_clock::now();

    // Generate hex grid from floor map
    auto hex_grid_factory = grid::HexGridFactory();
    std::unique_ptr<grid::HexGrid> hex_grid_ptr =
        hex_grid_factory.create_hexagonal_grid_from_image(floor_img, tile_size, threshold);
    grid::HexGrid& hex_grid = *hex_grid_ptr;

    update_timestamp(start, finish, data["gridCompute"]["displayRuntime"]);

    // Supergrid
    if (data["gridCompute"]["displayRuntime"]) {
        std::cout << "Connecting super hexgrid..." << std::endl;
        start = std::chrono::high_resolution_clock::now();
    }

    constexpr int NumRings = 2;
    grid::SuperHexGrid<NumRings> super_hex_grid(hex_grid);
    update_timestamp(start, finish, data["gridCompute"]["displayRuntime"]);

    // Oriented grid (for oriented A*)
    if (data["gridCompute"]["displayRuntime"]) {
        std::cout << "Constructing oriented grid..." << std::endl;
        start = std::chrono::high_resolution_clock::now();
    }

    grid::OrientedHexGrid<NumRings> oriented_grid(super_hex_grid);
    update_timestamp(start, finish, data["gridCompute"]["displayRuntime"]);

    // Generate json and save hex grid
    save::JsonFactory json_factory(data["json"]["spacing"]);

    if (data["json"]["save"]) {
        json_factory >> hex_grid;
    }

    // Wall segmentation
    if (data["gridCompute"]["displayRuntime"]) {
        std::cout << "Constructing wall map..." << std::endl;
        start = std::chrono::high_resolution_clock::now();
    }

    double jump_penalty = data["gridCompute"]["wallMap"]["jumpPenalty"];
    bool limit_wall_size = data["gridCompute"]["wallMap"]["limitWallSize"];
    int max_wall_size = data["gridCompute"]["wallMap"]["maxWallSize"];

    if (!limit_wall_size) {
        max_wall_size = hex_grid.size() + 1;
    }

    graph_util::ComponentMapHex wall_map = graph_util::WallMap(hex_grid, jump_penalty, max_wall_size);

    update_timestamp(start, finish, data["gridCompute"]["displayRuntime"]);

    if (data["gridCompute"]["displayRuntime"]) {
        std::cout << "Plotting hex map..." << std::endl;
        start = std::chrono::high_resolution_clock::now();
    }

    std::tuple<double, double, double> default_rgb{data["plot"]["defaultRGB"]["R"], data["plot"]["defaultRGB"]["G"],
                                                   data["plot"]["defaultRGB"]["B"]};
    plot::Figure hex_figure(hex_grid, wall_map, data["plot"]["upscale"], default_rgb);  // Colored plot

    update_timestamp(start, finish, data["gridCompute"]["displayRuntime"]);
    std::cout << std::endl;

    // Compute scaling factor (meters to pixels)
    double x_min = data["rescaling"]["xMin"];
    double x_max = data["rescaling"]["xMax"];
    double y_min = data["rescaling"]["yMin"];
    double y_max = data["rescaling"]["yMax"];
    double x_scale = x_max - x_min;
    double y_scale = y_max - y_min;

    double scaling_factor_horizontal = floor_img.cols / x_scale;
    double scaling_factor_vertical = floor_img.rows / y_scale;
    double scaling_factor = (scaling_factor_horizontal + scaling_factor_vertical) / 2;

    // Define point coordinate transform between pixels and unity
    transforms::CoordinateTransform pixel_to_unity;  // Identity
    pixel_to_unity = pixel_to_unity.compose(transforms::OffsetAngle(M_PI / 2));
    pixel_to_unity = pixel_to_unity.compose(transforms::FlipAngle());
    pixel_to_unity = pixel_to_unity.compose(
        transforms::PointToPixel(floor_img.rows, floor_img.cols, x_min, x_max, y_min, y_max).inverse());
    pixel_to_unity = pixel_to_unity.compose(transforms::FlipYAxisImage(floor_img.rows));

    transforms::CoordinateTransform unity_to_pixel = pixel_to_unity.inverse();

    // Start and goal hexes
    geometry::Point start_point(data["robot"]["start"]["position"]["x"], data["robot"]["start"]["position"]["y"]);
    geometry::Point goal_point(data["pathPlanning"]["goal"]["position"]["x"],
                               data["pathPlanning"]["goal"]["position"]["y"]);
    double theta_start = data["robot"]["start"]["theta"];
    double theta_goal = data["pathPlanning"]["goal"]["theta"];

    if (data["rescaling"]["inMeters"]) {
        start_point = unity_to_pixel.point_transform(start_point);
        goal_point = unity_to_pixel.point_transform(goal_point);
        theta_start = unity_to_pixel.angle_transform(theta_start);
        theta_goal = unity_to_pixel.angle_transform(theta_goal);
    }

    // TODO: Load circle skeleton
    std::vector<geometry::Circle> circle_skeleton;
    circle_skeleton.reserve(data["robot"]["circles"].size());

    for (auto& circle : data["robot"]["circles"]) {
        circle_skeleton.push_back(
            geometry::Circle{.center = geometry::Point(circle["x"], circle["y"]), .radius = circle["radius"]});
    }

    // Compute turning radius
    double steering_angle = data["robot"]["steeringAngle"];
    double wheelbase = data["robot"]["wheelbase"];
    double turning_radius = wheelbase / tan(steering_angle);

    // Load robot information
    robot::RectangularRobot robot(data["robot"]["length"],       // m
                                  data["robot"]["width"],        // m
                                  turning_radius,                // rad
                                  data["robot"]["mass"],         // kg
                                  data["robot"]["maxVelocity"],  // m/s
                                  circle_skeleton                // m
    );

    robot::Human human{
        .collision_radius = data["human"]["collisionRadius"],     // m
        .contraction_cutoff = data["human"]["contractionCutoff"]  //
    };

    // We don't scale circle skeleton since the coordinates are normalized
    if (data["rescaling"]["inMeters"]) {
        robot.length *= scaling_factor;
        robot.width *= scaling_factor;
        robot.v_max *= scaling_factor;
        robot.turning_radius *= scaling_factor;

        human.collision_radius *= scaling_factor;
    }

    grid::HexCell* hex_start = hex_grid.nearest_hexagon(start_point);
    grid::HexCell* hex_goal = hex_grid.nearest_hexagon(goal_point);

    // Sanity Check: Nearest hex index
    if (data["pathPlanning"]["nearestHexSanityCheck"]) {
        std::cout << "Start point : (" << start_point.x << ", " << start_point.y << ")" << std::endl;
        std::cout << "Hex start: (" << hex_start->x << ", " << hex_start->y << ") ; Grid indices: (" << hex_start->row
                  << ", " << hex_start->col << ")" << std::endl;

        std::cout << std::endl << "Goal point : (" << goal_point.x << ", " << goal_point.y << ")" << std::endl;
        std::cout << "Hex goal: (" << hex_goal->x << ", " << hex_goal->y << ") ; Grid indices: (" << hex_goal->row
                  << ", " << hex_goal->col << ")" << std::endl;
    }

    // Voronoi segmentation (invoke with component_map_hex instead of wall_map for "classic" Voronoi map)
    if (data["gridCompute"]["displayRuntime"]) {
        std::cout << std::endl << "Computing Voronoi map..." << std::endl;
        start = std::chrono::high_resolution_clock::now();
    }

    graph_util::VoronoiMap voronoi_map(hex_grid, wall_map);

    update_timestamp(start, finish, data["gridCompute"]["displayRuntime"]);

    if (data["json"]["save"]) {
        json_factory >> voronoi_map;  // Save Voronoi map to json
    }

    // Define nearest obstacle map
    std::unique_ptr<graph_util::NearestObstacleMap> nearest_obstacle_map;

    if (data["gridCompute"]["displayRuntime"]) {
        std::cout << "Constructing nearest obstacle map..." << std::endl;
        start = std::chrono::high_resolution_clock::now();
    }

    if (data["robot"]["approximateNearestObstacle"]) {
        nearest_obstacle_map = std::make_unique<graph_util::ApproximateNearestObstacleMap>(hex_grid, voronoi_map);
    } else {
        nearest_obstacle_map = std::make_unique<graph_util::ExactNearestObstacleMap>(hex_grid, voronoi_map);
    }

    update_timestamp(start, finish, data["gridCompute"]["displayRuntime"]);

    // Obtain the rooms
    if (data["gridCompute"]["displayRuntime"]) {
        std::cout << "Room segmentation..." << std::endl;
        start = std::chrono::high_resolution_clock::now();
    }

    // graph_util::RoomSegmentation room_segmentation(voronoi_map);
    graph_util::RoomGraph room_segmentation(voronoi_map);

    if (data["json"]["save"]) {
        json_factory >> room_segmentation;
    }

    update_timestamp(start, finish, data["gridCompute"]["displayRuntime"]);

    // Mask the hexes inside the boundary of the map using the room segmentation map
    std::vector<bool> interior_mask(hex_grid.size());

    for (size_t i = 0; i < interior_mask.size(); ++i) {
        interior_mask[i] = (room_segmentation.room_map[i] != -1);
    }

    // Plot the Voronoi skeleton and color code the rooms, while masking out the exterior pixels
    if (data["gridCompute"]["displayRuntime"]) {
        std::cout << "Plotting Voronoi map and room segmentation map..." << std::endl;
        start = std::chrono::high_resolution_clock::now();
    }

    hex_figure.plot(voronoi_map, interior_mask);
    hex_figure.plot(room_segmentation);
    // hex_figure.plot(room_segmentation, {2, 25, 10, 5, 16, 17, 6});

    update_timestamp(start, finish, data["gridCompute"]["displayRuntime"]);

    // Find the room centers and door crossings
    graph_util::RoomFeaturePoints features = room_segmentation.get_feature_points();
    features.doors.clear();     // Turn off door plotting
    hex_figure.plot(features);  // Mark points along rooms in room graph that lie on the path

    // Define heuristic function for A* / Theta* / Two-level algorithms
    planning::HeuristicParams heuristic_params{
        .inf = data["pathPlanning"]["heuristicParams"]["infinity"],
        .tile_size = static_cast<double>(tile_size),
        .avoid_walls = data["pathPlanning"]["heuristicParams"]["avoidWalls"],
        .valley_potential = data["pathPlanning"]["heuristicParams"]["useValleyPotential"],
        .potential = voronoi_map.center_distance};

    planning::OrientedHeuristicParams oriented_heuristic_params{
        .linear_params = heuristic_params, .angle_weight = data["pathPlanning"]["heuristicParams"]["angleWeight"]};

    std::unique_ptr<planning::Heuristic> heuristic;
    std::unique_ptr<planning::OrientedHeuristic<NumRings>> oriented_heuristic;

    if (data["pathPlanning"]["heuristicParams"]["heuristic"] == "hexagonal") {
        heuristic = std::make_unique<planning::HexagonalDistance>(heuristic_params);
        oriented_heuristic = std::make_unique<planning::OrientedHexagonalDistance<NumRings>>(oriented_heuristic_params);
    } else if (data["pathPlanning"]["heuristicParams"]["heuristic"] == "euclidean") {
        heuristic = std::make_unique<planning::EuclideanDistance>(heuristic_params);
        oriented_heuristic = std::make_unique<planning::OrientedEuclideanDistance<NumRings>>(oriented_heuristic_params);
    } else {
        heuristic = std::make_unique<planning::Heuristic>(heuristic_params);
        oriented_heuristic = std::make_unique<planning::OrientedHeuristic<NumRings>>(oriented_heuristic_params);
    }

    // Define input for planning algorithm
    std::unique_ptr<planning::PlannerOutput> output;

    if (data["gridCompute"]["displayRuntime"]) {
        std::cout << "Path planning..." << std::endl;
        start = std::chrono::high_resolution_clock::now();
    }

    if (data["pathPlanning"]["algorithm"] == "Oriented-A-Star") {
        size_t start_angle_idx = oriented_grid.nearest_angle_index(theta_start);
        size_t goal_angle_idx = oriented_grid.nearest_angle_index(theta_goal);

        size_t start_idx = start_angle_idx * oriented_grid.stride() + hex_start->raw_idx;
        size_t goal_idx = goal_angle_idx * oriented_grid.stride() + hex_goal->raw_idx;

        grid::OrientedHex<NumRings>* oriented_hex_start = &oriented_grid.oriented_map[start_idx];
        grid::OrientedHex<NumRings>* oriented_hex_goal = &oriented_grid.oriented_map[goal_idx];

        // Define angle cost function
        double angle_cost_multiplier = data["pathPlanning"]["OrientedAStarParams"]["angleCostMultiplier"];
        double angle_sensitivity = data["pathPlanning"]["OrientedAStarParams"]["angleSensitivity"];
        double angle_deadzone = data["pathPlanning"]["OrientedAStarParams"]["angleDeadzone"];
        double turnrate_penalty = data["pathPlanning"]["OrientedAStarParams"]["turnratePenalty"];
        double maximum_angle = std::max(1e-4 + M_PI / 6, robot.max_turning_angle(tile_size * sqrt(3)));

        std::function<double(double)> angle_cost_function = planning::get_angle_cost_function(
            data["pathPlanning"]["OrientedAStarParams"]["angleCostFunction"], angle_cost_multiplier, angle_sensitivity,
            angle_deadzone, maximum_angle, turnrate_penalty);

        planning::OrientedAStarInput<NumRings> grid_plan_input(oriented_hex_start, oriented_hex_goal,
                                                               *oriented_heuristic, angle_cost_function);

        planning::OrientedAStarPlanner<NumRings> oriented_a_star_planner(oriented_grid, robot, *nearest_obstacle_map);

        planning::OrientedGridSearchOutput<NumRings> debug_output = oriented_a_star_planner.find_path(grid_plan_input);
        output = std::make_unique<planning::OrientedGridSearchOutput<NumRings>>(debug_output);
    } else {
        planning::GridSearchOutput debug_output;

        if (data["pathPlanning"]["algorithm"] == "A-Star") {
            planning::GridSearchInput grid_plan_input(hex_start, hex_goal, *heuristic);
            planning::AStarPlanner a_star_planner(hex_grid);
            debug_output = a_star_planner.find_path(grid_plan_input);
        }

        else if (data["pathPlanning"]["algorithm"] == "Theta-Star") {
            planning::ThetaStarInput grid_plan_input(hex_start, hex_goal, *heuristic);
            planning::ThetaStarPlanner theta_star_planner(hex_grid);
            debug_output = theta_star_planner.find_path(grid_plan_input);
        }

        if (data["pathPlanning"]["plotSearchTree"]) {
            hex_figure.plot(debug_output.search_tree);
        }

        output = std::make_unique<planning::GridSearchOutput>(debug_output);
    }

    update_timestamp(start, finish, data["gridCompute"]["displayRuntime"]);
    hex_figure.plot(*output);                                             // Plot the (non-smoothed) planned path
    hex_figure.savefig(output_dir / data["fileNames"]["outputMapName"]);  // Save for easier debugging

    // Smooth out the path using a social force algorithm, which respects the physics of motion
    if (output->found_path) {
        std::cout << std::endl << "Global path found." << std::endl;

        if (data["gridCompute"]["displayRuntime"]) {
            std::cout << std::endl << "Social force path smoothing..." << std::endl;
            start = std::chrono::high_resolution_clock::now();
        }

        social_force::LocalPlannerParams params{
            .max_samples = data["localPlanner"]["socialForce"]["linearParams"]["maxSamples"],                   //
            .lookahead = data["localPlanner"]["socialForce"]["linearParams"]["lookahead"],                      //
            .sampling_period = data["localPlanner"]["socialForce"]["linearParams"]["samplingPeriod"],           // s
            .goal_cutoff_distance = data["localPlanner"]["socialForce"]["linearParams"]["goalCutoffDistance"],  // m
            .walk_length = data["localPlanner"]["socialForce"]["freeWalkParams"]["walkLength"],
            .skip_steps = data["localPlanner"]["socialForce"]["freeWalkParams"]["skipSteps"],
            .project_starting_point = data["localPlanner"]["socialForce"]["freeWalkParams"]["projectStartingPoint"]};

        social_force::SocialForceParams sf_params{
            .alpha = data["localPlanner"]["socialForce"]["rotationParams"]["alpha"],        //
            .k_d = data["localPlanner"]["socialForce"]["rotationParams"]["k_d"],            // kg/s
            .k_o = data["localPlanner"]["socialForce"]["rotationParams"]["k_o"],            //
            .k_lambda = data["localPlanner"]["socialForce"]["rotationParams"]["k_lambda"],  // 1 / (kg m)
            .influence_auxiliary_agents = data["localPlanner"]["socialForce"]["influenceAuxiliaryAgents"]};

        if (data["rescaling"]["inMeters"]) {  // if the units are provided in meters rescale them to pixels (and
                                              // undo the scaling later)
            params.goal_cutoff_distance *= scaling_factor;
            sf_params.k_lambda /= scaling_factor;
        }

        // Define potential field
        potential::PotentialField pot_field;
        std::string map_potential_type = data["potentialField"]["mapPotential"]["type"];
        std::string driving_potential_type = data["potentialField"]["drivingPotential"]["type"];

        double A = data["potentialField"]["obstacleRepulsionParams"]["A"];  // N
        double B = data["potentialField"]["obstacleRepulsionParams"]["B"];  // m^2

        if (data["rescaling"]["inMeters"]) {
            A *= scaling_factor;
            B *= (scaling_factor * scaling_factor);
        }

        std::shared_ptr<potential::DistanceFunction> obs_repulsion_fnc = std::make_shared<potential::Exponential>(A, B);

        if (map_potential_type == "VoronoiFieldPotential") {
            double alpha = data["potentialField"]["mapPotential"]["voronoiFieldPotentialParams"]["alpha"];  // m
            double dist_obs_max =
                data["potentialField"]["mapPotential"]["voronoiFieldPotentialParams"]["distObsMax"];  // m

            if (data["rescaling"]["inMeters"]) {
                alpha *= scaling_factor;
                dist_obs_max *= scaling_factor;
            }

            pot_field.map_potential = std::make_shared<potential::VoronoiFieldPotential>(
                hex_grid, voronoi_map, *nearest_obstacle_map,
                data["potentialField"]["mapPotential"]["voronoiFieldPotentialParams"]["scale"], alpha, dist_obs_max);
        } else if (map_potential_type == "ObstacleRepulsionPotential") {
            pot_field.map_potential = std::make_shared<potential::ObstacleRepulsionPotential>(
                hex_grid, voronoi_map, *nearest_obstacle_map, obs_repulsion_fnc);
        }

        if (data["json"]["save"]) {
            json_factory >> pot_field;
        }

        std::shared_ptr<potential::DistanceFunction> dist_fnc;

        if (data["potentialField"]["drivingPotential"]["distFunction"] == "Conic") {
            dist_fnc = std::make_shared<potential::Conic>();
        } else if (data["potentialField"]["drivingPotential"]["distFunction"] == "Quadratic") {
            dist_fnc = std::make_shared<potential::Quadratic>();
        }

        double waypoint_attraction = data["potentialField"]["drivingPotential"]["waypointAttraction"];  // N

        if (data["rescaling"]["inMeters"]) {
            waypoint_attraction *= scaling_factor;
        }

        if (driving_potential_type == "GoalAndPathPotential") {
            double valley_attraction =
                data["potentialField"]["drivingPotential"]["goalAndPathPotentialParams"]["valleyAttraction"];  // N

            if (data["rescaling"]["inMeters"]) {
                valley_attraction *= scaling_factor;
            }

            pot_field.driving_potential = std::make_shared<potential::GoalAndPathPotential>(
                output->path, waypoint_attraction, valley_attraction, dist_fnc);
        } else if (driving_potential_type == "LookaheadPotential") {
            pot_field.driving_potential = std::make_shared<potential::LookaheadPotential>(
                output->path, waypoint_attraction,
                data["potentialField"]["drivingPotential"]["lookaheadPotentialParams"]["windowSize"],
                data["potentialField"]["drivingPotential"]["lookaheadPotentialParams"]["lookahead"], dist_fnc);
        } else if (driving_potential_type == "WindowPotential") {
            double cutoff_dist =
                data["potentialField"]["drivingPotential"]["windowPotentialParams"]["cutoffDist"];  // m

            if (data["rescaling"]["inMeters"]) {
                cutoff_dist *= scaling_factor;
            }

            pot_field.driving_potential = std::make_shared<potential::WindowPotential>(
                output->path, waypoint_attraction, cutoff_dist,
                data["potentialField"]["drivingPotential"]["windowPotentialParams"]["windowSize"], dist_fnc);
        }

        if (data["potentialField"]["precomputePotential"]) {
            pot_field.map_potential = std::make_shared<potential::GridMapPotential>(pot_field.map_potential, hex_grid);
            pot_field.driving_potential =
                std::make_shared<potential::GridDrivingPotential>(pot_field.driving_potential, hex_grid);
        }

        constexpr int NumRingsLocal = 2;

        std::unique_ptr<social_force::LocalPlanner> local_planner;
        std::unique_ptr<social_force::LocalAStarParams> lp_params;
        std::unique_ptr<social_force::OrientedLocalAStarParams<NumRingsLocal>> olp_params;
        std::unique_ptr<planning::Heuristic> local_heuristic;
        std::unique_ptr<planning::OrientedHeuristic<NumRingsLocal>> local_oriented_heuristic;
        std::unique_ptr<social_force::PathSmoother> path_smoother;
        std::shared_ptr<database::GraphDatabase> graph_db;

        robot::State goal_state{.position = {hex_goal->x, hex_goal->y},
                                .velocity = {0, 0},
                                .orientation = theta_goal,
                                .angular_velocity = 0};

        // Construct graph database and dynamic potential
        graph_db = std::make_shared<database::MockDatabase>(start_point);

        // Dynamic potential to be used in social force model
        std::shared_ptr<potential::DynamicPotential> dynamic_potential;

        if (data["potentialField"]["dynamicPotential"]["type"] == "RipplePotential") {
            dynamic_potential = make_shared<potential::RipplePotential>(obs_repulsion_fnc);
        } else {
            double cutoff = data["potentialField"]["dynamicPotential"]["cutoff"];
            dynamic_potential = make_shared<potential::EggshellPotential>(obs_repulsion_fnc, cutoff);
        }

        if (data["localPlanner"]["planner"] == "SFM") {
            local_planner = std::make_unique<social_force::SocialForceLocalPlanner>(
                *graph_db, goal_state, *output, params, hex_grid, pot_field, dynamic_potential, robot, sf_params);
        } else {
            assert(data["localPlanner"]["planner"] == "A-Star" || data["localPlanner"]["planner"] == "Oriented-A-Star");

            bool lightweight = data["localPlanner"]["AStarParams"]["lightweight"];
            double downscale_field = data["localPlanner"]["AStarParams"]["downscaleField"];

            int half_height = data["localPlanner"]["gridParams"]["halfHeight"];
            int half_width = data["localPlanner"]["gridParams"]["halfWidth"];
            double downscale_factor = data["localPlanner"]["gridParams"]["gridDownscaleFactor"];
            double center_slide = data["localPlanner"]["gridParams"]["centerSlide"];

            int lookahead = data["localPlanner"]["AStarParams"]["lookahead"];
            bool fixed_walk_length = data["localPlanner"]["AStarParams"]["fixedWalkLength"];
            bool rim_termination = data["localPlanner"]["AStarParams"]["rimTermination"];

            double stoppage_distance = data["localPlanner"]["AStarParams"]["stoppageDistance"];
            if (data["rescaling"]["inMeters"]) {
                stoppage_distance *= scaling_factor;  // m
            }

            double grid_quanta;

            if (data["localPlanner"]["gridParams"]["adaptive"]) {
                double v_cruise = data["robot"]["cruiseVelocity"];

                if (data["rescaling"]["inMeters"]) {
                    v_cruise *= scaling_factor;  // m/s
                }

                double T_sample = data["localPlanner"]["socialForce"]["linearParams"]["samplingPeriod"];
                grid_quanta = v_cruise * T_sample;
            } else {
                grid_quanta = tile_size * downscale_factor;
            }

            // Define local heuristic
            planning::HeuristicParams local_heuristic_params{
                .inf = data["localPlanner"]["heuristicParams"]["infinity"],
                .tile_size = grid_quanta,
                .avoid_walls = data["localPlanner"]["heuristicParams"]["avoidWalls"],
                .valley_potential = false,  // There is no valley potential for the local grid
                .potential = {}};

            planning::OrientedHeuristicParams local_oriented_heuristic_params{
                .linear_params = local_heuristic_params,
                .angle_weight = data["localPlanner"]["heuristicParams"]["angleWeight"]};

            if (data["localPlanner"]["heuristicParams"]["heuristic"] == "hexagonal") {
                local_heuristic = std::make_unique<planning::HexagonalDistance>(local_heuristic_params);
                local_oriented_heuristic = std::make_unique<planning::OrientedHexagonalDistance<NumRingsLocal>>(
                    local_oriented_heuristic_params);
            } else if (data["localPlanner"]["heuristicParams"]["heuristic"] == "euclidean") {
                local_heuristic = std::make_unique<planning::EuclideanDistance>(local_heuristic_params);
                local_oriented_heuristic = std::make_unique<planning::OrientedEuclideanDistance<NumRingsLocal>>(
                    local_oriented_heuristic_params);
            } else {
                local_heuristic = std::make_unique<planning::Heuristic>(local_heuristic_params);
                local_oriented_heuristic =
                    std::make_unique<planning::OrientedHeuristic<NumRingsLocal>>(local_oriented_heuristic_params);
            }

            // Path smoothing
            if (data["localPlanner"]["AStarParams"]["pathSmoothing"]["algorithm"] == "None") {
                path_smoother = std::make_unique<social_force::IdentitySmoother>();
            } else {
                assert(data["localPlanner"]["AStarParams"]["pathSmoothing"]["algorithm"] == "Gaussian");
                int half_window = data["localPlanner"]["AStarParams"]["pathSmoothing"]["halfWindow"];
                double sigma = data["localPlanner"]["AStarParams"]["pathSmoothing"]["sigma"];

                path_smoother = std::make_unique<social_force::GaussianSmoother>(half_window, sigma);
            }

            lp_params = std::make_unique<social_force::LocalAStarParams>(
                lightweight, fixed_walk_length, rim_termination, half_height, half_width, center_slide, lookahead,
                grid_quanta, downscale_field, stoppage_distance, *local_heuristic, *path_smoother,
                *nearest_obstacle_map);

            double angle_cost_multiplier = data["localPlanner"]["OrientedAStarParams"]["angleCostMultiplier"];
            double angle_sensitivity = data["localPlanner"]["OrientedAStarParams"]["angleSensitivity"];
            double angle_deadzone = data["localPlanner"]["OrientedAStarParams"]["angleDeadzone"];
            double turnrate_penalty = data["pathPlanning"]["OrientedAStarParams"]["turnratePenalty"];
            double maximum_angle = std::max(1e-4 + M_PI / 6, robot.max_turning_angle(tile_size * sqrt(3)));

            std::function<double(double)> angle_cost_function = planning::get_angle_cost_function(
                data["pathPlanning"]["OrientedAStarParams"]["angleCostFunction"], angle_cost_multiplier,
                angle_sensitivity, angle_deadzone, maximum_angle, turnrate_penalty);

            olp_params = std::make_unique<social_force::OrientedLocalAStarParams<NumRingsLocal>>(
                *local_oriented_heuristic, angle_cost_function);

            if (data["localPlanner"]["planner"] == "A-Star") {
                local_planner = std::make_unique<social_force::LocalAStar>(*graph_db, goal_state, *output, params,
                                                                           hex_grid, pot_field, dynamic_potential,
                                                                           robot, human, sf_params, *lp_params);
            } else {
                local_planner = std::make_unique<social_force::LocalOrientedAStar<NumRingsLocal>>(
                    *graph_db, goal_state, *output, params, hex_grid, pot_field, dynamic_potential, robot, human,
                    sf_params, *lp_params, *olp_params);
            }
        }

        social_force::Trajectory trajectory = local_planner->get_aggregated_path();

        update_timestamp(start, finish, data["gridCompute"]["displayRuntime"]);

        if (data["gridCompute"]["displayRuntime"]) {
            std::cout << "Plotting and saving SFM planner output..." << std::endl;
            start = std::chrono::high_resolution_clock::now();
        }

        hex_figure.plot(trajectory);  // Plot the SFM-smoothed path

        // Coordinate conversion before saving to file
        trajectory.adjust_coordinates(pixel_to_unity);

        trajectory.save_to_file(
            output_dir /
            data["fileNames"]["outputTrajName"]);  // Save the trajectory as sequence of points and angle values

        // Save list of potential field snapshots
        if (static_cast<bool>(data["plot"]["potentialFieldSnapshots"]["save"]) &&
            data["potentialField"]["drivingPotential"]["type"] !=
                "WindowPotential") {  // plotting doesnt make sense for window potential
            const std::filesystem::path snapshots_dir = output_dir / "snapshots";

            // Make a "fake" dynamic potential for demonstration purposes
            std::shared_ptr<potential::DynamicPotential> fake_dynamic_potential =
                make_shared<potential::RipplePotential>(obs_repulsion_fnc);

            // TODO:Design custom local state and update dynamic potential
            std::vector<robot::State> fake_states(5);

            for (auto& state : fake_states) {
                state.velocity = {0, 0};
                state.angular_velocity = 0;
                state.orientation = 0;
            }

            fake_states[0].position = {200, 300};
            fake_states[1].position = {370, 410};
            fake_states[2].position = {355, 420};
            fake_states[3].position = {350, 400};
            fake_states[4].position = {700, 400};

            fake_dynamic_potential->update_state(fake_states);

            plot::PotentialFieldPlotter potential_field_plotter(
                floor_img.rows, floor_img.cols, snapshots_dir, data["plot"]["potentialFieldSnapshots"]["name"],
                data["plot"]["potentialFieldSnapshots"]["extension"], data["plot"]["potentialFieldSnapshots"]["stride"],
                data["plot"]["potentialFieldSnapshots"]["logPlot"],
                data["plot"]["potentialFieldSnapshots"]["flipColorScheme"]);

            // If snapshots folder does not exist then create it
            if (!std::filesystem::exists(snapshots_dir)) {
                std::filesystem::create_directory(snapshots_dir);
            }

            potential_field_plotter.save(pot_field, *fake_dynamic_potential);
        }

        update_timestamp(start, finish, data["gridCompute"]["displayRuntime"]);
    } else {
        std::cout << std::endl << "Global path not found." << std::endl;
    }

    // Save figure and smoothed trajectory
    if (data["gridCompute"]["displayRuntime"]) {
        std::cout << "Saving hex plot as image..." << std::endl;
        start = std::chrono::high_resolution_clock::now();
    }

    if (data["json"]["save"]) {
        json_factory << database_dir / "database.json";
    }
    hex_figure.savefig(output_dir / data["fileNames"]["outputMapName"]);
    update_timestamp(start, finish, data["gridCompute"]["displayRuntime"]);

    return 0;
}
