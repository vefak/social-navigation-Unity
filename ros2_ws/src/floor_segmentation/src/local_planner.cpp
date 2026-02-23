#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "unitycustommsg/msg/point2_d.hpp"
#include "unitycustommsg/msg/point2_d_array.hpp"
#include "unitycustommsg/msg/social_metrics.hpp"
#include <opencv2/opencv.hpp>
#include <nav_msgs/msg/path.hpp>
// Include headers from the floor_segmentation libraries
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
#include "potential.h"
#include "save.h"
#include "social_force.h"
#include "util.h"

#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/follow_path.hpp"

struct Parameters
{
    std::filesystem::path input_dir, json_dir;
    int tile_size;
    double threshold, jump_penalty, x_scale, y_scale, scaling_factor;
    planning::HeuristicParams heuristic_params;
    social_force::LocalPlannerParams params;
    social_force::SocialForceParams sf_params;
    double waypoint_attraction;
};

struct LocalPlanner : public rclcpp::Node
{

    Parameters params;
    cv::Mat floor_img;
    std::unique_ptr<grid::HexGrid> hex_grid_ptr;
    grid::HexCell *hex_start, *hex_goal;
    std::unique_ptr<robot::RectangularRobot> robot_ptr;
    std::unique_ptr<graph_util::VoronoiMap> voronoi_map_ptr;
    std::vector<bool> interior_mask;
    graph_util::RoomFeaturePoints features;
    std::unique_ptr<planning::Heuristic> heuristic;
    std::unique_ptr<planning::PlannerOutput> output;

    std::unique_ptr<social_force::SocialForceParams> sf_params_ptr;
    potential::PotentialField pot_field;
    std::shared_ptr<potential::DistanceFunction> dist_fnc;
    std::shared_ptr<potential::DynamicPotential> dynamic_potential;
    std::shared_ptr<potential::DistanceFunction> obs_repulsion_fnc;

    std::shared_ptr<database::GraphDatabase> graph_db;
    transforms::CoordinateTransform pixel_to_unity, unity_to_pixel;
    std::unique_ptr<graph_util::NearestObstacleMap> nearest_obstacle_map;

    static constexpr int NumRings = 2;
    std::unique_ptr<grid::SuperHexGrid<NumRings>> super_hex_grid_ptr;
    std::unique_ptr<grid::OrientedHexGrid<NumRings>> oriented_grid_ptr;
    std::unique_ptr<planning::OrientedHeuristic<NumRings>> oriented_heuristic;

    static constexpr int NumRingsLocal = 2;
    std::unique_ptr<social_force::LocalPlanner> local_planner;
    std::unique_ptr<social_force::LocalAStarParams> lp_params;
    std::unique_ptr<social_force::OrientedLocalAStarParams<NumRingsLocal>>
        olp_params;
    std::unique_ptr<planning::Heuristic> local_heuristic;
    std::unique_ptr<planning::OrientedHeuristic<NumRingsLocal>>
        local_oriented_heuristic;
    std::unique_ptr<social_force::PathSmoother> path_smoother;
    robot::State goal_state;
    std::unique_ptr<robot::Human> human_ptr;

    // ROS
    rclcpp::Publisher<unitycustommsg::msg::Point2DArray>::SharedPtr
        global_trajectory_publisher;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr nav_path_publisher;
    rclcpp_action::Client<nav2_msgs::action::FollowPath>::SharedPtr follow_path_client_;

    LocalPlanner(const std::filesystem::path &root)
        : Node("local_planner"), unity_to_pixel(), pixel_to_unity()
    {
        // ROS
        follow_path_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(this, "/follow_path");

        // Load floor plan
        params.input_dir = root / "input";
        params.json_dir = root / "json";
        // Input configuration parsing
        std::ifstream in_stream(params.json_dir / "config.json");

        nlohmann::json data = nlohmann::json::parse(in_stream);
        // save map
        save::JsonFactory json_factory(data["json"]["spacing"]);

        // Load floor image
        floor_img =
            cv::imread(params.input_dir / data["fileNames"]["inputMapName"]);

        // Construct hex grid
        params.tile_size = data["gridCompute"]["tileSize"];
        params.threshold = data["gridCompute"]["detectionThreshold"];

        // Generate hex grid from floor map
        std::cout << "Constructing hex grid..." << std::endl;

        auto hex_grid_factory = grid::HexGridFactory();
        hex_grid_ptr = hex_grid_factory.create_hexagonal_grid_from_image(
            floor_img, params.tile_size, params.threshold);
        grid::HexGrid &hex_grid = *hex_grid_ptr;

        // Supergrid
        std::cout << "Connecting super hexgrid..." << std::endl;

        constexpr int NumRings = 2;
        super_hex_grid_ptr =
            std::make_unique<grid::SuperHexGrid<NumRings>>(hex_grid);
        auto super_hex_grid = *super_hex_grid_ptr;

        // Oriented grid (for oriented A*)
        std::cout << "Constructing oriented grid..." << std::endl;

        oriented_grid_ptr =
            std::make_unique<grid::OrientedHexGrid<NumRings>>(super_hex_grid);
        auto oriented_grid = *oriented_grid_ptr;

        if (data["json"]["save"])
        {
            json_factory >> hex_grid;
        }
        // Wall segmentation
        std::cout << "Constructing wall map..." << std::endl;

        params.jump_penalty = data["gridCompute"]["wallMap"]["jumpPenalty"];
        bool limit_wall_size = data["gridCompute"]["wallMap"]["limitWallSize"];
        int max_wall_size = data["gridCompute"]["wallMap"]["maxWallSize"];

        if (!limit_wall_size)
        {
            max_wall_size = hex_grid.size() + 1;
        }

        graph_util::ComponentMapHex wall_map =
            graph_util::WallMap(hex_grid, params.jump_penalty, max_wall_size);

        // Compute scaling factor (meters to pixels)
        double x_min = data["rescaling"]["xMin"];
        double x_max = data["rescaling"]["xMax"];
        double y_min = data["rescaling"]["yMin"];
        double y_max = data["rescaling"]["yMax"];
        double x_scale, y_scale;
        x_scale = params.x_scale = x_max - x_min;
        y_scale = params.y_scale = y_max - y_min;

        double scaling_factor_horizontal = floor_img.cols / x_scale;
        double scaling_factor_vertical = floor_img.rows / y_scale;
        params.scaling_factor =
            (scaling_factor_horizontal + scaling_factor_vertical) / 2;

        // Define point coordinate transform between pixels and unity
        pixel_to_unity = pixel_to_unity.compose(transforms::OffsetAngle(M_PI / 2));
        pixel_to_unity = pixel_to_unity.compose(transforms::FlipAngle());
        pixel_to_unity = pixel_to_unity.compose(
            transforms::PointToPixel(floor_img.rows, floor_img.cols, x_min, x_max,
                                     y_min, y_max)
                .inverse());
        pixel_to_unity =
            pixel_to_unity.compose(transforms::FlipYAxisImage(floor_img.rows));

        unity_to_pixel = pixel_to_unity.inverse();

        // Start and goal hexes
        geometry::Point start_point(data["robot"]["start"]["position"]["x"],
                                    data["robot"]["start"]["position"]["y"]);
        geometry::Point goal_point(data["pathPlanning"]["goal"]["position"]["x"],
                                   data["pathPlanning"]["goal"]["position"]["y"]);
        double theta_start = data["robot"]["start"]["theta"];
        double theta_goal = data["pathPlanning"]["goal"]["theta"];

        if (data["rescaling"]["inMeters"])
        {
            start_point = unity_to_pixel.point_transform(start_point);
            goal_point = unity_to_pixel.point_transform(goal_point);
            theta_start = unity_to_pixel.angle_transform(theta_start);
            theta_goal = unity_to_pixel.angle_transform(theta_goal);
        }

        std::vector<geometry::Circle> circle_skeleton;
        circle_skeleton.reserve(data["robot"]["circles"].size());

        for (auto &circle : data["robot"]["circles"])
        {
            circle_skeleton.push_back(
                geometry::Circle{.center = geometry::Point(circle["x"], circle["y"]),
                                 .radius = circle["radius"]});
            std::cout << "Circle " << circle_skeleton.back().center.x << " "
                      << circle_skeleton.back().radius << std::endl;
        }

        // Compute turning radius
        double steering_angle = data["robot"]["steeringAngle"];
        double wheelbase = data["robot"]["wheelbase"];
        double turning_radius = wheelbase / tan(steering_angle);

        // Load robot information
        robot_ptr = std::make_unique<robot::RectangularRobot>(
            data["robot"]["length"],      // m
            data["robot"]["width"],       // m
            turning_radius,               // m
            data["robot"]["mass"],        // kg
            data["robot"]["maxVelocity"], // m/s
            circle_skeleton               // m
        );
        robot::RectangularRobot &robot = *robot_ptr;

        human_ptr = std::make_unique<robot::Human>();
        human_ptr->collision_radius = data["human"]["collisionRadius"];     // m
        human_ptr->contraction_cutoff = data["human"]["contractionCutoff"]; //
        auto &human = *human_ptr;

        std::cout << "Robot " << robot.length << " " << robot.width << std::endl;

        if (data["rescaling"]["inMeters"])
        {
            robot.length *= params.scaling_factor;
            robot.width *= params.scaling_factor;
            robot.v_max *= params.scaling_factor;
            robot.turning_radius *= params.scaling_factor;
            human.collision_radius *= params.scaling_factor;
        }

        std::cout << "Robot " << robot.length << " " << robot.width << std::endl;

        hex_start = hex_grid.nearest_hexagon(start_point);
        hex_goal = hex_grid.nearest_hexagon(goal_point);

        // Voronoi segmentation (invoke with component_map_hex instead of wall_map
        // for "classic" Voronoi map)
        std::cout << std::endl
                  << "Computing Voronoi map..." << std::endl;

        voronoi_map_ptr =
            std::make_unique<graph_util::VoronoiMap>(hex_grid, wall_map);
        graph_util::VoronoiMap &voronoi_map = *voronoi_map_ptr;

        if (data["json"]["save"])
        {
            json_factory >> voronoi_map; // Save Voronoi map to json
        }

        // Define nearest obstacle map
        std::cout << "Constructing nearest obstacle map..." << std::endl;

        if (data["robot"]["approximateNearestObstacle"])
        {
            nearest_obstacle_map =
                std::make_unique<graph_util::ApproximateNearestObstacleMap>(
                    hex_grid, voronoi_map);
        }
        else
        {
            nearest_obstacle_map =
                std::make_unique<graph_util::ExactNearestObstacleMap>(hex_grid,
                                                                      voronoi_map);
        }

        // Obtain the rooms
        std::cout << "Room segmentation..." << std::endl;
        graph_util::RoomGraph room_segmentation(voronoi_map);
        if (data["json"]["save"])
        {
            json_factory >> room_segmentation;
            std::cout << "Saved room segmentation to JSON." << std::endl;
        }

        // Mask the hexes inside the boundary of the map using the room segmentation
        // map
        interior_mask.resize(hex_grid.size());

        for (size_t i = 0; i < interior_mask.size(); ++i)
        {
            interior_mask[i] = (room_segmentation.room_map[i] != -1);
        }

        // Find the room centers and door crossings
        features = room_segmentation.get_feature_points();
        features.doors.clear(); // Turn off door plotting

        // Define heuristic function for A* / Theta* / Two-level algorithms
        params.heuristic_params = planning::HeuristicParams{
            .inf = data["pathPlanning"]["heuristicParams"]["infinity"],
            .tile_size = static_cast<double>(params.tile_size),
            .avoid_walls = data["pathPlanning"]["heuristicParams"]["avoidWalls"],
            .valley_potential =
                data["pathPlanning"]["heuristicParams"]["useValleyPotential"],
            .potential = voronoi_map.center_distance};

        // Heuristic for oriented A*
        planning::OrientedHeuristicParams oriented_heuristic_params{
            .linear_params = params.heuristic_params,
            .angle_weight = data["pathPlanning"]["heuristicParams"]["angleWeight"]};

        if (data["pathPlanning"]["heuristicParams"]["heuristic"] == "hexagonal")
        {
            heuristic = std::make_unique<planning::HexagonalDistance>(
                params.heuristic_params);
            oriented_heuristic =
                std::make_unique<planning::OrientedHexagonalDistance<NumRings>>(
                    oriented_heuristic_params);
        }
        else if (data["pathPlanning"]["heuristicParams"]["heuristic"] ==
                 "euclidean")
        {
            heuristic = std::make_unique<planning::EuclideanDistance>(
                params.heuristic_params);
            oriented_heuristic =
                std::make_unique<planning::OrientedEuclideanDistance<NumRings>>(
                    oriented_heuristic_params);
        }
        else
        {
            heuristic =
                std::make_unique<planning::Heuristic>(params.heuristic_params);
            oriented_heuristic =
                std::make_unique<planning::OrientedHeuristic<NumRings>>(
                    oriented_heuristic_params);
        }

        // Define input for planning algorithm
        std::cout << "Path planning..." << std::endl;

        if (data["pathPlanning"]["algorithm"] == "Oriented-A-Star")
        {
            size_t start_angle_idx = oriented_grid.nearest_angle_index(theta_start);
            size_t goal_angle_idx = oriented_grid.nearest_angle_index(theta_goal);

            size_t start_idx =
                start_angle_idx * oriented_grid.stride() + hex_start->raw_idx;
            size_t goal_idx =
                goal_angle_idx * oriented_grid.stride() + hex_goal->raw_idx;

            grid::OrientedHex<NumRings> *oriented_hex_start =
                &oriented_grid.oriented_map[start_idx];
            grid::OrientedHex<NumRings> *oriented_hex_goal =
                &oriented_grid.oriented_map[goal_idx];

            // Define angle cost function
            double angle_cost_multiplier =
                data["pathPlanning"]["OrientedAStarParams"]["angleCostMultiplier"];
            double angle_sensitivity =
                data["pathPlanning"]["OrientedAStarParams"]["angleSensitivity"];
            double angle_deadzone =
                data["pathPlanning"]["OrientedAStarParams"]["angleDeadzone"];
            double turnrate_penalty =
                data["pathPlanning"]["OrientedAStarParams"]["turnratePenalty"];
            double maximum_angle = std::max(
                1e-4 + M_PI / 6, robot.max_turning_angle(params.tile_size * sqrt(3)));

            std::function<double(double)> angle_cost_function =
                planning::get_angle_cost_function(
                    data["pathPlanning"]["OrientedAStarParams"]["angleCostFunction"],
                    angle_cost_multiplier, angle_sensitivity, angle_deadzone,
                    maximum_angle, turnrate_penalty);

            planning::OrientedAStarInput<NumRings> grid_plan_input(
                oriented_hex_start, oriented_hex_goal, *oriented_heuristic,
                angle_cost_function);

            planning::OrientedAStarPlanner<NumRings> oriented_a_star_planner(
                oriented_grid, robot, *nearest_obstacle_map);

            planning::OrientedGridSearchOutput<NumRings> debug_output =
                oriented_a_star_planner.find_path(grid_plan_input);
            output = std::make_unique<planning::OrientedGridSearchOutput<NumRings>>(
                debug_output);
        }
        else
        {
            planning::GridSearchOutput debug_output;

            if (data["pathPlanning"]["algorithm"] == "A-Star")
            {
                planning::GridSearchInput grid_plan_input(hex_start, hex_goal,
                                                          *heuristic);
                planning::AStarPlanner a_star_planner(hex_grid);
                debug_output = a_star_planner.find_path(grid_plan_input);
            }
            else if (data["pathPlanning"]["algorithm"] == "Theta-Star")
            {
                planning::ThetaStarInput grid_plan_input(hex_start, hex_goal,
                                                         *heuristic);
                planning::ThetaStarPlanner theta_star_planner(hex_grid);
                debug_output = theta_star_planner.find_path(grid_plan_input);
            }

            output = std::make_unique<planning::GridSearchOutput>(debug_output);
        }

        assert(output->found_path);

        // Smooth out the path using a social force algorithm, which respects the
        // physics of motion
        std::cout << std::endl
                  << "Global path found." << std::endl;

        params.params = social_force::LocalPlannerParams{
            .max_samples = data["localPlanner"]["socialForce"]["linearParams"]
                               ["maxSamples"], //
            .lookahead =
                data["localPlanner"]["socialForce"]["linearParams"]["lookahead"], //
            .sampling_period = data["localPlanner"]["socialForce"]["linearParams"]
                                   ["samplingPeriod"], // s
            .goal_cutoff_distance = data["localPlanner"]["socialForce"]
                                        ["linearParams"]["goalCutoffDistance"], // m
            .walk_length =
                data["localPlanner"]["socialForce"]["freeWalkParams"]["walkLength"],
            .skip_steps =
                data["localPlanner"]["socialForce"]["freeWalkParams"]["skipSteps"],
            .project_starting_point =
                data["localPlanner"]["socialForce"]["freeWalkParams"]
                    ["projectStartingPoint"]};

        params.sf_params = social_force::SocialForceParams{
            .alpha =
                data["localPlanner"]["socialForce"]["rotationParams"]["alpha"], //
            .k_d = data["localPlanner"]["socialForce"]["rotationParams"]
                       ["k_d"],                                                  // kg/s
            .k_o = data["localPlanner"]["socialForce"]["rotationParams"]["k_o"], //
            .k_lambda = data["localPlanner"]["socialForce"]["rotationParams"]
                            ["k_lambda"], // 1 / (kg m)
            .influence_auxiliary_agents =
                data["localPlanner"]["socialForce"]["influenceAuxiliaryAgents"]};

        if (data["rescaling"]
                ["inMeters"])
        { // if the units are provided in meters rescale them
            // to pixels (and undo the scaling later)
            params.params.goal_cutoff_distance *= params.scaling_factor;
            params.sf_params.k_lambda /= params.scaling_factor;
        }

        // Define potential field
        std::cout << "Constructing map potential..." << std::endl;
        std::string map_potential_type =
            data["potentialField"]["mapPotential"]["type"];
        std::string driving_potential_type =
            data["potentialField"]["drivingPotential"]["type"];

        // Define repulsive distance function
        double A = data["potentialField"]["obstacleRepulsionParams"]["A"]; // N
        double B = data["potentialField"]["obstacleRepulsionParams"]["B"]; // m^2

        if (data["rescaling"]["inMeters"])
        {
            A *= params.scaling_factor;
            B *= (params.scaling_factor * params.scaling_factor);
        }

        obs_repulsion_fnc = std::make_shared<potential::Exponential>(A, B);

        if (map_potential_type == "VoronoiFieldPotential")
        {
            double alpha = data["potentialField"]["mapPotential"]
                               ["voronoiFieldPotentialParams"]["alpha"]; // m
            double dist_obs_max =
                data["potentialField"]["mapPotential"]["voronoiFieldPotentialParams"]
                    ["distObsMax"]; // m

            if (data["rescaling"]["inMeters"])
            {
                alpha *= params.scaling_factor;
                dist_obs_max *= params.scaling_factor;
            }

            //   std::cout << "Voronoi Field Potential " << dist_obs_max << std::endl;
            pot_field.map_potential =
                std::make_shared<potential::VoronoiFieldPotential>(
                    hex_grid, voronoi_map, *nearest_obstacle_map,
                    data["potentialField"]["mapPotential"]
                        ["voronoiFieldPotentialParams"]["scale"],
                    alpha, dist_obs_max);
        }
        else if (map_potential_type == "ObstacleRepulsionPotential")
        {
            pot_field.map_potential =
                std::make_shared<potential::ObstacleRepulsionPotential>(
                    hex_grid, voronoi_map, *nearest_obstacle_map, obs_repulsion_fnc);
        }
        if (data["json"]["save"])
        {
            json_factory >> pot_field;
        }

        std::cout << "Constructing driving potential..." << std::endl;

        if (data["potentialField"]["drivingPotential"]["distFunction"] == "Conic")
        {
            dist_fnc = std::make_shared<potential::Conic>();
        }
        else if (data["potentialField"]["drivingPotential"]["distFunction"] ==
                 "Quadratic")
        {
            dist_fnc = std::make_shared<potential::Quadratic>();
        }

        params.waypoint_attraction =
            data["potentialField"]["drivingPotential"]["waypointAttraction"]; // N

        if (data["rescaling"]["inMeters"])
        {
            params.waypoint_attraction *= params.scaling_factor;
        }

        if (driving_potential_type == "GoalAndPathPotential")
        {
            double valley_attraction =
                data["potentialField"]["drivingPotential"]
                    ["goalAndPathPotentialParams"]["valleyAttraction"]; // N

            if (data["rescaling"]["inMeters"])
            {
                valley_attraction *= params.scaling_factor;
            }

            pot_field.driving_potential =
                std::make_shared<potential::GoalAndPathPotential>(
                    output->path, params.waypoint_attraction, valley_attraction,
                    dist_fnc);
        }
        else if (driving_potential_type == "LookaheadPotential")
        {
            pot_field.driving_potential =
                std::make_shared<potential::LookaheadPotential>(
                    output->path, params.waypoint_attraction,
                    data["potentialField"]["drivingPotential"]
                        ["lookaheadPotentialParams"]["windowSize"],
                    data["potentialField"]["drivingPotential"]
                        ["lookaheadPotentialParams"]["lookahead"],
                    dist_fnc);
        }
        else if (driving_potential_type == "WindowPotential")
        {
            double cutoff_dist = data["potentialField"]["drivingPotential"]
                                     ["windowPotentialParams"]["cutoffDist"]; // m

            if (data["rescaling"]["inMeters"])
            {
                cutoff_dist *= params.scaling_factor;
            }

            pot_field.driving_potential =
                std::make_shared<potential::WindowPotential>(
                    output->path, params.waypoint_attraction, cutoff_dist,
                    data["potentialField"]["drivingPotential"]
                        ["windowPotentialParams"]["windowSize"],
                    dist_fnc);
        }

        if (data["potentialField"]["precomputePotential"])
        {
            pot_field.map_potential = std::make_shared<potential::GridMapPotential>(
                pot_field.map_potential, hex_grid);
            pot_field.driving_potential =
                std::make_shared<potential::GridDrivingPotential>(
                    pot_field.driving_potential, hex_grid);
        }

        goal_state = robot::State{.position = {hex_goal->x, hex_goal->y},
                                  .velocity = {0, 0},
                                  .orientation = theta_goal,
                                  .angular_velocity = 0};

        // Spawn neo4j
        graph_db = std::make_shared<database::Neo4j>(root, unity_to_pixel);

        if (data["potentialField"]["dynamicPotential"]["type"] ==
            "RipplePotential")
        {
            dynamic_potential =
                std::make_shared<potential::RipplePotential>(obs_repulsion_fnc);
        }
        else
        {
            double cutoff = data["potentialField"]["dynamicPotential"]["cutoff"];
            dynamic_potential = std::make_shared<potential::EggshellPotential>(
                obs_repulsion_fnc, cutoff);
        }

        // Define global planner input

        if (data["localPlanner"]["planner"] == "SFM")
        {
            local_planner = std::make_unique<social_force::SocialForceLocalPlanner>(
                *graph_db, goal_state, *output, params.params, hex_grid, pot_field,
                dynamic_potential, robot, params.sf_params);
        }
        else
        {
            assert(data["localPlanner"]["planner"] == "A-Star" ||
                   data["localPlanner"]["planner"] == "Oriented-A-Star");

            bool lightweight = data["localPlanner"]["AStarParams"]["lightweight"];
            double downscale_field =
                data["localPlanner"]["AStarParams"]["downscaleField"];

            int half_height = data["localPlanner"]["gridParams"]["halfHeight"];
            int half_width = data["localPlanner"]["gridParams"]["halfWidth"];
            double downscale_factor =
                data["localPlanner"]["gridParams"]["gridDownscaleFactor"];
            double center_slide = data["localPlanner"]["gridParams"]["centerSlide"];

            int lookahead = data["localPlanner"]["AStarParams"]["lookahead"];
            bool fixed_walk_length =
                data["localPlanner"]["AStarParams"]["fixedWalkLength"];
            bool rim_termination =
                data["localPlanner"]["AStarParams"]["rimTermination"];

            double stoppage_distance =
                data["localPlanner"]["AStarParams"]["stoppageDistance"];
            if (data["rescaling"]["inMeters"])
            {
                stoppage_distance *= params.scaling_factor; // m
                std::cout << "Stoppage distance: " << stoppage_distance << std::endl;
            }

            double grid_quanta;

            if (data["localPlanner"]["gridParams"]["adaptive"])
            {
                double v_cruise = data["robot"]["cruiseVelocity"];

                if (data["rescaling"]["inMeters"])
                {
                    v_cruise *= params.scaling_factor; // m/s
                }

                double T_sample = data["localPlanner"]["socialForce"]["linearParams"]
                                      ["samplingPeriod"];

                grid_quanta = v_cruise * T_sample;
            }
            else
            {
                grid_quanta = params.tile_size * downscale_factor;
            }

            // Define local heuristic
            planning::HeuristicParams local_heuristic_params{
                .inf = data["localPlanner"]["heuristicParams"]["infinity"],
                .tile_size = grid_quanta,
                .avoid_walls = data["localPlanner"]["heuristicParams"]["avoidWalls"],
                .valley_potential =
                    false, // There is no valley potential for the local grid
                .potential = {}};

            planning::OrientedHeuristicParams local_oriented_heuristic_params{
                .linear_params = local_heuristic_params,
                .angle_weight =
                    data["localPlanner"]["heuristicParams"]["angleWeight"]};

            if (data["localPlanner"]["heuristicParams"]["heuristic"] == "hexagonal")
            {
                local_heuristic = std::make_unique<planning::HexagonalDistance>(
                    local_heuristic_params);
                local_oriented_heuristic = std::make_unique<
                    planning::OrientedHexagonalDistance<NumRingsLocal>>(
                    local_oriented_heuristic_params);
            }
            else if (data["localPlanner"]["heuristicParams"]["heuristic"] ==
                     "euclidean")
            {
                local_heuristic = std::make_unique<planning::EuclideanDistance>(
                    local_heuristic_params);
                local_oriented_heuristic = std::make_unique<
                    planning::OrientedEuclideanDistance<NumRingsLocal>>(
                    local_oriented_heuristic_params);
            }
            else
            {
                local_heuristic =
                    std::make_unique<planning::Heuristic>(local_heuristic_params);
                local_oriented_heuristic =
                    std::make_unique<planning::OrientedHeuristic<NumRingsLocal>>(
                        local_oriented_heuristic_params);
            }

            // Path smoothing
            if (data["localPlanner"]["AStarParams"]["pathSmoothing"]["algorithm"] ==
                "None")
            {
                path_smoother = std::make_unique<social_force::IdentitySmoother>();
            }
            else
            {
                assert(
                    data["localPlanner"]["AStarParams"]["pathSmoothing"]["algorithm"] ==
                    "Gaussian");
                int half_window =
                    data["localPlanner"]["AStarParams"]["pathSmoothing"]["halfWindow"];
                double sigma =
                    data["localPlanner"]["AStarParams"]["pathSmoothing"]["sigma"];

                path_smoother = std::make_unique<social_force::GaussianSmoother>(
                    half_window, sigma);
            }

            lp_params = std::make_unique<social_force::LocalAStarParams>(
                lightweight, fixed_walk_length, rim_termination, half_height,
                half_width, center_slide, lookahead, grid_quanta, downscale_field,
                stoppage_distance, *local_heuristic, *path_smoother,
                *nearest_obstacle_map);

            double angle_cost_multiplier =
                data["localPlanner"]["OrientedAStarParams"]["angleCostMultiplier"];
            double angle_sensitivity =
                data["localPlanner"]["OrientedAStarParams"]["angleSensitivity"];
            double angle_deadzone =
                data["localPlanner"]["OrientedAStarParams"]["angleDeadzone"];
            double turnrate_penalty =
                data["pathPlanning"]["OrientedAStarParams"]["turnratePenalty"];
            double maximum_angle = std::max(
                1e-4 + M_PI / 6, robot.max_turning_angle(params.tile_size * sqrt(3)));

            std::function<double(double)> angle_cost_function =
                planning::get_angle_cost_function(
                    data["pathPlanning"]["OrientedAStarParams"]["angleCostFunction"],
                    angle_cost_multiplier, angle_sensitivity, angle_deadzone,
                    maximum_angle, turnrate_penalty);

            olp_params = std::make_unique<
                social_force::OrientedLocalAStarParams<NumRingsLocal>>(
                *local_oriented_heuristic, angle_cost_function);

            if (data["localPlanner"]["planner"] == "A-Star")
            {
                local_planner = std::make_unique<social_force::LocalAStar>(
                    *graph_db, goal_state, *output, params.params, hex_grid, pot_field,
                    dynamic_potential, robot, human, params.sf_params, *lp_params);
            }
            else
            {
                local_planner =
                    std::make_unique<social_force::LocalOrientedAStar<NumRingsLocal>>(
                        *graph_db, goal_state, *output, params.params, hex_grid,
                        pot_field, dynamic_potential, robot, human, params.sf_params,
                        *lp_params, *olp_params);
            }
        }

        std::cout << "Constructor done" << std::endl;
        const std::filesystem::path database_dir = data["json"]["databasePath"];
        if (data["json"]["save"])
        {
            std::filesystem::path save_path = database_dir / "database.json";
            json_factory << save_path;
            std::cout << "Saved database.json to: " << std::filesystem::absolute(save_path) << std::endl;
        }
        // ROS Publisher
        std::cout << "Created Publisher" << std::endl;
        rclcpp::QoS qos_profile =
            rclcpp::QoS(10)
                .transient_local(); // Keep last message for late subscribers
        global_trajectory_publisher =
            this->create_publisher<unitycustommsg::msg::Point2DArray>(
                "/global_trajectory", qos_profile);
        nav_path_publisher = this->create_publisher<nav_msgs::msg::Path>("/plan", qos_profile);

        publishGlobalTrajectory();
        sendPathToController(createNavPath());
    }

    nav_msgs::msg::Path createNavPath()
    {
        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = this->now();

        for (const auto &point : output->path)
        {
            robot::State state;
            state.position = {point.x, point.y};
            state = pixel_to_unity(state);

            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = state.position.x + 36;
            pose.pose.position.y = state.position.y + 13.5;
            pose.pose.position.z = 0.0;

            pose.pose.orientation.w = 1.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;

            path_msg.poses.push_back(pose);
        }
        return path_msg;
    }

    void publishGlobalTrajectory()
    {
        auto global_message = unitycustommsg::msg::Point2DArray();

        nav_msgs::msg::Path path_msg;
        path_msg.header.frame_id = "map"; // Use "map" or "odom" depending on your TF
        path_msg.header.stamp = this->now();

        for (const auto &point : output->path)
        {
            robot::State state;
            state.position = {point.x, point.y};
            state = pixel_to_unity(state);
            unitycustommsg::msg::Point2D global_msg;
            global_msg.x = state.position.x;
            global_msg.y = state.position.y;
            global_message.points.push_back(global_msg);
        }
        for (const auto &point : output->path)
        {
            robot::State state;
            state.position = {point.x, point.y};
            state = pixel_to_unity(state); // Convert to Unity coordinates

            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = state.position.x + 36;
            pose.pose.position.y = state.position.y + 13.5;
            pose.pose.position.z = 0.0;

            pose.pose.orientation.w = 1.0; // Default to no rotation
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;

            path_msg.poses.push_back(pose);
        }

        global_trajectory_publisher->publish(global_message);
        nav_path_publisher->publish(path_msg);
    }

    void sendPathToController(const nav_msgs::msg::Path &path)
    {
        using FollowPath = nav2_msgs::action::FollowPath;
        auto goal_msg = FollowPath::Goal();

        goal_msg.path = path;
        goal_msg.controller_id = "FollowPath";     // Replace if using a different controller plugin
        goal_msg.goal_checker_id = "goal_checker"; // Replace if needed

        if (!follow_path_client_->wait_for_action_server(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(this->get_logger(), "FollowPath action server not available.");
            return;
        }

        auto send_goal_options = rclcpp_action::Client<FollowPath>::SendGoalOptions();

        send_goal_options.result_callback = [](const rclcpp_action::ClientGoalHandle<FollowPath>::WrappedResult &result)
        {
            switch (result.code)
            {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(rclcpp::get_logger("FollowPath"), "FollowPath succeeded.");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(rclcpp::get_logger("FollowPath"), "FollowPath aborted.");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(rclcpp::get_logger("FollowPath"), "FollowPath canceled.");
                break;
            default:
                RCLCPP_WARN(rclcpp::get_logger("FollowPath"), "Unknown result code.");
                break;
            }
        };

        follow_path_client_->async_send_goal(goal_msg, send_goal_options);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Create node
    std::filesystem::path root_path("../floor-segmentation");
    std::cout << std::filesystem::current_path() << std::endl;
    auto node = std::make_shared<LocalPlanner>(root_path);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
