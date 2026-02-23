
#include <chrono>

#include "database.h"
#include "nlohmann/json.hpp"
#include "robot.h"
#include "transforms.h"

void print_time(std::chrono::time_point<std::chrono::high_resolution_clock> start,
                std::chrono::time_point<std::chrono::high_resolution_clock> end) {
    std::chrono::duration<float> elapsed = end - start;
    std::cout << "Done. Elapsed " << elapsed.count() << " [s]." << std::endl << std::endl;
}

int main() {
    const std::filesystem::path root = "..";
    const std::filesystem::path input_dir = root / "input";
    const std::filesystem::path json_dir = root / "json";

    // Input configuration parsing
    std::ifstream in_stream(json_dir / "config.json");
    nlohmann::json data = nlohmann::json::parse(in_stream);

    // Load rescaling information
    cv::Mat floor_img = cv::imread(input_dir / data["fileNames"]["inputMapName"]);

    // Unity angle
    auto pixel_to_unity_angle = transforms::OffsetAngle(M_PI / 2);

    // Initialize interface with Neo4j through Python
    std::cout << "Initializing interface..." << std::endl;
    std::chrono::time_point start = std::chrono::high_resolution_clock::now();

    transforms::CoordinateTransform transform;  // Identity
    // transform = transform.compose(transforms::FlipAngle());
    // transform = transform.compose(transforms::OffsetAngle(M_PI / 2));
    // transform = transform.compose(transforms::FlipYAxisImage(floor_img.rows));
    // transform = transform.compose(transforms::PointToPixel(floor_img.rows, floor_img.cols, data["rescaling"]["xMin"],
    //                                                        data["rescaling"]["xMax"], data["rescaling"]["yMin"],
    //                                                        data["rescaling"]["yMax"]));

    database::Neo4j db(root, transform);

    print_time(start, std::chrono::high_resolution_clock::now());

    // Set-up a basic example
    std::cout << "Example setup..." << std::endl;
    start = std::chrono::high_resolution_clock::now();
    db.demo_setup();
    print_time(start, std::chrono::high_resolution_clock::now());

    // Check that localization queries work
    std::cout << "Localization queries..." << std::endl;
    start = std::chrono::high_resolution_clock::now();

    robot::State robot_state = db.get_robot_state();
    std::cout << "position: " << db.get_robot_position() << std::endl;
    std::cout << "orientation: " << db.get_robot_orientation() << std::endl;
    std::cout << "linear velocity: " << db.get_robot_velocity() << std::endl;
    std::cout << "angular velocity: " << db.get_robot_angular_velocity() << std::endl;

    print_time(start, std::chrono::high_resolution_clock::now());

    // Check that nearby obstacle query works
    std::cout << "Dynamic obstacles query..." << std::endl;
    start = std::chrono::high_resolution_clock::now();
    std::vector<robot::State> dynamic_obstacles = db.get_dynamic_obstacles(db.get_robot_position());
    std::cout << "count: " << dynamic_obstacles.size() << std::endl;
    print_time(start, std::chrono::high_resolution_clock::now());

    // Check that update queries work
    std::cout << "Update queries..." << std::endl;
    start = std::chrono::high_resolution_clock::now();

    db.update_robot_position({-1, 1});
    db.update_robot_orientation(M_PI / 2);
    db.update_robot_velocity({0, 1});
    db.update_robot_angular_velocity(1);

    robot::State new_robot_state = db.get_robot_state();
    std::cout << "position: " << db.get_robot_position() << std::endl;
    std::cout << "orientation: " << db.get_robot_orientation() << std::endl;
    std::cout << "linear velocity: " << db.get_robot_velocity() << std::endl;
    std::cout << "angular velocity: " << db.get_robot_angular_velocity() << std::endl;

    print_time(start, std::chrono::high_resolution_clock::now());

    // Check again nearby obstacles
    std::cout << "Dynamic query for updated robot position..." << std::endl;
    start = std::chrono::high_resolution_clock::now();
    std::vector<robot::State> new_dynamic_obstacles = db.get_dynamic_obstacles(db.get_robot_position());
    std::cout << "count: " << new_dynamic_obstacles.size() << std::endl;
    print_time(start, std::chrono::high_resolution_clock::now());

    return 0;
}