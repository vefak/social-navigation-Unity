#pragma once

#include <pybind11/embed.h>

#include <filesystem>
#include <fstream>
#include <vector>

#include "geometry.h"
#include "graph_util.h"
#include "grid.h"
#include "robot.h"
#include "transforms.h"

namespace py = pybind11;

namespace database {

class GraphDatabase {
   public:
    virtual std::vector<robot::State> get_dynamic_obstacles(geometry::Point p) = 0;

    virtual robot::State get_robot_state() = 0;

    virtual geometry::Point get_robot_position() = 0;

    virtual double get_robot_orientation() = 0;

    virtual geometry::Point get_robot_velocity() = 0;

    virtual double get_robot_angular_velocity() = 0;

    virtual void update_robot_position(geometry::Point p) = 0;

    virtual void update_robot_orientation(double theta) = 0;

    virtual void update_robot_velocity(geometry::Point v) = 0;

    virtual void update_robot_angular_velocity(double omega) = 0;
};

class Neo4j : public GraphDatabase {
   private:
    py::scoped_interpreter guard;  // Initializes the interpreter and keeps it alive until it is destroyed
    py::module_ db_module;
    py::object db_class, db_interface;
    transforms::CoordinateTransform transform;

    std::vector<double> cast_point(py::object list);

    robot::State state_from_py(auto robot_state_py);

   public:
    void demo_setup();

    std::vector<robot::State> get_dynamic_obstacles(geometry::Point p) override;

    robot::State get_robot_state() override;

    geometry::Point get_robot_position() override;

    double get_robot_orientation() override;

    geometry::Point get_robot_velocity() override;

    double get_robot_angular_velocity() override;

    // Warning: These updates shouldn't be used in our Neo4j communication (state is bound via Unity interface)
    void update_robot_position(geometry::Point p) override;

    void update_robot_orientation(double theta) override;

    void update_robot_velocity(geometry::Point v) override;

    void update_robot_angular_velocity(double omega) override;

    // Instantiate python interpreter and keep it alive while this interface object is alive
    Neo4j(std::filesystem::path root, const transforms::CoordinateTransform& transform)
        : guard(), transform(transform) {
        std::filesystem::path database_script_dir = root / "python";

        // Put subdir on the path
        py::module_ sys = py::module_::import("sys");
        sys.attr("path").attr("append")(database_script_dir.string());

        // Initialize database interface object in python
        db_module = py::module_::import("database");
        db_class = db_module.attr("Neo4jInterface");
        db_interface = db_class(root.string());
    }
};

class MockDatabase : public GraphDatabase {
   protected:
    robot::State robot_state;

   public:
    std::vector<robot::State> get_dynamic_obstacles(geometry::Point p) override { return std::vector<robot::State>(); }

    void update_robot_position(geometry::Point p) override { robot_state.position = p; }

    void update_robot_orientation(double theta) override { robot_state.orientation = theta; }

    void update_robot_velocity(geometry::Point v) override { robot_state.velocity = v; }

    void update_robot_angular_velocity(double omega) override { robot_state.angular_velocity = omega; }

    robot::State get_robot_state() override { return robot_state; }

    geometry::Point get_robot_position() override { return robot_state.position; };

    double get_robot_orientation() override { return robot_state.orientation; }

    geometry::Point get_robot_velocity() override { return robot_state.velocity; };

    double get_robot_angular_velocity() override { return robot_state.angular_velocity; }

    MockDatabase(geometry::Point robot_position) {
        robot_state.position = robot_position;
        robot_state.velocity = {0, 0};
        robot_state.angular_velocity = 0;
    }

    MockDatabase() {}
};

class DenseDatabase : public MockDatabase {
    grid::HexGrid& hex_grid;
    graph_util::RoomGraph& room_graph;

   public:
    std::vector<grid::HexCell*> get_free_neighbors(grid::HexCell* hex);

    std::vector<graph_util::RoomEdge> outgoing_edges(graph_util::Room* room);

    graph_util::Room* containing_room(grid::HexCell* hex);

    DenseDatabase(grid::HexGrid& hex_grid, graph_util::RoomGraph& room_graph)
        : hex_grid(hex_grid), room_graph(room_graph) {}
};

}  // namespace database