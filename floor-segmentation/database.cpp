#include "database.h"

namespace database {

std::vector<double> Neo4j::cast_point(py::object list) {
    std::vector<double> vec(3);  // List size is at most 3 (3D point)
    size_t count = 0;

    for (auto& item : list) {
        vec[count++] = item.cast<double>();
    }

    return vec;
}

robot::State Neo4j::state_from_py(auto robot_state_py) {
    robot::State robot_state;

    auto position_list = cast_point(robot_state_py.attr("position"));
    robot_state.position = {position_list[0], position_list[2]};

    auto rotation_list = cast_point(robot_state_py.attr("rotation"));
    robot_state.orientation = rotation_list[1];

    auto velocity_list = cast_point(robot_state_py.attr("linear_velocity"));
    robot_state.velocity = {velocity_list[0], velocity_list[2]};

    auto angular_velocity_list = cast_point(robot_state_py.attr("angular_velocity"));
    robot_state.angular_velocity = angular_velocity_list[1];

    // Transform the state directly
    return transform(robot_state);
}

void Neo4j::demo_setup() { db_interface.attr("demo_setup")(); }

std::vector<robot::State> Neo4j::get_dynamic_obstacles(geometry::Point p) {
    p = transform.inverse().point_transform(p);
    py::object dynamic_obstacles_py = db_interface.attr("get_dynamic_obstacles")(p.x, p.y);
    std::vector<robot::State> dynamic_obstacles;

    for (auto& obstacle_py : dynamic_obstacles_py) {
        dynamic_obstacles.push_back(state_from_py(obstacle_py));
    }

    return dynamic_obstacles;
}

robot::State Neo4j::get_robot_state() {
    py::object robot_state_py = db_interface.attr("get_robot_state")();
    return state_from_py(robot_state_py);
}

geometry::Point Neo4j::get_robot_position() {
    py::object robot_position_py = db_interface.attr("get_robot_position")();
    auto position_list = cast_point(robot_position_py);
    geometry::Point position{position_list[0], position_list[1]};  // Python function already returns (x, y)
    return transform.point_transform(position);
}

double Neo4j::get_robot_orientation() {
    py::object robot_rotation_py = db_interface.attr("get_robot_orientation")();
    double orientation = robot_rotation_py.cast<double>();
    return transform.angle_transform(orientation);
}

geometry::Point Neo4j::get_robot_velocity() {
    py::object robot_velocity_py = db_interface.attr("get_robot_velocity")();
    auto velocity_list = cast_point(robot_velocity_py);
    geometry::Point velocity{velocity_list[0], velocity_list[1]};  // Python function already returns (v_x, v_y)
    return transform.velocity_transform(velocity);
}

double Neo4j::get_robot_angular_velocity() {
    py::object robot_angular_velocity_py = db_interface.attr("get_robot_angular_velocity")();
    double angular_velocity = robot_angular_velocity_py.cast<double>();
    return transform.angular_velocity_transform(angular_velocity);
}

void Neo4j::update_robot_position(geometry::Point p) {
    p = transform.inverse().point_transform(p);
    db_interface.attr("update_robot_position")(p.x, p.y);
}

void Neo4j::update_robot_orientation(double theta) {
    theta = transform.inverse().angle_transform(theta);
    db_interface.attr("update_robot_orientation")(theta);
}

void Neo4j::update_robot_velocity(geometry::Point v) {
    v = transform.inverse().velocity_transform(v);
    db_interface.attr("update_robot_velocity")(v.x, v.y);
}

void Neo4j::update_robot_angular_velocity(double omega) {
    omega = transform.inverse().angular_velocity_transform(omega);
    db_interface.attr("update_robot_angular_velocity")(omega);
}

std::vector<grid::HexCell*> DenseDatabase::get_free_neighbors(grid::HexCell* hex) {
    std::vector<grid::HexCell*> neighbors;
    neighbors.reserve(6);

    for (grid::HexCell* neighbor : hex->neighbors) {
        if (neighbor != nullptr && neighbor->free) neighbors.emplace_back(neighbor);
    }

    return neighbors;
}

std::vector<graph_util::RoomEdge> DenseDatabase::outgoing_edges(graph_util::Room* room) {
    return room_graph.adjacency_list[room->index];
}

graph_util::Room* DenseDatabase::containing_room(grid::HexCell* hex) {
    int room_index = room_graph.room_map[hex->raw_idx];
    return &room_graph.rooms[room_index];
}

}  // namespace database