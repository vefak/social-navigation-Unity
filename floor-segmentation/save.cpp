#include "save.h"

namespace save {

JsonFactory& JsonFactory::operator>>(grid::HexGrid& hex_grid) {
    json hex_array;

    for (grid::HexCell& hex : hex_grid.grid_map) {
        hex_array.push_back(json::object());

        hex_array.back()["row"] = hex.row;
        hex_array.back()["col"] = hex.col;
        hex_array.back()["rawIdx"] = hex.raw_idx;
        hex_array.back()["x"] = hex.x;
        hex_array.back()["y"] = hex.y;
        hex_array.back()["free"] = hex.free;

        int neighbor_list[6] = {-1, -1, -1, -1, -1, -1};

        for (size_t i = 0; i < hex.num_directions(); ++i) {
            if (hex.neighbors[i] != nullptr) {
                neighbor_list[i] = hex.neighbors[i]->raw_idx;
            }
        }

        hex_array.back()["neighbors"] = neighbor_list;
    }

    json_stream["hexGrid"] = hex_array;

    return *this;
}

JsonFactory& JsonFactory::operator>>(potential::PotentialField& pot_field) {
    for (auto& hex : json_stream["hexGrid"]) {
        geometry::Point p_hex(hex["x"], hex["y"]);
        double potential = pot_field.map_potential->get_potential(p_hex);
        hex["potential"] = potential;
    }

    return *this;
}

JsonFactory& JsonFactory::operator>>(graph_util::VoronoiMap& voronoi_map) {
    json voronoi_array;

    for (graph_util::VoronoiCell& cell : voronoi_map.cells) {
        voronoi_array.push_back(json::object());
        voronoi_array.back()["root"] = cell.root->index;  // Index of root connected component

        // Collect all member nodes and relevant information
        json nodes;

        for (size_t i = 0; i < cell.nodes.size(); ++i) {
            std::unordered_map<const char*, int> node{{"rawIdx", cell.nodes[i]->raw_idx},
                                                      {"hexDist", cell.hex_distances[i]},
                                                      {"centerDist", cell.center_distances[i]}};
            json j_node(node);
            nodes.push_back(j_node);
        }

        voronoi_array.back()["nodes"] = nodes;
    }

    json_stream["voronoiMap"] = voronoi_array;

    return *this;
}

JsonFactory& JsonFactory::operator>>(graph_util::RoomGraph& room_graph) {
    json rooms_array;

    for (graph_util::Room& room : room_graph.rooms) {
        rooms_array.push_back(json::object());
        rooms_array.back()["index"] = room.index;

        std::vector<int> cell_indices(room.cells.size());
        for (size_t i = 0; i < cell_indices.size(); ++i) {
            cell_indices[i] = room.cells[i]->raw_idx;
        }

        rooms_array.back()["cellIndices"] = cell_indices;

        json neighbors;

        for (graph_util::RoomEdge& edge : room_graph.adjacency_list[room.index]) {
            neighbors.push_back(json::object());
            neighbors.back()["index"] = edge.v->index;
            neighbors.back()["distance"] = edge.weight;
        }

        rooms_array.back()["neighbors"] = neighbors;
    }

    json_stream["roomGraph"] = rooms_array;

    return *this;
}

JsonFactory& JsonFactory::operator<<(const std::filesystem::path path) {
    std::ofstream file(path);
    file << json_stream.dump(dump_spacing);
    return *this;
}

}  // namespace save