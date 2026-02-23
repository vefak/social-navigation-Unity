#pragma once

#include "graph_util.h"
#include "grid.h"
#include "nlohmann/json.hpp"
#include "potential.h"
#include "social_force.h"

using namespace nlohmann;

namespace save {
struct JsonFactory {
    int dump_spacing;
    json json_stream;

    JsonFactory& operator>>(grid::HexGrid& hex_grid);

    JsonFactory& operator>>(graph_util::VoronoiMap& voronoi_map);

    JsonFactory& operator>>(graph_util::RoomGraph& room_graph);

    JsonFactory& operator>>(potential::PotentialField& pot_field);

    JsonFactory& operator<<(const std::filesystem::path path);

    void clear() { json_stream.clear(); }

    JsonFactory() { dump_spacing = -1; }

    JsonFactory(int dump_spacing) : dump_spacing(dump_spacing) {}
};
}  // namespace save