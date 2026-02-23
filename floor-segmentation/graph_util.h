#pragma once
#include <unordered_set>
#include <vector>

#include "grid.h"

namespace graph_util {

using namespace grid;

// Connected components

template <int N>
struct ConnectedComponent {
    std::vector<Cell<N>*> nodes;
    int index;
};

template <int N>
class ComponentMap {
    void bfs_util(Cell<N>* node, std::vector<int>& component_map, int comp_idx);

   protected:
    // Construct list of components given component map
    void construct_components_from_map(Grid<N>& floor_grid, int component_count);

   public:
    std::vector<ConnectedComponent<N>> components;
    std::vector<int> component_map;

    ComponentMap() = default;

    ComponentMap(Grid<N>& floor_grid);

    size_t num_components() const { return components.size(); }
};

typedef ConnectedComponent<4> ConnectedComponentSquare;
typedef ComponentMap<4> ComponentMapSquare;
typedef ConnectedComponent<6> ConnectedComponentHex;
typedef ComponentMap<6> ComponentMapHex;

struct HexEdge {
    HexCell* hex;
    int side;

    bool operator!=(HexEdge edge) { return (hex != edge.hex || side != edge.side); }
};

class LineFitCumulativeTables {
    std::array<std::vector<double>, 2> xx, xy, yy, x, y;

    // Round start/endpoint to nearest integer with appropriate parity
    std::pair<int, int> bitwise_indices(int start, int endpoint, int parity) const;

    double query_util(const std::array<std::vector<double>, 2>& vec, int start, int endpoint, int parity) const;

   public:
    LineFitCumulativeTables(const std::vector<HexCell*>& shell);

    double xx_query(int start, int endpoint, int parity) const { return query_util(xx, start, endpoint, parity); }

    double xy_query(int start, int endpoint, int parity) const { return query_util(xy, start, endpoint, parity); }

    double x_query(int start, int endpoint, int parity) const { return query_util(x, start, endpoint, parity); }

    double y_query(int start, int endpoint, int parity) const { return query_util(y, start, endpoint, parity); }

    double yy_query(int start, int endpoint, int parity) const { return query_util(yy, start, endpoint, parity); }

    int range_length(int start, int endpoint, int parity) const {
        std::pair<int, int> range = bitwise_indices(start, endpoint, parity);
        return range.second - range.first + 1;
    }
};

class WallMap : public ComponentMapHex {
    HexEdge find_starting_edge(ConnectedComponentHex& component, std::set<HexEdge>& explored);

    // Returns outer shell of connected component
    std::vector<HexCell*> outer_shell(ConnectedComponentHex& component);

    // Least-squares error for fitting nodes [start, finish] with a line
    double line_fit_error_bitwise(const LineFitCumulativeTables& ct, int start, int finish, int parity);
    double line_fit_error(const LineFitCumulativeTables& ct, int start, int finish);

    // Segment the outer shell into walls
    void wall_segmentation(HexGrid& hex_grid, double jump_penalty, int max_wall_size, bool should_merge_walls);

    // Merge walls that have significant overlap
    void merge_walls(std::vector<HexCell*>& shell, int baseline_component_index, double threshold_ratio = 0.75);

   public:
    WallMap(HexGrid& hex_grid, double jump_penalty, int max_wall_size, bool should_merge_walls = true)
        : ComponentMapHex(hex_grid) {
        wall_segmentation(hex_grid, jump_penalty, max_wall_size, should_merge_walls);
    }
};

/*
    Union find (disjoint set) data structure
*/
struct UnionFind {
    std::vector<HexCell*> root;
    std::vector<int> size;

    UnionFind(int num_nodes);

    void make_set(HexCell* cell);

    HexCell* find(HexCell* cell);

    void set_union(HexCell* u, HexCell* v);
};

/* Voronoi map */

struct VoronoiCell {
    std::vector<HexCell*> nodes;
    std::vector<int> hex_distances, center_distances;
    ConnectedComponentHex* root;
};

struct VoronoiMap {
    HexGrid& hex_grid;
    ComponentMapHex& component_map;
    std::vector<VoronoiCell> cells;
    std::vector<ConnectedComponentHex*> root;
    std::vector<int> hex_distance, center_distance;

    void find_regions(HexGrid& hex_grid, ComponentMapHex& component_map, std::vector<int>& component_size);

    void compute_center_distance(HexGrid& hex_grid);

    void init(HexGrid& hex_grid, ComponentMapHex& component_map);

    VoronoiMap(HexGrid& hex_grid, ComponentMapHex& component_map) : hex_grid(hex_grid), component_map(component_map) {
        init(hex_grid, component_map);
    }
};

geometry::Point nearest_obstacle_point(geometry::Point p, HexGrid& hex_grid, const VoronoiMap& voronoi_map,
                                       const double err_tol = 1e-4);

geometry::Point nearest_voronoi_skeleton_point(geometry::Point p, HexGrid& hex_grid, const VoronoiMap& voronoi_map);

/* Room segmentation */

struct Room {
    std::vector<HexCell*> cells;
    int index;
};

struct RoomFeaturePoints {
    std::vector<HexCell*> room_centers, doors;
};

struct RoomEdge {
    Room *u, *v;
    int weight;
};

// Segmentation of floor plan into rooms
struct RoomSegmentation {
    std::vector<Room> rooms;
    std::vector<int> room_map;
    VoronoiMap& voronoi_map;
    RoomFeaturePoints features;

    // Find all feature poitns given hex distance map
    const RoomFeaturePoints& get_feature_points() { return features; }
    void find_room_centers();
    void find_doors();
    void reorder_features();  // Reorder features so that rooms[i] corresponds to features.room_centers[i]

    // BFS starting from hex keeping the distance constant
    void equidistant_bfs_util(HexCell* start, std::vector<ConnectedComponentHex>& distance_components,
                              std::vector<int>& distance_component_map);

    void iterative_deepening_bfs_util(int threshold);

    // Eliminate redundant minima (only keep median)
    void prune_features(std::vector<HexCell*>& distance_optima);

    void construct_rooms();

    RoomSegmentation(VoronoiMap& voronoi_map) : voronoi_map(voronoi_map) { construct_rooms(); }
};

// BUild graph over room segmentation
struct RoomGraph : public RoomSegmentation {
    std::vector<std::vector<RoomEdge>> adjacency_list;

    int bfs_distance(HexCell* start, HexCell* goal);

    void generate_adjacency_list();

    RoomGraph(VoronoiMap& voronoi_map) : RoomSegmentation(voronoi_map) { generate_adjacency_list(); }
};

struct NearestObstacleMap {
    grid::HexGrid& hex_grid;
    VoronoiMap& voronoi_map;

    virtual geometry::Point find_nearest_obstacle(geometry::Point p) = 0;

    NearestObstacleMap(grid::HexGrid& hex_grid, VoronoiMap& voronoi_map)
        : hex_grid(hex_grid), voronoi_map(voronoi_map) {}
};

// Precompute nearest obstacle point for each hex
struct ExactNearestObstacleMap : public NearestObstacleMap {
    geometry::Point find_nearest_obstacle(geometry::Point p) override;

    using NearestObstacleMap::NearestObstacleMap;
};

struct ApproximateNearestObstacleMap : public NearestObstacleMap {
    std::vector<geometry::Point> nearest_obstacle;

    geometry::Point find_nearest_obstacle(geometry::Point p) override;

    ApproximateNearestObstacleMap(grid::HexGrid& hex_grid, VoronoiMap& voronoi_map);
};

}  // namespace graph_util

#include "graph_util.tpp"
