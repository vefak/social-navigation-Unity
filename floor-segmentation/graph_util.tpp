#pragma once

namespace graph_util {

template <int N>
void ComponentMap<N>::construct_components_from_map(Grid<N>& floor_grid, int component_count) {
    components.clear();  // For use in wall segmentation (derived class)
    components.resize(component_count);

    for (int i = 0; i < component_count; ++i) components[i].index = i;

    for (Cell<N>& node : floor_grid.grid_map) {
        if (!node.free) {
            int comp_idx = component_map[node.raw_idx];
            components[comp_idx].nodes.emplace_back(&node);
        }
    }
}

template <int N>
ComponentMap<N>::ComponentMap(Grid<N>& floor_grid) {
    // Initially all nodes are unvisited (being part of component zero)
    component_map.resize(floor_grid.size(), -1);
    int component_count = 0;

    for (Cell<N>& node : floor_grid.grid_map) {
        if (component_map[node.raw_idx] == -1 && !node.free) {
            bfs_util(&node, component_map, component_count);
            ++component_count;
        }
    }

    construct_components_from_map(floor_grid, component_count);
}

template <int N>
void ComponentMap<N>::bfs_util(Cell<N>* node, std::vector<int>& component_map, int comp_idx) {
    std::queue<Cell<N>*> node_queue;
    node_queue.push(node);
    component_map[node->raw_idx] = comp_idx;

    while (!node_queue.empty()) {
        Cell<N>* head = node_queue.front();
        node_queue.pop();

        for (const auto& neighbor : head->neighbors) {
            if (neighbor != nullptr && !neighbor->free && component_map[neighbor->raw_idx] == -1) {
                component_map[neighbor->raw_idx] = comp_idx;
                node_queue.push(neighbor);
            }
        }
    }
}

}  // namespace graph_util
