#ifndef READ_GRAPH_HPP
#define READ_GRAPH_HPP

#include <vector>
#include <map>
#include <optional>

struct GraphData {
    std::map<int, std::vector<std::pair<int, int>>> graph;
    std::optional<int> optimal_value;
};

GraphData read_input();

#endif