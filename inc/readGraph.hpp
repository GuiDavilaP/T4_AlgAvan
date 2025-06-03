#ifndef READ_GRAPH_HPP
#define READ_GRAPH_HPP

#include <string>
#include <vector>
#include <map>
#include <optional>

struct GraphData {
    std::map<std::string, std::vector<std::pair<std::string, int>>> graph;
    std::optional<int> optimal_value;
};

GraphData read_input();

#endif