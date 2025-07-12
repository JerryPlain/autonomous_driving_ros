#pragma once
#include <vector>
#include <utility>
#include <queue>
#include <unordered_map>

struct Node {
    int x, y;
    double cost, priority;

    bool operator>(const Node& other) const {
        return priority > other.priority;
    }
};

class AStarPlanner {
public:
    AStarPlanner(const std::vector<std::vector<int>>& grid);
    std::vector<std::pair<int, int>> plan(int start_x, int start_y, int goal_x, int goal_y);

private:
    std::vector<std::vector<int>> grid_;
    int width_, height_;
    std::vector<std::pair<int, int>> getNeighbors(int x, int y);
    double heuristic(int x1, int y1, int x2, int y2);
};

