#include "a_star.h"
#include <cmath>
#include <algorithm>

AStarPlanner::AStarPlanner(const std::vector<std::vector<int>>& grid) : grid_(grid) {
    height_ = grid.size();
    width_ = grid[0].size();
}

std::vector<std::pair<int, int>> AStarPlanner::getNeighbors(int x, int y) {
    std::vector<std::pair<int, int>> neighbors;
    const int dx[4] = {1, -1, 0, 0};
    const int dy[4] = {0, 0, 1, -1};
    for (int i = 0; i < 4; ++i) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_ && grid_[ny][nx] == 0) {
            neighbors.emplace_back(nx, ny);
        }
    }
    return neighbors;
}

double AStarPlanner::heuristic(int x1, int y1, int x2, int y2) {
    return std::hypot(x1 - x2, y1 - y2);  // 欧几里得距离
}

std::vector<std::pair<int, int>> AStarPlanner::plan(int start_x, int start_y, int goal_x, int goal_y) {
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;
    std::unordered_map<int, std::pair<int, int>> came_from;
    std::unordered_map<int, double> cost_so_far;

    auto toKey = [this](int x, int y) { return y * width_ + x; };

    open.push({start_x, start_y, 0.0, heuristic(start_x, start_y, goal_x, goal_y)});
    came_from[toKey(start_x, start_y)] = {-1, -1};
    cost_so_far[toKey(start_x, start_y)] = 0.0;

    while (!open.empty()) {
        Node current = open.top();
        open.pop();

        if (current.x == goal_x && current.y == goal_y)
            break;

        for (const auto& [nx, ny] : getNeighbors(current.x, current.y)) {
            double new_cost = cost_so_far[toKey(current.x, current.y)] + 1.0;
            if (!cost_so_far.count(toKey(nx, ny)) || new_cost < cost_so_far[toKey(nx, ny)]) {
                cost_so_far[toKey(nx, ny)] = new_cost;
                double priority = new_cost + heuristic(nx, ny, goal_x, goal_y);
                open.push({nx, ny, new_cost, priority});
                came_from[toKey(nx, ny)] = {current.x, current.y};
            }
        }
    }

    std::vector<std::pair<int, int>> path;
    int cx = goal_x, cy = goal_y;
    while (!(cx == start_x && cy == start_y)) {
        path.emplace_back(cx, cy);
        std::tie(cx, cy) = came_from[toKey(cx, cy)];
        if (cx == -1 && cy == -1) return {}; // no path
    }
    path.emplace_back(start_x, start_y);
    std::reverse(path.begin(), path.end());
    return path;
}
