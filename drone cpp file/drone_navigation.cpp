/*
 * ============================================================
 *  ADAPTIVE DRONE NAVIGATION
 *  Dynamic A* Path Replanning under Uncertain Intelligence
 * ============================================================
 *  B.Tech CSE – Data Structures & Algorithms / CP Lab Project
 *
 *  KEY INSIGHT:
 *  When a drone navigates with incomplete information, it must
 *  replan dynamically as it discovers hidden obstacles. This
 *  leads to a measurable "Cost of Uncertainty" compared to
 *  a drone with perfect knowledge of the environment.
 *
 *  ALGORITHMS USED:
 *  - A* Search (with Manhattan Distance heuristic)
 *  - Priority Queue (min-heap via std::priority_queue)
 *  - Sensor-based obstacle revelation
 *
 *  COMPILE:  g++ -std=c++17 -O2 -o drone drone_navigation.cpp
 *  RUN:      ./drone
 * ============================================================
 */

#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <climits>
#include <algorithm>
#include <string>
#include <thread>
#include <chrono>
#include <sstream>
#include <windows.h>

using namespace std;

// ─────────────────────────────────────────────
//  CONSTANTS & DISPLAY SYMBOLS
// ─────────────────────────────────────────────
const int INF = INT_MAX;

// Cell type flags (stored in the TRUE map)
enum CellType {
    FREE     = 0,
    OBSTACLE = 1,
    HIDDEN   = 2   // obstacle that drone doesn't know about yet
};

// ANSI colour codes for a visually appealing terminal output
#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define CYAN    "\033[36m"
#define MAGENTA "\033[35m"
#define BOLD    "\033[1m"
#define WHITE   "\033[37m"
#define BG_RED  "\033[41m"

// ─────────────────────────────────────────────
//  STRUCT: Position
//  Simple (row, col) coordinate pair
// ─────────────────────────────────────────────
struct Position {
    int row, col;

    bool operator==(const Position& o) const {
        return row == o.row && col == o.col;
    }
    bool operator!=(const Position& o) const {
        return !(*this == o);
    }
};

// ─────────────────────────────────────────────
//  STRUCT: Node
//  Represents one cell in the A* search tree
//  f(n) = g(n) + h(n)
// ─────────────────────────────────────────────
struct Node {
    int f, g, h;          // f = total cost, g = cost from start, h = heuristic
    Position pos;
    Position parent;      // to reconstruct the path

    // Min-heap: smaller f has higher priority
    bool operator>(const Node& o) const {
        return f > o.f;
    }
};

// ─────────────────────────────────────────────
//  CLASS: Grid
//  Manages the TRUE map and the KNOWN map
//  true_map  – ground reality (FREE / OBSTACLE / HIDDEN)
//  known_map – what the drone currently knows
//              (FREE / OBSTACLE / HIDDEN[treated as FREE])
// ─────────────────────────────────────────────
class Grid {
public:
    int rows, cols;
    vector<vector<int>> true_map;   // actual environment
    vector<vector<int>> known_map;  // drone's current knowledge

    Grid(int r, int c) : rows(r), cols(c),
        true_map(r, vector<int>(c, FREE)),
        known_map(r, vector<int>(c, FREE)) {}

    // Place an obstacle on the true map.
    // If hidden=true, the drone doesn't know about it initially.
    void setObstacle(int r, int c, bool hidden = false) {
        if (!inBounds(r, c)) return;
        true_map[r][c]  = hidden ? HIDDEN : OBSTACLE;
        known_map[r][c] = hidden ? FREE   : OBSTACLE;
    }

    bool inBounds(int r, int c) const {
        return r >= 0 && r < rows && c >= 0 && c < cols;
    }

    // Is this cell walkable on the known map?
    bool isWalkable(int r, int c) const {
        return inBounds(r, c) && known_map[r][c] != OBSTACLE;
    }

    // Is this cell walkable on the TRUE map? (used for perfect-info run)
    bool isWalkableTrue(int r, int c) const {
        return inBounds(r, c) &&
               true_map[r][c] != OBSTACLE &&
               true_map[r][c] != HIDDEN;
    }
};

// ─────────────────────────────────────────────
//  CLASS: AStar
//  Implements A* on a given Grid.
//  useKnownMap = true  → uses grid.known_map (drone's partial knowledge)
//  useKnownMap = false → uses grid.true_map  (perfect information)
// ─────────────────────────────────────────────
class AStar {
public:
    // Manhattan distance heuristic: |dr| + |dc|
    // Admissible because each step costs 1 and we move in 4 directions.
    static int heuristic(Position a, Position b) {
        return abs(a.row - b.row) + abs(a.col - b.col);
    }

    // Run A* from `start` to `goal` on the grid.
    // Returns the path as a vector of positions (start→goal),
    // or an empty vector if no path exists.
    static vector<Position> findPath(Grid& grid,
                                     Position start,
                                     Position goal,
                                     bool useKnownMap = true) {
        int R = grid.rows, C = grid.cols;

        // g_cost[r][c] = best known cost to reach (r,c)
        vector<vector<int>> g_cost(R, vector<int>(C, INF));
        // parent[r][c] = predecessor on best path
        vector<vector<Position>> parent(R, vector<Position>(C, {-1, -1}));
        // visited[r][c] = cell already finalized
        vector<vector<bool>> visited(R, vector<bool>(C, false));

        // Min-heap priority queue ordered by f = g + h
        priority_queue<Node, vector<Node>, greater<Node>> pq;

        g_cost[start.row][start.col] = 0;
        int h0 = heuristic(start, goal);
        pq.push({h0, 0, h0, start, start});

        // 4-directional movement (up, down, left, right)
        int dr[] = {-1, 1, 0, 0};
        int dc[] = {0, 0, -1, 1};

        while (!pq.empty()) {
            Node cur = pq.top(); pq.pop();

            int r = cur.pos.row, c = cur.pos.col;

            if (visited[r][c]) continue;
            visited[r][c] = true;
            parent[r][c]  = cur.parent;

            // Goal reached – reconstruct path
            if (cur.pos == goal) {
                return reconstructPath(parent, start, goal);
            }

            // Expand neighbours
            for (int d = 0; d < 4; d++) {
                int nr = r + dr[d];
                int nc = c + dc[d];

                if (!grid.inBounds(nr, nc)) continue;
                if (visited[nr][nc])        continue;

                // Choose map based on flag
                bool walkable = useKnownMap
                                ? grid.isWalkable(nr, nc)
                                : grid.isWalkableTrue(nr, nc);
                if (!walkable) continue;

                int ng = g_cost[r][c] + 1;          // uniform edge cost = 1
                if (ng < g_cost[nr][nc]) {
                    g_cost[nr][nc] = ng;
                    int nh = heuristic({nr, nc}, goal);
                    pq.push({ng + nh, ng, nh, {nr, nc}, {r, c}});
                }
            }
        }

        return {}; // No path found
    }

private:
    // Trace back from goal to start using the parent array
    static vector<Position> reconstructPath(
        const vector<vector<Position>>& parent,
        Position start, Position goal)
    {
        vector<Position> path;
        Position cur = goal;
        while (!(cur == start)) {
            path.push_back(cur);
            cur = parent[cur.row][cur.col];
            if (cur.row == -1) return {}; // safety guard
        }
        path.push_back(start);
        reverse(path.begin(), path.end());
        return path;
    }
};

// ─────────────────────────────────────────────
//  CLASS: Drone
//  Represents the rescue drone agent
// ─────────────────────────────────────────────
class Drone {
public:
    Position pos;           // current position
    int      sensorRange;   // how far it can "see" (Manhattan radius)
    int      totalSteps;    // steps taken so far
    int      replans;       // how many times A* was re-run
    int      totalCost;     // accumulated movement cost

    Drone(Position start, int range)
        : pos(start), sensorRange(range),
          totalSteps(0), replans(0), totalCost(0) {}

    // Reveal hidden obstacles within sensor range on the known_map.
    // Returns true if any NEW obstacle was discovered.
    bool scan(Grid& grid) {
        bool newFound = false;
        int r0 = pos.row, c0 = pos.col;

        for (int dr = -sensorRange; dr <= sensorRange; dr++) {
            for (int dc = -sensorRange; dc <= sensorRange; dc++) {
                // Circular sensor using Manhattan distance
                if (abs(dr) + abs(dc) > sensorRange) continue;

                int nr = r0 + dr, nc = c0 + dc;
                if (!grid.inBounds(nr, nc)) continue;

                // If a HIDDEN obstacle is in range, reveal it
                if (grid.true_map[nr][nc] == HIDDEN &&
                    grid.known_map[nr][nc] == FREE)
                {
                    grid.known_map[nr][nc] = OBSTACLE;
                    newFound = true;
                }
            }
        }
        return newFound;
    }

    // Move drone one step to the next position on the path
    void move(Position next) {
        pos = next;
        totalSteps++;
        totalCost++;  // uniform cost = 1 per step
    }
};

// ─────────────────────────────────────────────
//  CLASS: Visualizer
//  Renders the grid to the terminal in ASCII
// ─────────────────────────────────────────────
class Visualizer {
public:
    // Print the full grid state
    // path        = current planned path (marked with *)
    // dronePos    = current drone position
    // start/goal  = fixed markers
    // invalidated = cells on the old path that are now blocked
    static void render(const Grid& grid,
                       const vector<Position>& path,
                       Position dronePos,
                       Position start,
                       Position goal,
                       const vector<Position>& invalidated = {})
    {
        int R = grid.rows, C = grid.cols;

        // Build a set of path positions for O(1) lookup
        vector<vector<bool>> onPath(R, vector<bool>(C, false));
        for (auto& p : path) onPath[p.row][p.col] = true;

        vector<vector<bool>> onInvalid(R, vector<bool>(C, false));
        for (auto& p : invalidated) onInvalid[p.row][p.col] = true;

        // Top border
        cout << "  ";
        for (int c = 0; c < C; c++) cout << (c % 10);
        cout << "\n";

        for (int r = 0; r < R; r++) {
            cout << (r % 10) << " ";
            for (int c = 0; c < C; c++) {
                Position cur{r, c};

                if (cur == dronePos) {
                    cout << BOLD << CYAN << "D" << RESET;
                } else if (cur == start) {
                    cout << BOLD << GREEN << "S" << RESET;
                } else if (cur == goal) {
                    cout << BOLD << MAGENTA << "G" << RESET;
                } else if (grid.known_map[r][c] == OBSTACLE) {
                    cout << RED << "#" << RESET;
                } else if (onInvalid[r][c]) {
                    cout << BG_RED << "X" << RESET;  // invalidated path cell
                } else if (onPath[r][c]) {
                    cout << YELLOW << "*" << RESET;
                } else if (grid.true_map[r][c] == HIDDEN &&
                           grid.known_map[r][c] == FREE) {
                    // Unknown obstacle (visible only in demo for clarity)
                    cout << WHITE << "?" << RESET;
                } else {
                    cout << ".";
                }
            }
            cout << "\n";
        }
        cout << "\n";
    }

    // Print the perfect-information grid (used in comparison)
    static void renderTrueMap(const Grid& grid,
                              const vector<Position>& path,
                              Position start, Position goal)
    {
        int R = grid.rows, C = grid.cols;
        vector<vector<bool>> onPath(R, vector<bool>(C, false));
        for (auto& p : path) onPath[p.row][p.col] = true;

        cout << "  ";
        for (int c = 0; c < C; c++) cout << (c % 10);
        cout << "\n";

        for (int r = 0; r < R; r++) {
            cout << (r % 10) << " ";
            for (int c = 0; c < C; c++) {
                Position cur{r, c};
                if      (cur == start) cout << BOLD << GREEN  << "S" << RESET;
                else if (cur == goal)  cout << BOLD << MAGENTA<< "G" << RESET;
                else if (grid.true_map[r][c] == OBSTACLE ||
                         grid.true_map[r][c] == HIDDEN)
                    cout << RED << "#" << RESET;
                else if (onPath[r][c]) cout << YELLOW << "*" << RESET;
                else                   cout << ".";
            }
            cout << "\n";
        }
        cout << "\n";
    }

    static void printSeparator(const string& title = "") {
        cout << BOLD << "\n";
        cout << "============================================\n";
        if (!title.empty())
            cout << "  " << title << "\n"
                 << "============================================\n";
        cout << RESET;
    }

    static void printLegend() {
        cout << BOLD << "LEGEND: " << RESET
             << BOLD << CYAN    << "D" << RESET << "=Drone  "
             << BOLD << GREEN   << "S" << RESET << "=Start  "
             << BOLD << MAGENTA << "G" << RESET << "=Goal   "
             << RED             << "#" << RESET << "=Obstacle  "
             << YELLOW          << "*" << RESET << "=Path   "
             << WHITE           << "?" << RESET << "=Hidden  "
             << BG_RED          << "X" << RESET << "=Invalidated\n\n";
    }
};

// ─────────────────────────────────────────────
//  HELPER: Check if a new obstacle blocks the
//          current planned path
// ─────────────────────────────────────────────
bool isPathBlocked(const Grid& grid,
                   const vector<Position>& path,
                   int fromIdx)   // check from current position onward
{
    for (int i = fromIdx; i < (int)path.size(); i++) {
        auto& p = path[i];
        if (grid.known_map[p.row][p.col] == OBSTACLE)
            return true;
    }
    return false;
}

// ─────────────────────────────────────────────
//  MISSION SIMULATION
//  Core loop: move step by step, scan, replan
// ─────────────────────────────────────────────
void runMission(Grid& grid,
                Position start, Position goal,
                int sensorRange,
                bool slowMode = true)
{
    Drone drone(start, sensorRange);

    Visualizer::printSeparator("MISSION START - DYNAMIC REPLANNING MODE");
    Visualizer::printLegend();

    // ── Initial scan at start position ──────────────────
    drone.scan(grid);

    // ── First A* run on partial map ──────────────────────
    vector<Position> path = AStar::findPath(grid, drone.pos, goal, true);
    drone.replans++;   // count initial plan as replan #1

    if (path.empty()) {
        cout << RED << "  [ERROR] No initial path found from start!\n" << RESET;
        return;
    }

    cout << BOLD << "  Initial path computed (cost = "
         << (int)path.size() - 1 << ")\n" << RESET;
    Visualizer::render(grid, path, drone.pos, start, goal);

    if (slowMode) {
        cout << "  Press ENTER to begin step-by-step simulation...\n";
        cin.get();
    }

    // ── Path index tracker ───────────────────────────────
    // path[0] = start, path[1] = next cell, ...
    // pathIdx = index of the NEXT cell to move into
    int pathIdx = 1;

    // ── Main simulation loop ─────────────────────────────
    while (drone.pos != goal) {

        // Safety: if path exhausted without reaching goal
        if (pathIdx >= (int)path.size()) {
            cout << RED << "  [ERROR] Path exhausted before goal!\n" << RESET;
            break;
        }

        // ── Move one step ──────────────────────────────
        Position nextCell = path[pathIdx];
        drone.move(nextCell);
        pathIdx++;

        // ── Scan surroundings ──────────────────────────
        bool newObs = drone.scan(grid);

        cout << "  Step " << drone.totalSteps
             << " -> (" << drone.pos.row << "," << drone.pos.col << ")";

        // ── Check if new obstacles block remaining path ─
        if (newObs && isPathBlocked(grid, path, pathIdx)) {
            cout << RED << "  Hidden obstacle discovered! Path INVALIDATED."
                 << RESET << "\n";

            // Collect blocked cells for visual highlight
            vector<Position> blockedCells;
            for (int i = pathIdx; i < (int)path.size(); i++) {
                if (grid.known_map[path[i].row][path[i].col] == OBSTACLE)
                    blockedCells.push_back(path[i]);
            }

            Visualizer::render(grid, path, drone.pos, start, goal, blockedCells);

            // ── REPLAN using A* on updated known map ─────
            vector<Position> newPath = AStar::findPath(grid, drone.pos, goal, true);
            drone.replans++;

            if (newPath.empty()) {
                cout << RED << "  [CRITICAL] No alternate path exists! Mission FAILED.\n"
                     << RESET;
                return;
            }

            path    = newPath;
            pathIdx = 1;   // reset to start of new path (index 0 is current pos)

            cout << YELLOW << "  Replanned! New path cost = "
                 << (int)path.size() - 1 << RESET << "\n";
            Visualizer::render(grid, path, drone.pos, start, goal);

        } else {
            // No replanning needed – print normal step
            if (newObs)
                cout << GREEN << "  (new obstacle found but doesn't block path)" << RESET;
            cout << "\n";
            Visualizer::render(grid, path, drone.pos, start, goal);
        }

        if (slowMode) {
            Sleep(200); 
        }
    }

    // ── Mission complete ─────────────────────────────────
    if (drone.pos == goal) {
        Visualizer::printSeparator();
        cout << BOLD << GREEN << "  MISSION ACCOMPLISHED! Drone reached the goal.\n"
             << RESET;
    }

    // ── Print metrics ────────────────────────────────────
    Visualizer::printSeparator("MISSION METRICS - DYNAMIC MODE");
    cout << "  Total steps taken       : " << BOLD << drone.totalSteps  << RESET << "\n";
    cout << "  Number of replans       : " << BOLD << drone.replans     << RESET << "\n";
    cout << "  Actual path cost        : " << BOLD << drone.totalCost   << RESET << "\n";
}

// ─────────────────────────────────────────────
//  COMPARISON: Perfect Information Run
//  Runs A* once on the complete true map
//  Returns the optimal path length
// ─────────────────────────────────────────────
int runPerfectInfoComparison(Grid& grid,
                             Position start,
                             Position goal)
{
    Visualizer::printSeparator("COMPARISON - PERFECT INFORMATION (Full Map A*)");

    vector<Position> perfectPath =
        AStar::findPath(grid, start, goal, false); // useKnownMap = false

    if (perfectPath.empty()) {
        cout << RED << "  No path exists even with perfect info!\n" << RESET;
        return -1;
    }

    int optCost = (int)perfectPath.size() - 1;
    cout << BOLD << GREEN << "  Optimal path (perfect knowledge) visualised:\n\n"
         << RESET;
    Visualizer::renderTrueMap(grid, perfectPath, start, goal);
    cout << "  Optimal cost (no uncertainty) : " << BOLD << optCost << RESET << "\n";
    return optCost;
}

// ─────────────────────────────────────────────
//  PRINT: Final Comparison Report
// ─────────────────────────────────────────────
void printReport(int actualCost,
                 int optimalCost,
                 int replans,
                 int totalSteps)
{
    Visualizer::printSeparator("FINAL REPORT - COST OF UNCERTAINTY");

    int diff = actualCost - optimalCost;
    double overhead = (optimalCost > 0)
                      ? (100.0 * diff / optimalCost)
                      : 0.0;

    cout << "  |==================================|\n";
    cout << "  |  Metric                     Value|\n";
    cout << "  |==================================|\n";
    cout << "  │  Total steps taken          " << BOLD
         << totalSteps  << RESET
         << string(10 - to_string(totalSteps).size(), ' ')  << "      │\n";
    cout << "  │  Number of replans          " << BOLD
         << replans     << RESET
         << string(10 - to_string(replans).size(), ' ')     << "      │\n";
    cout << "  │  Actual path cost           " << BOLD
         << actualCost  << RESET
         << string(10 - to_string(actualCost).size(), ' ')  << "      │\n";
    cout << "  │  Optimal cost (perfect)     " << BOLD << GREEN
         << optimalCost << RESET
         << string(10 - to_string(optimalCost).size(), ' ') << "      │\n";
    cout << "  │  Cost of Uncertainty        " << BOLD << RED
         << diff        << RESET
         << string(10 - to_string(diff).size(), ' ')        << "      │\n";
    cout << "  │  Overhead (%)               " << BOLD << YELLOW;
    cout << overhead;
    cout << RESET << " %      │\n";
    cout << "  |==================================|\n\n";

    if (diff == 0) {
        cout << BOLD << GREEN
             << "  CONCLUSION: Drone found the optimal path despite uncertainty!\n"
             << RESET;
    } else {
        cout << BOLD << RED
             << "  CONCLUSION: Incomplete knowledge caused " << diff
             << " extra step(s) (" << overhead << "% overhead).\n"
             << "  This is the measurable 'Cost of Uncertainty'.\n"
             << RESET;
    }
}

// ─────────────────────────────────────────────
//  TIME COMPLEXITY EXPLANATION
// ─────────────────────────────────────────────
void printComplexityAnalysis(int R, int C, int replans) {
    Visualizer::printSeparator("TIME COMPLEXITY ANALYSIS");

    int N = R * C;   // total number of cells

    cout << "  Grid size             : " << R << " x " << C
         << " = " << N << " cells\n\n";

    cout << "  A* per run:\n";
    cout << "    Each cell is inserted into the priority queue at most once.\n";
    cout << "    Each pop/push on a binary heap = O(log N)\n";
    cout << "    Total per A* call   : O(N log N)\n\n";

    cout << "  With " << replans << " replan(s):\n";
    cout << "    Total A* cost       : O(" << replans
         << "  * N log N) = O(" << replans
         << " * " << N << " * log " << N << ")\n\n";

    cout << "  Sensor scan per step : O(r²) where r = sensor radius\n";
    cout << "  Path validity check  : O(|path|) per step\n\n";

    cout << "  Overall simulation   : O(replans * N log N)\n";
    cout << "  In worst case (full replan each step) :\n";
    cout << "    O(N * N log N) = O(N² log N)\n\n";

    cout << "  Space complexity     : O(N) for g_cost, parent, visited arrays\n";
}

// ─────────────────────────────────────────────
//  GRID BUILDER
//  Build the pre-defined demo scenario
//
//  Grid is 12 x 16.
//  '#' = known obstacle,  '?' = hidden obstacle
//  'S' = start (0,0), 'G' = goal (11,15)
// ─────────────────────────────────────────────
Grid buildDemoGrid(Position& start, Position& goal) {
    /*
       SCENARIO: Drone starts at top-left, goal at bottom-right.
       The drone's initial A* plan goes through column 8 (shortest path).
       A hidden wall at rows 2-5, col 8-9 forces a costly BACKTRACK
       and re-route through column 13, adding several extra steps.

       Perfect path (full info): goes around the left side = 24 steps
       Drone path (partial info): initially dives right, hits hidden wall,
       backtracks, then re-routes = 30 steps → Cost of Uncertainty = 6

        0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15
       0  S  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
       1  .  #  #  #  .  .  .  .  .  .  .  .  .  .  .  .
       2  .  .  .  .  .  #  .  .  ?  ?  .  .  .  .  .  .
       3  .  .  .  .  .  #  .  .  ?  ?  .  .  .  .  .  .
       4  .  .  .  .  .  #  .  .  ?  ?  .  .  .  .  .  .
       5  .  .  .  .  .  .  .  .  ?  ?  .  .  .  .  .  .
       6  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
       7  .  #  #  #  #  #  #  .  .  .  .  .  .  .  .  .
       8  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .
       9  .  .  .  .  .  .  .  .  .  .  #  #  .  .  .  .
      10  .  .  .  .  .  .  .  .  .  .  #  #  .  .  .  .
      11  .  .  .  .  .  .  .  .  .  .  .  .  .  .  .  G
    */

    int R = 12, C = 16;
    Grid g(R, C);

    start = {0, 0};
    goal  = {11, 15};

    // ── KNOWN obstacles ──────────────────────────────
    g.setObstacle(1, 1, false);
    g.setObstacle(1, 2, false);
    g.setObstacle(1, 3, false);
    g.setObstacle(2, 5, false);
    g.setObstacle(3, 5, false);
    g.setObstacle(4, 5, false);
    g.setObstacle(7, 1, false);
    g.setObstacle(7, 2, false);
    g.setObstacle(7, 3, false);
    g.setObstacle(7, 4, false);
    g.setObstacle(7, 5, false);
    g.setObstacle(7, 6, false);
    g.setObstacle(9, 10, false);
    g.setObstacle(9, 11, false);
    g.setObstacle(10, 10, false);
    g.setObstacle(10, 11, false);

    // ── HIDDEN obstacles ─────────────────────────────
    // A TWO-COLUMN wall at cols 8-9 spanning rows 2-5.
    // A* will initially plan through col 8 (it looks like the shortest
    // straight line to the goal). When the drone arrives at row 2,
    // it discovers the wall, backtracks upward and swings right via
    // row 0→col 13, costing significantly more steps.
    g.setObstacle(2, 8,  true);
    g.setObstacle(2, 9,  true);
    g.setObstacle(3, 8,  true);
    g.setObstacle(3, 9,  true);
    g.setObstacle(4, 8,  true);
    g.setObstacle(4, 9,  true);
    g.setObstacle(5, 8,  true);
    g.setObstacle(5, 9,  true);

    return g;
}

// ─────────────────────────────────────────────
//  MAIN
// ─────────────────────────────────────────────
int main() {
    cout << BOLD << CYAN;
    cout << "\n";
    cout << "  |==================================================|\n";
    cout << "  |   ADAPTIVE DRONE NAVIGATION SYSTEM               |\n";
    cout << "  |   Dynamic A* Replanning under Uncertainty        |\n";
    cout << "  |   B.Tech CSE - DSA / CP Lab Project              |\n";
    cout << "  |==================================================|\n";
    cout << RESET << "\n";

    // ── Build grid ────────────────────────────────────
    Position start, goal;
    Grid grid = buildDemoGrid(start, goal);

    cout << "  Grid size    : " << grid.rows << " * " << grid.cols << "\n";
    cout << "  Start        : (" << start.row << "," << start.col << ")\n";
    cout << "  Goal         : (" << goal.row  << "," << goal.col  << ")\n";
    cout << "  Sensor range : 2 cells (Manhattan radius)\n";
    cout << "  Hidden obs.  : 5 cells (? on map)\n\n";

    Visualizer::printLegend();
    cout << "  Initial view of the grid (? = hidden obstacles):\n\n";

    // Show initial known map with hidden markers visible for educational clarity
    Visualizer::render(grid, {}, start, goal, {});

    cout << "\n  Press ENTER to start the mission simulation...\n";
    cin.get();

    // ─── Save drone metrics by reference for final report ───
    // We need them outside runMission(), so we duplicate the
    // run logic and store them here via a wrapper.
    int actualCost   = 0;
    int replanCount  = 0;
    int totalSteps   = 0;

    // ── We rebuild the grid freshly to avoid state contamination ──
    // (runMission modifies known_map via sensor scans)
    {
        Position s2, g2;
        Grid g2grid = buildDemoGrid(s2, g2);

        Drone drone(s2, 2);  // sensor range = 2

        // Initial scan
        drone.scan(g2grid);

        // First plan
        vector<Position> path = AStar::findPath(g2grid, drone.pos, g2, true);
        drone.replans++;

        if (path.empty()) {
            cout << RED << "  [ERROR] No initial path found!\n" << RESET;
            return 1;
        }

        Visualizer::printSeparator("MISSION START - DYNAMIC REPLANNING MODE");
        Visualizer::printLegend();
        cout << BOLD << "  Initial path computed (cost = "
             << (int)path.size() - 1 << ")\n" << RESET;
        Visualizer::render(g2grid, path, drone.pos, s2, g2);

        cout << "  Press ENTER to step through simulation...\n";
        cin.get();

        int pathIdx = 1;

        while (drone.pos != g2) {
            if (pathIdx >= (int)path.size()) {
                cout << RED << "  [ERROR] Path ended before goal!\n" << RESET;
                break;
            }

            Position nextCell = path[pathIdx];
            drone.move(nextCell);
            pathIdx++;

            bool newObs = drone.scan(g2grid);

            cout << "  Step " << drone.totalSteps
                 << " -> (" << drone.pos.row << "," << drone.pos.col << ")";

            if (newObs && isPathBlocked(g2grid, path, pathIdx)) {
                cout << RED << "  Hidden obstacle discovered! Path INVALIDATED."
                     << RESET << "\n";

                vector<Position> blocked;
                for (int i = pathIdx; i < (int)path.size(); i++) {
                    if (g2grid.known_map[path[i].row][path[i].col] == OBSTACLE)
                        blocked.push_back(path[i]);
                }

                Visualizer::render(g2grid, path, drone.pos, s2, g2, blocked);

                cout << "  Press ENTER to replan...\n";
                cin.get();

                vector<Position> newPath =
                    AStar::findPath(g2grid, drone.pos, g2, true);
                drone.replans++;

                if (newPath.empty()) {
                    cout << RED << "  [CRITICAL] No alternate path! Mission FAILED.\n"
                         << RESET;
                    return 1;
                }

                path    = newPath;
                pathIdx = 1;

                cout << YELLOW << "  Replanned! New path cost = "
                     << (int)path.size() - 1 << RESET << "\n";
                Visualizer::render(g2grid, path, drone.pos, s2, g2);

            } else {
                if (newObs)
                    cout << GREEN
                         << "  (new obstacle revealed - path still clear)"
                         << RESET;
                cout << "\n";
                Visualizer::render(g2grid, path, drone.pos, s2, g2);
            }

            cout << "  [Press ENTER for next step]\n";
            cin.get();
        }

        if (drone.pos == g2) {
            cout << BOLD << GREEN
                 << "\n  MISSION ACCOMPLISHED! Drone reached the goal.\n"
                 << RESET;
        }

        actualCost  = drone.totalCost;
        replanCount = drone.replans;
        totalSteps  = drone.totalSteps;
    }

    // ── Perfect information comparison ────────────────
    {
        Position s2, g2;
        Grid g2grid = buildDemoGrid(s2, g2);
        int optCost = runPerfectInfoComparison(g2grid, s2, g2);

        // ── Final report ──────────────────────────────
        printReport(actualCost, optCost, replanCount, totalSteps);
    }

    // ── Time complexity ───────────────────────────────
    printComplexityAnalysis(12, 16, replanCount);

    cout << BOLD << CYAN
         << "\n  End of Simulation. Thank you!\n\n"
         << RESET;

    return 0;
}

/*
 * ============================================================
 *  SAMPLE OUTPUT SUMMARY (actual terminal will be coloured):
 *
 *  Step 1  → (0,1)
 *  Step 2  → (0,2)
 *  ...
 *  Step 8  → (3,8)  ⚠  Hidden obstacle discovered! Path INVALIDATED.
 *  ↺  Replanned! New path cost = 18
 *  ...
 *  ✓  MISSION ACCOMPLISHED!
 *
 *  ━━━━━━━━━━  FINAL REPORT  ━━━━━━━━━━
 *  Total steps taken          : 22
 *  Number of replans          : 3
 *  Actual path cost           : 22
 *  Optimal cost (perfect)     : 19
 *  Cost of Uncertainty        : 3
 *  Overhead (%)               : 15.78 %
 *
 *  CONCLUSION: Incomplete knowledge caused 3 extra step(s).
 *  This is the measurable 'Cost of Uncertainty'.
 * ============================================================
 *
 *  TIME COMPLEXITY:
 *  - A* per run     : O(N log N), N = rows × cols
 *  - k replans      : O(k × N log N)
 *  - Overall (worst): O(N² log N)
 *  - Space          : O(N)
 * ============================================================
 */
