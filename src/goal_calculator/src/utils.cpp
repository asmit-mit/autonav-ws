#include <cmath>
#include <queue>
#include <unordered_set>
#include <vector>

struct WorldPose {
  double x;
  double y;

  WorldPose(double x_val = 0.0, double y_val = 0.0) : x(x_val), y(y_val) {}
};

struct MapPose {
  int x;
  int y;

  MapPose(int x_val = 0, int y_val = 0) : x(x_val), y(y_val) {}
};

struct Map {
  int width, height;
  double resolution;
  WorldPose origin;
  std::vector<int8_t> grid;

  int getIndex(int x, int y) const { return y * width + x; }

  bool isValid(int x, int y) const {
    return x >= 0 && x < width && y >= 0 && y < height;
  }

  int8_t getValue(int x, int y) const {
    if (!isValid(x, y))
      return -1;
    return grid[getIndex(x, y)];
  }

  void setValue(int x, int y, int8_t value) {
    if (isValid(x, y)) {
      grid[getIndex(x, y)] = value;
    }
  }
};

struct BotPose {
  WorldPose world_pose;
  MapPose map_pose;
  double roll, pitch, yaw;
};

class Utils {
public:
  Utils() {}

  static double getAngleRadians(const WorldPose &a, const WorldPose &b) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    return std::atan2(dy, dx);
  }

  static double mapDistance(const MapPose &a, const MapPose &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  static double worldDistance(const WorldPose &a, const WorldPose &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  static MapPose
  getMapPoseFromWorldPose(const WorldPose &pose, const Map &map) {
    MapPose map_pose;
    map_pose.x = static_cast<int>((pose.x - map.origin.x) / map.resolution);
    map_pose.y = static_cast<int>((pose.y - map.origin.y) / map.resolution);
    return map_pose;
  }

  static WorldPose
  getWorldPoseFromMapPose(const MapPose &pose, const Map &map) {
    WorldPose world_pose;
    world_pose.x = map.origin.x + (pose.x * map.resolution);
    world_pose.y = map.origin.y + (pose.y * map.resolution);
    return world_pose;
  }

  static MapPose findClosestForValue(
      const MapPose &pose, const Map &map, int radius, int value
  ) {
    const int dx[] = {0, 1, 0, -1, 1, -1, 1, -1};
    const int dy[] = {-1, 0, 1, 0, -1, 1, 1, -1};

    std::queue<std::pair<MapPose, int>> q;

    auto hashFunc = [&map](const MapPose &p) { return p.y * map.width + p.x; };

    std::unordered_set<int> visited;

    if (!map.isValid(pose.x, pose.y)) {
      return MapPose(-1, -1);
    }

    q.push({pose, 0});
    visited.insert(hashFunc(pose));

    while (!q.empty()) {
      auto pair = q.front();
      q.pop();

      MapPose current = pair.first;
      int distance    = pair.second;

      if (map.getValue(current.x, current.y) == value) {
        return current;
      }

      if (distance >= radius) {
        continue;
      }

      for (int i = 0; i < 4; i++) {
        int nx = current.x + dx[i];
        int ny = current.y + dy[i];
        MapPose neighbor(nx, ny);
        int hash = hashFunc(neighbor);

        if (map.isValid(nx, ny) && visited.find(hash) == visited.end()) {
          visited.insert(hash);
          q.push({neighbor, distance + 1});
        }
      }
    }

    return MapPose(-1, -1);
  }

  static MapPose exploreMiddleLane(const MapPose &start, const Map &map) {
    const int dx[] = {0, 1, 0, -1, 1, -1, 1, -1};
    const int dy[] = {-1, 0, 1, 0, -1, 1, 1, -1};

    std::queue<MapPose> q;
    std::unordered_set<int> visited;

    auto hashFunc = [&map](const MapPose &p) { return p.y * map.width + p.x; };

    q.push(start);
    visited.insert(hashFunc(start));

    MapPose last_cell = start;

    while (!q.empty()) {
      MapPose current = q.front();
      q.pop();

      last_cell = current;

      for (int i = 0; i < 8; i++) {
        int nx = current.x + dx[i];
        int ny = current.y + dy[i];
        MapPose neighbor(nx, ny);
        int hash = hashFunc(neighbor);

        if (map.isValid(nx, ny) && visited.find(hash) == visited.end() &&
            map.getValue(nx, ny) == 100) {
          visited.insert(hash);
          q.push(neighbor);
        }
      }

      const int search_radius = 10;
      for (int dy = -search_radius; dy <= search_radius; ++dy) {
        for (int dx = -search_radius; dx <= search_radius; ++dx) {
          int nx = current.x + dx;
          int ny = current.y + dy;
          MapPose neighbor(nx, ny);
          int hash = hashFunc(neighbor);

          if (map.isValid(nx, ny) && visited.find(hash) == visited.end() &&
              map.getValue(nx, ny) == 100) {
            visited.insert(hash);
            q.push(neighbor);
          }
        }
      }
    }

    return last_cell;
  }

  static void removeMapBehindBot(
      Map &map, const WorldPose &bot_pose, double angle, int height_to_remove,
      int width_to_remove
  ) {
    while (angle > M_PI)
      angle -= 2 * M_PI;
    while (angle < -M_PI)
      angle += 2 * M_PI;

    auto process_rows = [&](int start_y, int end_y) {
      for (int y = start_y; y < end_y; y++) {
        for (int x = 0; x < map.width; x++) {
          MapPose map_pose(x, y);
          WorldPose world_pose = Utils::getWorldPoseFromMapPose(map_pose, map);

          double angle_to_pixel = Utils::getAngleRadians(bot_pose, world_pose);

          double angle_diff = angle_to_pixel - angle;

          while (angle_diff > M_PI)
            angle_diff -= 2 * M_PI;
          while (angle_diff < -M_PI)
            angle_diff += 2 * M_PI;

          double dx = world_pose.x - bot_pose.x;
          double dy = world_pose.y - bot_pose.y;

          if (std::abs(angle_diff) > M_PI / 2 &&
              std::abs(dx) <= width_to_remove &&
              std::abs(dy) <= height_to_remove) {
            map.setValue(x, y, -1);
          }
        }
      }
    };

    process_rows(0, map.height);
  }
};
