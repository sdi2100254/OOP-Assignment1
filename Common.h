#pragma once

#include <string>
#include <vector>
#include <cmath>
#include <sstream>
#include <unordered_map>
#include <random>

// ==========================================
//           Data Structures
// ==========================================

struct Position {
  int x{0};
  int y{0};
};

enum class Direction { Up, Down, Left, Right };

struct SimulationParams {
  unsigned int seed{0};
  int dimX{40};
  int dimY{40};
  int simulationTicks{100};
  double minConfidenceThreshold{0.40};
  int numMovingCars{0};
  int numMovingBikes{0};
  int numParkedCars{0};
  int numStopSigns{0};
  int numTrafficLights{0};
  std::vector<Position> gpsPath;
};

// ==========================================
//           Helper Functions
// ==========================================

// Template must be in the header
template <typename T>
T my_clamp(T value, T minimum, T maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

// Inline ensures these functions can be included in multiple files without error
inline std::string generateId(const std::string& category) {
  static std::unordered_map<std::string, int> counts;
  int next = ++counts[category];
  return category + ":" + std::to_string(next);
}

inline std::string toString(const Position& position) {
  std::ostringstream stream;
  stream << "(" << position.x << ", " << position.y << ")";
  return stream.str();
}

inline double distanceBetween(const Position& a, const Position& b) {
  // Use Manhattan Distance as per Section 1.3
  return static_cast<double>(std::abs(a.x - b.x) + std::abs(a.y - b.y));
}