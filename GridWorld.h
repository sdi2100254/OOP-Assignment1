#pragma once
#include <vector>
#include <memory>
#include <random>
#include "Common.h" // For SimulationParams

// Forward declaration to avoid circular includes if needed
class WorldObject; 

class GridWorld {
 public:
  GridWorld(int width, int height); // Constructor declaration only

  bool inBounds(const Position& position) const;
  void updateAllObjects();
  void populate(const SimulationParams& params, std::mt19937& rng);

  int width() const;
  int height() const;
  const std::vector<std::unique_ptr<WorldObject>>& getObjects() const;

 private:
  int width_;
  int height_;
  std::vector<std::unique_ptr<WorldObject>> objects_;
};