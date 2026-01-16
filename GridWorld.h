#pragma once
#include <vector>
#include <memory>
#include <random>
#include "Common.h"


class WorldObject; 

class GridWorld {
 public:
  GridWorld(int width, int height);
  ~GridWorld();

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