#include "GridWorld.h"
#include "WorldObject.h"
#include "StaticObjects.h"
#include "MovingObjects.h"
#include <iostream>
#include <algorithm>
#include <unordered_set> //Αναγκαστικο για το USED_positions 

GridWorld::GridWorld(int width, int height) : width_(width), height_(height) {}

// Destructor 
GridWorld::~GridWorld() = default;

bool GridWorld::inBounds(const Position& position) const {
  return position.x >= 0 && position.y >= 0 && position.x < width_ &&
         position.y < height_;
}

void GridWorld::updateAllObjects() {
  for (const auto& object : objects_) {
    if (object) object->update();
  }
  auto it = std::remove_if(objects_.begin(), objects_.end(),
                           [](const std::unique_ptr<WorldObject>& object) {
                             return object && object->markedForRemoval();
                           });
  objects_.erase(it, objects_.end());
}

void GridWorld::populate(const SimulationParams& params, std::mt19937& rng) {
  const int capacity = width_ * height_;
  const int total_objects = params.numMovingCars + params.numMovingBikes + params.numParkedCars +
                            params.numStopSigns + params.numTrafficLights;
  if (total_objects > capacity) throw std::runtime_error("Too many objects for the grid size.");

  objects_.clear();
  std::unordered_set<int> used_positions;
  std::uniform_int_distribution<int> x_dist(0, width_ - 1);
  std::uniform_int_distribution<int> y_dist(0, height_ - 1);
  std::uniform_int_distribution<int> direction_dist(0, 3);

  auto randomPosition = [&]() {
    Position position;
    int index = 0;
    do {
      position = {x_dist(rng), y_dist(rng)};
      index = position.y * width_ + position.x;
    } while (!used_positions.insert(index).second);
    return position;
  };
  auto randomDirection = [&]() { return static_cast<Direction>(direction_dist(rng)); };

  for (int i = 0; i < params.numMovingCars; ++i) objects_.push_back(std::make_unique<MovingCar>(randomPosition(), this, 1, randomDirection()));
  for (int i = 0; i < params.numMovingBikes; ++i) objects_.push_back(std::make_unique<MovingBike>(randomPosition(), this, 1, randomDirection()));
  for (int i = 0; i < params.numParkedCars; ++i) objects_.push_back(std::make_unique<ParkedCar>(randomPosition(), this));
  for (int i = 0; i < params.numStopSigns; ++i) objects_.push_back(std::make_unique<StopSign>(randomPosition(), this));
  for (int i = 0; i < params.numTrafficLights; ++i) objects_.push_back(std::make_unique<TrafficLight>(randomPosition(), this, rng));
}

int GridWorld::width() const { return width_; }
int GridWorld::height() const { return height_; }
const std::vector<std::unique_ptr<WorldObject>>& GridWorld::getObjects() const { return objects_; }