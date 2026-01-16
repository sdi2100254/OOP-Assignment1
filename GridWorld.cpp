#include "GridWorld.h"
#include "WorldObject.h" 
#include "StaticObjects.h" 
#include "MovingObjects.h"
#include <iostream>
#include <algorithm>

GridWorld::GridWorld(int width, int height) : width_(width), height_(height) {}

bool GridWorld::inBounds(const Position& position) const {
  return position.x >= 0 && position.y >= 0 && position.x < width_ && position.y < height_;
}
