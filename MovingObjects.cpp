#include "MovingObjects.h"
#include "GridWorld.h" // Needed for inBounds check
#include <iostream>

// MovingObject
MovingObject::MovingObject(std::string id, char glyph, Position position, const GridWorld* world, int speed, Direction direction)
    : WorldObject(std::move(id), glyph, position, world),
      speed_(speed),
      direction_(direction) {}

void MovingObject::update() {
  Position next = position();
  switch (direction_) {
    case Direction::Up:    next.y -= speed_; break;
    case Direction::Down:  next.y += speed_; break;
    case Direction::Left:  next.x -= speed_; break;
    case Direction::Right: next.x += speed_; break;
  }
  setPosition(next);
  // Check bounds using the world pointer
  if (world() && !world()->inBounds(next)) {
    markForRemoval();
  }
}

int MovingObject::speed() const { return speed_; }
Direction MovingObject::direction() const { return direction_; }

// MovingCar
MovingCar::MovingCar(const Position& position, const GridWorld* world, int speed, Direction direction)
    : MovingObject(generateId("CAR"), 'C', position, world, speed, direction) {
  std::cout << "[+CAR: " << id() << "] Initialized at " << toString(position) << ".\n";
}

// MovingBike
MovingBike::MovingBike(const Position& position, const GridWorld* world, int speed, Direction direction)
    : MovingObject(generateId("BIKE"), 'B', position, world, speed, direction) {
  std::cout << "[+BIKE: " << id() << "] Initialized at " << toString(position) << ".\n";
}