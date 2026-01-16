#pragma once
#include "WorldObject.h"
#include "Common.h"

class MovingObject : public WorldObject {
 public:
  MovingObject(std::string id, char glyph, Position position, const GridWorld* world, int speed, Direction direction);

  void update() override;

 protected:
  int speed() const;
  Direction direction() const;

 private:
  int speed_;
  Direction direction_;
};

class MovingCar : public MovingObject {
 public:
  MovingCar(const Position& position, const GridWorld* world, int speed, Direction direction);
};

class MovingBike : public MovingObject {
 public:
  MovingBike(const Position& position, const GridWorld* world, int speed, Direction direction);
};