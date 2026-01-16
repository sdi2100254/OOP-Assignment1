#pragma once
#include "WorldObject.h"
#include <random>
#include <iostream>

class StaticObject : public WorldObject {
 public:
  StaticObject(std::string id, char glyph, Position position, const GridWorld* world);
};

class StopSign : public StaticObject {
 public:
  StopSign(const Position& position, const GridWorld* world);
};

class ParkedCar : public StaticObject {
 public:
  ParkedCar(const Position& position, const GridWorld* world);
};

class TrafficLight : public StaticObject {
 public:
  enum class State { Red, Yellow, Green };

  TrafficLight(const Position& position, const GridWorld* world, std::mt19937& rng);

  void update() override;
  std::string currentStateName() const;

 private:
  static State randomState(std::mt19937& rng);
  static int ticksForState(State state);
  static State nextState(State state);
  static std::string stateName(State state);

  State state_;
  int ticks_in_state_{0};
};