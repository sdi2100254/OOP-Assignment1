#include "StaticObjects.h"
#include "Common.h" // For helper functions like generateId

// StaticObject
StaticObject::StaticObject(std::string id, char glyph, Position position, const GridWorld* world)
    : WorldObject(std::move(id), glyph, position, world) {}

// StopSign
StopSign::StopSign(const Position& position, const GridWorld* world)
    : StaticObject(generateId("STOP"), 'S', position, world) {
  std::cout << "[+STOP: " << id() << "] Initialized at " << toString(position) << ".\n";
}

// ParkedCar
ParkedCar::ParkedCar(const Position& position, const GridWorld* world)
    : StaticObject(generateId("PARKED_CAR"), 'P', position, world) {
  std::cout << "[+PARKED_CAR: " << id() << "] Initialized at " << toString(position) << ".\n";
}

// TrafficLight
TrafficLight::TrafficLight(const Position& position, const GridWorld* world, std::mt19937& rng)
    : StaticObject(generateId("LIGHT"), 'L', position, world),
      state_(randomState(rng)) {
  std::cout << "[+LIGHT: " << id() << "] Initialized at " << toString(position) << " to " << stateName(state_) << ".\n";
}

void TrafficLight::update() {
  ++ticks_in_state_;
  if (ticks_in_state_ >= ticksForState(state_)) {
    ticks_in_state_ = 0;
    state_ = nextState(state_);
  }
}

std::string TrafficLight::currentStateName() const { return stateName(state_); }

TrafficLight::State TrafficLight::randomState(std::mt19937& rng) {
  std::uniform_int_distribution<int> dist(0, 2);
  return static_cast<State>(dist(rng));
}

int TrafficLight::ticksForState(State state) {
  switch (state) {
    case State::Red: return 4;
    case State::Green: return 8;
    case State::Yellow: return 2;
  }
  return 4;
}

TrafficLight::State TrafficLight::nextState(State state) {
  switch (state) {
    case State::Red: return State::Green;
    case State::Green: return State::Yellow;
    case State::Yellow: return State::Red;
  }
  return State::Red;
}

std::string TrafficLight::stateName(State state) {
  switch (state) {
    case State::Red: return "RED";
    case State::Green: return "GREEN";
    case State::Yellow: return "YELLOW";
  }
  return "RED";
}