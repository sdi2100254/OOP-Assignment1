#include "Vehicle.h"
#include "GridWorld.h"
#include <iostream>
#include <algorithm>

AutonomousVehicle::AutonomousVehicle(Position start, std::vector<Position> gps_path)
    : position_(start), gps_path_(std::move(gps_path)) {
  sensors_.push_back(std::make_unique<LidarSensor>());
  sensors_.push_back(std::make_unique<RadarSensor>());
  sensors_.push_back(std::make_unique<CameraSensor>());
  std::cout << "[+VEHICLE] Created at " << toString(start) << ".\n";
}

void AutonomousVehicle::update() { decideNextMove(); }

void AutonomousVehicle::sense(const GridWorld& world, double minConfidence) {
  std::vector<std::vector<SensorReading>> all_readings;
  for (const auto& sensor : sensors_) {
    if (sensor) all_readings.push_back(sensor->detect(world, position_, direction_));
  }
  fused_data_ = fusion_engine_.fuse(all_readings, minConfidence);
}

const Position& AutonomousVehicle::position() const { return position_; }
Direction AutonomousVehicle::direction() const { return direction_; }
int AutonomousVehicle::speed() const { return speed_state_; }

Position AutonomousVehicle::target() const { 
    if(current_gps_index_ < static_cast<int>(gps_path_.size())) return gps_path_[current_gps_index_];
    return position_;
}

bool AutonomousVehicle::checkEmergencyStop(const std::vector<SensorReading>& objects) {
  for (const auto& object : objects) {
    bool in_front = false;
    switch (direction_) {
      case Direction::Up:    in_front = object.position.y < position_.y; break;
      case Direction::Down:  in_front = object.position.y > position_.y; break;
      case Direction::Left:  in_front = object.position.x < position_.x; break;
      case Direction::Right: in_front = object.position.x > position_.x; break;
    }
    if (!in_front) continue;

    double distance = distanceBetween(position_, object.position);
    
    // Decelerate/Stop conditions
    if (distance <= 2.0) return true; // Obstacle too close
    if (object.type == "TrafficLight" && (object.trafficLightState == "RED" || object.trafficLightState == "YELLOW") && distance < 3.0) {
      return true; // Light too close
    }
  }
  return false;
}

void AutonomousVehicle::decideNextMove() {
  // 1. Safety Check
  if (checkEmergencyStop(fused_data_)) {
    speed_state_ = 0;
    return;
  }

  // 2. Navigation Target
  if (current_gps_index_ >= static_cast<int>(gps_path_.size())) {
    speed_state_ = 0;
    return;
  }
  Position targetPos = gps_path_[current_gps_index_];

  // Check if reached
  if (distanceBetween(position_, targetPos) == 0.0) {
    ++current_gps_index_;
    if (current_gps_index_ >= static_cast<int>(gps_path_.size())) {
      speed_state_ = 0;
      return;
    }
    targetPos = gps_path_[current_gps_index_];
  }

  // 3. Deceleration Logic
  double distToTarget = distanceBetween(position_, targetPos);
  if (distToTarget <= 5.0) {
      speed_state_ = 1; // HALF_SPEED
  } else {
      speed_state_ = std::min(speed_state_ + 1, 2); // Accelerate up to FULL_SPEED
  }

  // 4. Steering (Manhattan)
  if (position_.x < targetPos.x) direction_ = Direction::Right;
  else if (position_.x > targetPos.x) direction_ = Direction::Left;
  else if (position_.y < targetPos.y) direction_ = Direction::Down;
  else if (position_.y > targetPos.y) direction_ = Direction::Up;

  if (speed_state_ == 0) return;

  // 5. Movement
  Position next = position_;
  switch (direction_) {
    case Direction::Up:    next.y -= std::min(speed_state_, static_cast<int>(std::abs(position_.y - targetPos.y))); break;
    case Direction::Down:  next.y += std::min(speed_state_, static_cast<int>(std::abs(position_.y - targetPos.y))); break;
    case Direction::Left:  next.x -= std::min(speed_state_, static_cast<int>(std::abs(position_.x - targetPos.x))); break;
    case Direction::Right: next.x += std::min(speed_state_, static_cast<int>(std::abs(position_.x - targetPos.x))); break;
  }
  position_ = next;
}