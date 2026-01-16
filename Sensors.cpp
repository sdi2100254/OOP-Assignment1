#include "Sensors.h"
#include "GridWorld.h"
#include "WorldObject.h"
#include "MovingObjects.h"
#include "StaticObjects.h"
#include <random>
#include <unordered_map>
#include <cmath>

// Helper to identify object types
namespace {
std::string objectType(const WorldObject& object) {
  if (dynamic_cast<const MovingCar*>(&object)) return "MovingCar";
  if (dynamic_cast<const MovingBike*>(&object)) return "MovingBike";
  if (dynamic_cast<const ParkedCar*>(&object)) return "ParkedCar";
  if (dynamic_cast<const StopSign*>(&object)) return "StopSign";
  if (dynamic_cast<const TrafficLight*>(&object)) return "TrafficLight";
  if (dynamic_cast<const MovingObject*>(&object)) return "MovingObject";
  if (dynamic_cast<const StaticObject*>(&object)) return "StaticObject";
  return "WorldObject";
}
}

// Sensor Base
double Sensor::calculateConfidence(double baseAccuracy, double distance, double maxRange) const {
  static std::mt19937 rng(std::random_device{}());
  std::uniform_real_distribution<double> noise_dist(-0.05, 0.05);
  double confidence = baseAccuracy * (1.0 - (distance / maxRange)) + noise_dist(rng);
  return my_clamp(confidence, 0.0, 1.0);
}

// Sensor Fusion
std::vector<SensorReading> SensorFusion::fuse(const std::vector<std::vector<SensorReading>>& allReadings, double minConfidence) const {
  struct Aggregate {
    SensorReading best_reading;
    double confidence_sum{0.0};
    std::size_t count{0};
    std::string traffic_state{"UNKNOWN"};
    double traffic_state_confidence{0.0};
  };

  std::unordered_map<std::string, Aggregate> grouped;
  for (const auto& readings : allReadings) {
    for (const auto& reading : readings) {
      auto& bucket = grouped[reading.id];
      if (bucket.count == 0 || reading.confidence > bucket.best_reading.confidence) {
        bucket.best_reading = reading;
      }
      if (reading.trafficLightState != "UNKNOWN" &&
          (bucket.traffic_state == "UNKNOWN" || reading.confidence > bucket.traffic_state_confidence)) {
        bucket.traffic_state = reading.trafficLightState;
        bucket.traffic_state_confidence = reading.confidence;
      }
      bucket.confidence_sum += reading.confidence;
      ++bucket.count;
    }
  }

  std::vector<SensorReading> fused;
  fused.reserve(grouped.size());
  for (const auto& entry : grouped) {
    const auto& bucket = entry.second;
    SensorReading fused_reading = bucket.best_reading;
    fused_reading.confidence = bucket.count > 0 ? bucket.confidence_sum / static_cast<double>(bucket.count) : 0.0;
    fused_reading.trafficLightState = bucket.traffic_state;

    // "If a sensor detects a bike, it is not rejected" (Section 3)
    if (fused_reading.type == "MovingBike" || fused_reading.confidence > minConfidence) {
      fused.push_back(std::move(fused_reading));
    }
  }
  return fused;
}

// Lidar Sensor
std::vector<SensorReading> LidarSensor::detect(const GridWorld& world, const Position& vehiclePos, Direction) {
  constexpr double kRange = 9.0;
  constexpr double kBaseAccuracy = 0.99;
  std::vector<SensorReading> readings;
  for (const auto& object : world.getObjects()) {
    if (!object) continue;
    const Position& pos = object->position();
    // 9x9 centered = +/- 4 cells
    if (std::abs(pos.x - vehiclePos.x) > 4 || std::abs(pos.y - vehiclePos.y) > 4) continue;
    double distance = distanceBetween(vehiclePos, pos);
    if (distance > kRange) continue;
    
    SensorReading reading;
    reading.position = pos;
    reading.type = objectType(*object);
    reading.id = object->id();
    reading.confidence = calculateConfidence(kBaseAccuracy, distance, kRange);
    readings.push_back(std::move(reading));
  }
  return readings;
}

// Radar Sensor
std::vector<SensorReading> RadarSensor::detect(const GridWorld& world, const Position& vehiclePos, Direction vehicleDir) {
  constexpr double kRange = 12.0;
  constexpr double kBaseAccuracy = 0.95;
  std::vector<SensorReading> readings;
  for (const auto& object : world.getObjects()) {
    if (!object || !dynamic_cast<const MovingObject*>(object.get())) continue;
    const Position& pos = object->position();
    bool in_line = false;
    double distance = 0.0;
    // "12 cells straight ahead" (Section 2.2.2)
    switch (vehicleDir) {
      case Direction::Up:
        in_line = pos.x == vehiclePos.x && pos.y <= vehiclePos.y - 1 && pos.y >= vehiclePos.y - static_cast<int>(kRange);
        distance = static_cast<double>(vehiclePos.y - pos.y);
        break;
      case Direction::Down:
        in_line = pos.x == vehiclePos.x && pos.y >= vehiclePos.y + 1 && pos.y <= vehiclePos.y + static_cast<int>(kRange);
        distance = static_cast<double>(pos.y - vehiclePos.y);
        break;
      case Direction::Left:
        in_line = pos.y == vehiclePos.y && pos.x <= vehiclePos.x - 1 && pos.x >= vehiclePos.x - static_cast<int>(kRange);
        distance = static_cast<double>(vehiclePos.x - pos.x);
        break;
      case Direction::Right:
        in_line = pos.y == vehiclePos.y && pos.x >= vehiclePos.x + 1 && pos.x <= vehiclePos.x + static_cast<int>(kRange);
        distance = static_cast<double>(pos.x - vehiclePos.x);
        break;
    }
    if (!in_line) continue;
    SensorReading reading;
    reading.position = pos;
    reading.type = objectType(*object);
    reading.id = object->id();
    reading.confidence = calculateConfidence(kBaseAccuracy, distance, kRange);
    readings.push_back(std::move(reading));
  }
  return readings;
}

// Camera Sensor
std::vector<SensorReading> CameraSensor::detect(const GridWorld& world, const Position& vehiclePos, Direction vehicleDir) {
  constexpr double kRange = 7.0;
  constexpr double kBaseAccuracy = 0.90;
  std::vector<SensorReading> readings;
  Position center = vehiclePos;
  // 7x7 square directly in front (Section 2.2.3)
  switch (vehicleDir) {
    case Direction::Up:    center.y -= 4; break;
    case Direction::Down:  center.y += 4; break;
    case Direction::Left:  center.x -= 4; break;
    case Direction::Right: center.x += 4; break;
  }
  for (const auto& object : world.getObjects()) {
    if (!object) continue;
    const Position& pos = object->position();
    // +/- 3 cells around the center = 7x7
    if (std::abs(pos.x - center.x) > 3 || std::abs(pos.y - center.y) > 3) continue;
    double distance = distanceBetween(vehiclePos, pos);
    if (distance > kRange) continue;
    
    SensorReading reading;
    reading.position = pos;
    reading.type = objectType(*object);
    reading.id = object->id();
    reading.confidence = calculateConfidence(kBaseAccuracy, distance, kRange);
    if (const auto* light = dynamic_cast<const TrafficLight*>(object.get())) {
      reading.trafficLightState = light->currentStateName();
    }
    readings.push_back(std::move(reading));
  }
  return readings;
}