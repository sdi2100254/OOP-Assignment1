#pragma once
#include <vector>
#include <memory>
#include "Common.h"
#include "Sensors.h"

// Forward declaration to avoid circular include
class GridWorld;

class AutonomousVehicle {
 public:
  AutonomousVehicle(Position start, std::vector<Position> gps_path);

  void update();
  void sense(const GridWorld& world, double minConfidence);

  const Position& position() const;
  Direction direction() const;
  int speed() const;
  Position target() const;

 private:
  bool checkEmergencyStop(const std::vector<SensorReading>& objects);
  void decideNextMove();

  Position position_;
  Direction direction_{Direction::Up};
  std::vector<std::unique_ptr<Sensor>> sensors_;
  SensorFusion fusion_engine_;
  std::vector<SensorReading> fused_data_;
  std::vector<Position> gps_path_;
  int current_gps_index_{0};
  int speed_state_{0};
};