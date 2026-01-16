#pragma once
#include <vector>
#include <string>
#include "Common.h"

// Forward declarations
class GridWorld;

struct SensorReading {
  Position position;
  std::string type;
  std::string id;
  double confidence;
  std::string trafficLightState{"UNKNOWN"};
};

class Sensor {
 public:
  virtual ~Sensor() = default;
  virtual std::vector<SensorReading> detect(const GridWorld& world,
                                            const Position& vehiclePos,
                                            Direction vehicleDir) = 0;
 protected:
  double calculateConfidence(double baseAccuracy, double distance, double maxRange) const;
};

class SensorFusion {
 public:
  std::vector<SensorReading> fuse(const std::vector<std::vector<SensorReading>>& allReadings, double minConfidence) const;
};

class LidarSensor : public Sensor {
 public:
  std::vector<SensorReading> detect(const GridWorld& world, const Position& vehiclePos, Direction) override;
};

class RadarSensor : public Sensor {
 public:
  std::vector<SensorReading> detect(const GridWorld& world, const Position& vehiclePos, Direction vehicleDir) override;
};

class CameraSensor : public Sensor {
 public:
  std::vector<SensorReading> detect(const GridWorld& world, const Position& vehiclePos, Direction vehicleDir) override;
};