#include <chrono>
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <vector>

struct Position {
  int x{0};
  int y{0};
};

struct SimulationParams {
  unsigned int seed{0};
  int dimX{40};
  int dimY{40};
  int simulationTicks{100};
  double minConfidenceThreshold{0.40};
  int numMovingCars{0};
  int numMovingBikes{0};
  int numParkedCars{0};
  int numStopSigns{0};
  int numTrafficLights{0};
  std::vector<Position> gpsPath;
};

class GridWorld;

class WorldObject {
 public:
  WorldObject(std::string id, char glyph, Position position, const GridWorld* world)
      : id_(std::move(id)), glyph_(glyph), position_(position), world_(world) {}
  virtual ~WorldObject() = default;

  const std::string& id() const { return id_; }
  char glyph() const { return glyph_; }
  const Position& position() const { return position_; }
  bool markedForRemoval() const { return marked_for_removal_; }

  virtual void update() {}

 protected:
  void setPosition(const Position& position) { position_ = position; }
  void markForRemoval() { marked_for_removal_ = true; }
  const GridWorld* world() const { return world_; }

 private:
  std::string id_;
  char glyph_;
  Position position_;
  bool marked_for_removal_{false};
  const GridWorld* world_;
};

class GridWorld {
 public:
  GridWorld(int width, int height) : width_(width), height_(height) {}

  bool inBounds(const Position& position) const {
    return position.x >= 0 && position.y >= 0 && position.x < width_ &&
           position.y < height_;
  }

  void updateAllObjects();
  void populate(const SimulationParams& params, std::mt19937& rng);

  int width() const { return width_; }
  int height() const { return height_; }
  const std::vector<std::unique_ptr<WorldObject>>& getObjects() const { return objects_; }

 private:
  int width_;
  int height_;
  std::vector<std::unique_ptr<WorldObject>> objects_;
};

namespace {

template <typename T>
T my_clamp(T value, T minimum, T maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

std::string generateId(const std::string& category) {
  static std::unordered_map<std::string, int> counts;
  int next = ++counts[category];
  return category + ":" + std::to_string(next);
}

std::string toString(const Position& position) {
  std::ostringstream stream;
  stream << "(" << position.x << ", " << position.y << ")";
  return stream.str();
}

}  // namespace

enum class Direction { Up, Down, Left, Right };

class StaticObject : public WorldObject {
 public:
  StaticObject(std::string id, char glyph, Position position, const GridWorld* world)
      : WorldObject(std::move(id), glyph, position, world) {}
};

class StopSign : public StaticObject {
 public:
  StopSign(const Position& position, const GridWorld* world)
      : StaticObject(generateId("STOP"), 'S', position, world) {
    std::cout << "[+STOP: " << id() << "] Initialized at " << toString(position) << ".\n";
  }

  ~StopSign() override { std::cout << "[-STOP: " << id() << "] Removed.\n"; }
};

class ParkedCar : public StaticObject {
 public:
  ParkedCar(const Position& position, const GridWorld* world)
      : StaticObject(generateId("PARKED_CAR"), 'P', position, world) {
    std::cout << "[+PARKED_CAR: " << id() << "] Initialized at " << toString(position)
              << ".\n";
  }

  ~ParkedCar() override { std::cout << "[-PARKED_CAR: " << id() << "] Removed.\n"; }
};

class TrafficLight : public StaticObject {
 public:
  enum class State { Red, Yellow, Green };

  TrafficLight(const Position& position, const GridWorld* world, std::mt19937& rng)
      : StaticObject(generateId("LIGHT"), 'L', position, world),
        state_(randomState(rng)) {
    std::cout << "[+LIGHT: " << id() << "] Initialized at " << toString(position)
              << " to " << stateName(state_) << ".\n";
  }

  ~TrafficLight() override { std::cout << "[-LIGHT: " << id() << "] Removed.\n"; }

  void update() override {
    ++ticks_in_state_;
    if (ticks_in_state_ >= ticksForState(state_)) {
      ticks_in_state_ = 0;
      state_ = nextState(state_);
    }
  }

  std::string currentStateName() const { return stateName(state_); }

 private:
  static State randomState(std::mt19937& rng) {
    std::uniform_int_distribution<int> dist(0, 2);
    return static_cast<State>(dist(rng));
  }

  static int ticksForState(State state) {
    switch (state) {
      case State::Red:
        return 4;
      case State::Green:
        return 8;
      case State::Yellow:
        return 2;
    }
    return 4;
  }

  static State nextState(State state) {
    switch (state) {
      case State::Red:
        return State::Green;
      case State::Green:
        return State::Yellow;
      case State::Yellow:
        return State::Red;
    }
    return State::Red;
  }

  static std::string stateName(State state) {
    switch (state) {
      case State::Red:
        return "RED";
      case State::Green:
        return "GREEN";
      case State::Yellow:
        return "YELLOW";
    }
    return "RED";
  }

  State state_;
  int ticks_in_state_{0};
};

class MovingObject : public WorldObject {
 public:
  MovingObject(std::string id,
               char glyph,
               Position position,
               const GridWorld* world,
               int speed,
               Direction direction)
      : WorldObject(std::move(id), glyph, position, world),
        speed_(speed),
        direction_(direction) {}

  void update() override {
    Position next = position();
    switch (direction_) {
      case Direction::Up:
        next.y -= speed_;
        break;
      case Direction::Down:
        next.y += speed_;
        break;
      case Direction::Left:
        next.x -= speed_;
        break;
      case Direction::Right:
        next.x += speed_;
        break;
    }
    setPosition(next);
    if (world() && !world()->inBounds(next)) {
      markForRemoval();
    }
  }

 protected:
  int speed() const { return speed_; }
  Direction direction() const { return direction_; }

 private:
  int speed_;
  Direction direction_;
};

class MovingCar : public MovingObject {
 public:
  MovingCar(const Position& position,
            const GridWorld* world,
            int speed,
            Direction direction)
      : MovingObject(generateId("CAR"), 'C', position, world, speed, direction) {
    std::cout << "[+CAR: " << id() << "] Initialized at " << toString(position)
              << " moving at speed " << speed << ".\n";
  }

  ~MovingCar() override { std::cout << "[-CAR: " << id() << "] Removed.\n"; }
};

class MovingBike : public MovingObject {
 public:
  MovingBike(const Position& position,
             const GridWorld* world,
             int speed,
             Direction direction)
      : MovingObject(generateId("BIKE"), 'B', position, world, speed, direction) {
    std::cout << "[+BIKE: " << id() << "] Initialized at " << toString(position)
              << " moving at speed " << speed << ".\n";
  }

  ~MovingBike() override { std::cout << "[-BIKE: " << id() << "] Removed.\n"; }
};

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
  double calculateConfidence(double baseAccuracy, double distance, double maxRange) const {
    static std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<double> noise_dist(-0.05, 0.05);
    double confidence = baseAccuracy * (1.0 - (distance / maxRange)) + noise_dist(rng);
    return my_clamp(confidence, 0.0, 1.0);
  }
};

namespace {

std::string objectType(const WorldObject& object) {
  if (dynamic_cast<const MovingCar*>(&object)) {
    return "MovingCar";
  }
  if (dynamic_cast<const MovingBike*>(&object)) {
    return "MovingBike";
  }
  if (dynamic_cast<const ParkedCar*>(&object)) {
    return "ParkedCar";
  }
  if (dynamic_cast<const StopSign*>(&object)) {
    return "StopSign";
  }
  if (dynamic_cast<const TrafficLight*>(&object)) {
    return "TrafficLight";
  }
  if (dynamic_cast<const MovingObject*>(&object)) {
    return "MovingObject";
  }
  if (dynamic_cast<const StaticObject*>(&object)) {
    return "StaticObject";
  }
  return "WorldObject";
}

double distanceBetween(const Position& a, const Position& b) {
  return static_cast<double>(std::abs(a.x - b.x) + std::abs(a.y - b.y));
}

}  // namespace

class SensorFusion {
 public:
  std::vector<SensorReading> fuse(
      const std::vector<std::vector<SensorReading>>& allReadings,
      double minConfidence) const {
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
            (bucket.traffic_state == "UNKNOWN" ||
             reading.confidence > bucket.traffic_state_confidence)) {
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
      fused_reading.confidence = bucket.count > 0
                                     ? bucket.confidence_sum /
                                           static_cast<double>(bucket.count)
                                     : 0.0;
      fused_reading.trafficLightState = bucket.traffic_state;

      if (fused_reading.type == "MovingBike" ||
          fused_reading.confidence > minConfidence) {
        fused.push_back(std::move(fused_reading));
      }
    }
    return fused;
  }
};

class LidarSensor : public Sensor {
 public:
  std::vector<SensorReading> detect(const GridWorld& world,
                                    const Position& vehiclePos,
                                    Direction /*vehicleDir*/) override {
    constexpr double kRange = 9.0;
    constexpr double kBaseAccuracy = 0.99;
    std::vector<SensorReading> readings;
    for (const auto& object : world.getObjects()) {
      if (!object) {
        continue;
      }
      const Position& pos = object->position();
      if (std::abs(pos.x - vehiclePos.x) > 4 || std::abs(pos.y - vehiclePos.y) > 4) {
        continue;
      }
      double distance = distanceBetween(vehiclePos, pos);
      if (distance > kRange) {
        continue;
      }
      SensorReading reading;
      reading.position = pos;
      reading.type = objectType(*object);
      reading.id = object->id();
      reading.confidence = calculateConfidence(kBaseAccuracy, distance, kRange);
      readings.push_back(std::move(reading));
    }
    return readings;
  }
};

class RadarSensor : public Sensor {
 public:
  std::vector<SensorReading> detect(const GridWorld& world,
                                    const Position& vehiclePos,
                                    Direction vehicleDir) override {
    constexpr double kRange = 12.0;
    constexpr double kBaseAccuracy = 0.95;
    std::vector<SensorReading> readings;
    for (const auto& object : world.getObjects()) {
      if (!object || !dynamic_cast<const MovingObject*>(object.get())) {
        continue;
      }
      const Position& pos = object->position();
      bool in_line = false;
      double distance = 0.0;
      switch (vehicleDir) {
        case Direction::Up:
          in_line = pos.x == vehiclePos.x && pos.y <= vehiclePos.y - 1 &&
                    pos.y >= vehiclePos.y - static_cast<int>(kRange);
          distance = static_cast<double>(vehiclePos.y - pos.y);
          break;
        case Direction::Down:
          in_line = pos.x == vehiclePos.x && pos.y >= vehiclePos.y + 1 &&
                    pos.y <= vehiclePos.y + static_cast<int>(kRange);
          distance = static_cast<double>(pos.y - vehiclePos.y);
          break;
        case Direction::Left:
          in_line = pos.y == vehiclePos.y && pos.x <= vehiclePos.x - 1 &&
                    pos.x >= vehiclePos.x - static_cast<int>(kRange);
          distance = static_cast<double>(vehiclePos.x - pos.x);
          break;
        case Direction::Right:
          in_line = pos.y == vehiclePos.y && pos.x >= vehiclePos.x + 1 &&
                    pos.x <= vehiclePos.x + static_cast<int>(kRange);
          distance = static_cast<double>(pos.x - vehiclePos.x);
          break;
      }
      if (!in_line) {
        continue;
      }
      SensorReading reading;
      reading.position = pos;
      reading.type = objectType(*object);
      reading.id = object->id();
      reading.confidence = calculateConfidence(kBaseAccuracy, distance, kRange);
      readings.push_back(std::move(reading));
    }
    return readings;
  }
};

class CameraSensor : public Sensor {
 public:
  std::vector<SensorReading> detect(const GridWorld& world,
                                    const Position& vehiclePos,
                                    Direction vehicleDir) override {
    constexpr double kRange = 7.0;
    constexpr double kBaseAccuracy = 0.90;
    std::vector<SensorReading> readings;
    Position center = vehiclePos;
    switch (vehicleDir) {
      case Direction::Up:
        center.y -= 4;
        break;
      case Direction::Down:
        center.y += 4;
        break;
      case Direction::Left:
        center.x -= 4;
        break;
      case Direction::Right:
        center.x += 4;
        break;
    }
    for (const auto& object : world.getObjects()) {
      if (!object) {
        continue;
      }
      const Position& pos = object->position();
      if (std::abs(pos.x - center.x) > 3 || std::abs(pos.y - center.y) > 3) {
        continue;
      }
      double distance = distanceBetween(vehiclePos, pos);
      if (distance > kRange) {
        continue;
      }
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
};

void GridWorld::updateAllObjects() {
  for (const auto& object : objects_) {
    if (object) {
      object->update();
    }
  }
  auto it = std::remove_if(objects_.begin(), objects_.end(),
                           [](const std::unique_ptr<WorldObject>& object) {
                             return object && object->markedForRemoval();
                           });
  objects_.erase(it, objects_.end());
}

void GridWorld::populate(const SimulationParams& params, std::mt19937& rng) {
  const int capacity = width_ * height_;
  const int total_objects =
      params.numMovingCars + params.numMovingBikes + params.numParkedCars +
      params.numStopSigns + params.numTrafficLights;
  if (total_objects > capacity) {
    throw std::runtime_error("Too many objects for the grid size.");
  }

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

  auto randomDirection = [&]() {
    return static_cast<Direction>(direction_dist(rng));
  };

  for (int i = 0; i < params.numMovingCars; ++i) {
    Position position = randomPosition();
    objects_.push_back(std::make_unique<MovingCar>(position, this, 1, randomDirection()));
  }

  for (int i = 0; i < params.numMovingBikes; ++i) {
    Position position = randomPosition();
    objects_.push_back(std::make_unique<MovingBike>(position, this, 1, randomDirection()));
  }

  for (int i = 0; i < params.numParkedCars; ++i) {
    Position position = randomPosition();
    objects_.push_back(std::make_unique<ParkedCar>(position, this));
  }

  for (int i = 0; i < params.numStopSigns; ++i) {
    Position position = randomPosition();
    objects_.push_back(std::make_unique<StopSign>(position, this));
  }

  for (int i = 0; i < params.numTrafficLights; ++i) {
    Position position = randomPosition();
    objects_.push_back(std::make_unique<TrafficLight>(position, this, rng));
  }
}

class AutonomousVehicle {
 public:
  AutonomousVehicle(Position start, std::vector<Position> gps_path)
      : position_(start), gps_path_(std::move(gps_path)) {
    sensors_.push_back(std::make_unique<LidarSensor>());
    sensors_.push_back(std::make_unique<RadarSensor>());
    sensors_.push_back(std::make_unique<CameraSensor>());
  }

  void update() {
    decideNextMove();
  }

  void sense(const GridWorld& world, double minConfidence) {
    std::size_t total_readings = 0;
    std::vector<std::vector<SensorReading>> all_readings;
    all_readings.reserve(sensors_.size());
    for (const auto& sensor : sensors_) {
      if (!sensor) {
        continue;
      }
      std::vector<SensorReading> readings = sensor->detect(world, position_, direction_);
      total_readings += readings.size();
      all_readings.push_back(std::move(readings));
    }
    fused_data_ = fusion_engine_.fuse(all_readings, minConfidence);
    std::cout << "[FUSION] Fused " << fused_data_.size() << " unique objects from "
              << total_readings << " total readings.\n";
  }

  const Position& position() const { return position_; }
  Direction direction() const { return direction_; }

 private:
  bool checkEmergencyStop(const std::vector<SensorReading>& objects) {
    for (const auto& object : objects) {
      bool in_front = false;
      switch (direction_) {
        case Direction::Up:
          in_front = object.position.y < position_.y;
          break;
        case Direction::Down:
          in_front = object.position.y > position_.y;
          break;
        case Direction::Left:
          in_front = object.position.x < position_.x;
          break;
        case Direction::Right:
          in_front = object.position.x > position_.x;
          break;
      }
      if (!in_front) {
        continue;
      }

      double distance = distanceBetween(position_, object.position);
      if (distance < 2.0) {
        return true;
      }
      if (object.type == "TrafficLight" &&
          (object.trafficLightState == "RED" || object.trafficLightState == "YELLOW") &&
          distance < 3.0) {
        return true;
      }
    }
    return false;
  }

  void decideNextMove() {
    if (checkEmergencyStop(fused_data_)) {
      speed_state_ = 0;
      return;
    }
    speed_state_ = std::min(speed_state_ + 1, 2);

    if (current_gps_index_ >= static_cast<int>(gps_path_.size())) {
      speed_state_ = 0;
      return;
    }

    Position target = gps_path_[current_gps_index_];
    if (distanceBetween(position_, target) == 0.0) {
      ++current_gps_index_;
      if (current_gps_index_ >= static_cast<int>(gps_path_.size())) {
        std::cout << "Destination Reached!\n";
        speed_state_ = 0;
        return;
      }
      target = gps_path_[current_gps_index_];
    }

    if (position_.x < target.x) {
      direction_ = Direction::Right;
    } else if (position_.x > target.x) {
      direction_ = Direction::Left;
    } else if (position_.y < target.y) {
      direction_ = Direction::Down;
    } else if (position_.y > target.y) {
      direction_ = Direction::Up;
    }

    std::cout << "[NAV] Pos: " << toString(position_) << " -> Target: "
              << toString(target) << " | Speed: " << speed_state_ << ".\n";

    if (speed_state_ == 0) {
      return;
    }

    Position next = position_;
    switch (direction_) {
      case Direction::Up: {
        int delta = std::min(speed_state_, std::abs(position_.y - target.y));
        next.y -= delta;
        break;
      }
      case Direction::Down: {
        int delta = std::min(speed_state_, std::abs(position_.y - target.y));
        next.y += delta;
        break;
      }
      case Direction::Left: {
        int delta = std::min(speed_state_, std::abs(position_.x - target.x));
        next.x -= delta;
        break;
      }
      case Direction::Right: {
        int delta = std::min(speed_state_, std::abs(position_.x - target.x));
        next.x += delta;
        break;
      }
    }
    position_ = next;
  }

  Position position_;
  Direction direction_{Direction::Up};
  std::vector<std::unique_ptr<Sensor>> sensors_;
  SensorFusion fusion_engine_;
  std::vector<SensorReading> fused_data_;
  std::vector<Position> gps_path_;
  int current_gps_index_{0};
  int speed_state_{0};
};

namespace {

std::string usageMessage(const char* programName) {
  std::ostringstream message;
  message << "Usage: " << programName << " [options]\n"
          << "Options:\n"
          << "  --seed <n>                     Random seed (default: current time)\n"
          << "  --dimX <n>                     Grid width (default: 40)\n"
          << "  --dimY <n>                     Grid height (default: 40)\n"
          << "  --simulationTicks <n>          Simulation tick count (default: 100)\n"
          << "  --minConfidenceThreshold <n>   Minimum confidence threshold (default: 0.40)\n"
          << "  --numMovingCars <n>            Number of moving cars\n"
          << "  --numMovingBikes <n>           Number of moving bikes\n"
          << "  --numParkedCars <n>            Number of parked cars\n"
          << "  --numStopSigns <n>             Number of stop signs\n"
          << "  --numTrafficLights <n>         Number of traffic lights\n"
          << "  --gps <x1> <y1> <x2> <y2>...    Required GPS coordinates (pairs)\n"
          << "  --help                         Show this help message\n";
  return message.str();
}

bool isFlag(const std::string& arg) {
  return arg.rfind("--", 0) == 0;
}

int parseInt(const std::string& value, const std::string& flag) {
  try {
    size_t idx = 0;
    int parsed = std::stoi(value, &idx);
    if (idx != value.size()) {
      throw std::invalid_argument("extra characters");
    }
    return parsed;
  } catch (const std::exception&) {
    throw std::runtime_error("Invalid value for " + flag + ": " + value);
  }
}

unsigned int parseUInt(const std::string& value, const std::string& flag) {
  try {
    size_t idx = 0;
    unsigned long parsed = std::stoul(value, &idx);
    if (idx != value.size()) {
      throw std::invalid_argument("extra characters");
    }
    return static_cast<unsigned int>(parsed);
  } catch (const std::exception&) {
    throw std::runtime_error("Invalid value for " + flag + ": " + value);
  }
}

double parseDouble(const std::string& value, const std::string& flag) {
  try {
    size_t idx = 0;
    double parsed = std::stod(value, &idx);
    if (idx != value.size()) {
      throw std::invalid_argument("extra characters");
    }
    return parsed;
  } catch (const std::exception&) {
    throw std::runtime_error("Invalid value for " + flag + ": " + value);
  }
}

SimulationParams parseArgs(int argc, char* argv[]) {
  SimulationParams params;
  params.seed = static_cast<unsigned int>(
      std::chrono::system_clock::now().time_since_epoch().count());

  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--help") {
      std::cout << usageMessage(argv[0]);
      std::exit(0);
    } else if (arg == "--seed") {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for --seed");
      }
      params.seed = parseUInt(argv[++i], "--seed");
    } else if (arg == "--dimX") {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for --dimX");
      }
      params.dimX = parseInt(argv[++i], "--dimX");
    } else if (arg == "--dimY") {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for --dimY");
      }
      params.dimY = parseInt(argv[++i], "--dimY");
    } else if (arg == "--simulationTicks") {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for --simulationTicks");
      }
      params.simulationTicks = parseInt(argv[++i], "--simulationTicks");
    } else if (arg == "--minConfidenceThreshold") {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for --minConfidenceThreshold");
      }
      params.minConfidenceThreshold =
          parseDouble(argv[++i], "--minConfidenceThreshold");
    } else if (arg == "--numMovingCars") {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for --numMovingCars");
      }
      params.numMovingCars = parseInt(argv[++i], "--numMovingCars");
    } else if (arg == "--numMovingBikes") {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for --numMovingBikes");
      }
      params.numMovingBikes = parseInt(argv[++i], "--numMovingBikes");
    } else if (arg == "--numParkedCars") {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for --numParkedCars");
      }
      params.numParkedCars = parseInt(argv[++i], "--numParkedCars");
    } else if (arg == "--numStopSigns") {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for --numStopSigns");
      }
      params.numStopSigns = parseInt(argv[++i], "--numStopSigns");
    } else if (arg == "--numTrafficLights") {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing value for --numTrafficLights");
      }
      params.numTrafficLights = parseInt(argv[++i], "--numTrafficLights");
    } else if (arg == "--gps") {
      if (i + 1 >= argc) {
        throw std::runtime_error("Missing coordinates for --gps");
      }

      while (i + 1 < argc && !isFlag(argv[i + 1])) {
        if (i + 2 >= argc || isFlag(argv[i + 2])) {
          throw std::runtime_error("GPS coordinates must be in x y pairs");
        }
        int x = parseInt(argv[++i], "--gps");
        int y = parseInt(argv[++i], "--gps");
        params.gpsPath.push_back({x, y});
      }
    } else {
      throw std::runtime_error("Unknown argument: " + arg);
    }
  }

  if (params.gpsPath.empty()) {
    throw std::runtime_error("--gps is required and must contain at least one pair");
  }

  return params;
}

}  // namespace

int main(int argc, char* argv[]) {
  SimulationParams params;
  try {
    params = parseArgs(argc, argv);
  } catch (const std::exception& ex) {
    std::cerr << "Error: " << ex.what() << "\n\n";
    std::cerr << usageMessage(argv[0]);
    return 1;
  }

  std::mt19937 rng(params.seed);
  GridWorld world(params.dimX, params.dimY);
  world.populate(params, rng);
  AutonomousVehicle vehicle(params.gpsPath.front(), params.gpsPath);

  for (int tick = 0; tick < params.simulationTicks; ++tick) {
    world.updateAllObjects();
    vehicle.update();
    vehicle.sense(world, params.minConfidenceThreshold);

    if (!world.inBounds(vehicle.position())) {
      std::cout << "Vehicle left the grid at tick " << tick << ".\n";
      break;
    }
  }

  return 0;
}
