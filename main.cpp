#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <memory>
#include <random>
#include <sstream>
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

class WorldObject {
 public:
  virtual ~WorldObject() = default;
  virtual void update() {}
};

class GridWorld {
 public:
  GridWorld(int width, int height)
      : width_(width), height_(height), grid_(width * height, nullptr) {}

  bool inBounds(const Position& position) const {
    return position.x >= 0 && position.y >= 0 && position.x < width_ &&
           position.y < height_;
  }

  void updateAllObjects() {
    for (WorldObject* object : grid_) {
      if (object) {
        object->update();
      }
    }
  }

  int width() const { return width_; }
  int height() const { return height_; }

 private:
  int width_;
  int height_;
  std::vector<WorldObject*> grid_;
};

class AutonomousVehicle {
 public:
  explicit AutonomousVehicle(Position start) : position_(start) {}

  void update() {
    // Placeholder for future navigation logic.
  }

  const Position& position() const { return position_; }

 private:
  Position position_;
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
  AutonomousVehicle vehicle(params.gpsPath.front());

  for (int tick = 0; tick < params.simulationTicks; ++tick) {
    world.updateAllObjects();
    vehicle.update();

    if (!world.inBounds(vehicle.position())) {
      std::cout << "Vehicle left the grid at tick " << tick << ".\n";
      break;
    }
  }

  return 0;
}
