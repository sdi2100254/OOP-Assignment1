#include <iostream>
#include <thread>
#include <chrono>
#include <cstdlib>
#include "Common.h"
#include "GridWorld.h"
#include "Vehicle.h"
#include "Visualization.h"

// ==========================================
//    CROSS-PLATFORM COMPATIBILITY
// ==========================================
#ifdef _WIN32
    #include <windows.h>
    void clearScreen() { std::system("cls"); }
    void sleepMs(int milliseconds) { Sleep(milliseconds); }
#else
    #include <unistd.h>
    void clearScreen() { std::system("clear"); }
    void sleepMs(int milliseconds) { std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds)); }
#endif

namespace {
std::string usageMessage(const char* programName) {
  return "Usage: " + std::string(programName) + " [options] --gps <x> <y> ...\n"; 
}
bool isFlag(const std::string& arg) { return arg.rfind("--", 0) == 0; }
int parseInt(const std::string& value, const std::string& flag) { return std::stoi(value); } 
unsigned int parseUInt(const std::string& value, const std::string& flag) { return std::stoul(value); }
double parseDouble(const std::string& value, const std::string& flag) { return std::stod(value); }

SimulationParams parseArgs(int argc, char* argv[]) {
  SimulationParams params;
  params.seed = static_cast<unsigned int>(std::chrono::system_clock::now().time_since_epoch().count());
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--seed") params.seed = parseUInt(argv[++i], "--seed");
    else if (arg == "--dimX") params.dimX = parseInt(argv[++i], "--dimX");
    else if (arg == "--dimY") params.dimY = parseInt(argv[++i], "--dimY");
    else if (arg == "--simulationTicks") params.simulationTicks = parseInt(argv[++i], "--simulationTicks");
    else if (arg == "--minConfidenceThreshold") params.minConfidenceThreshold = parseDouble(argv[++i], "--minConfidenceThreshold");
    else if (arg == "--numMovingCars") params.numMovingCars = parseInt(argv[++i], "--numMovingCars");
    else if (arg == "--numMovingBikes") params.numMovingBikes = parseInt(argv[++i], "--numMovingBikes");
    else if (arg == "--numParkedCars") params.numParkedCars = parseInt(argv[++i], "--numParkedCars");
    else if (arg == "--numStopSigns") params.numStopSigns = parseInt(argv[++i], "--numStopSigns");
    else if (arg == "--numTrafficLights") params.numTrafficLights = parseInt(argv[++i], "--numTrafficLights");
    else if (arg == "--gps") {
      while (i + 1 < argc && !isFlag(argv[i + 1])) {
        int x = parseInt(argv[++i], "--gps");
        int y = parseInt(argv[++i], "--gps");
        params.gpsPath.push_back({x, y});
      }
    }
    else if (arg == "--help") {
        std::cout << usageMessage(argv[0]);
        std::exit(0);
    }
  }
  if (params.gpsPath.empty()) throw std::runtime_error("No GPS path provided.");
  return params;
}
}

int main(int argc, char* argv[]) {
  SimulationParams params;
  try { params = parseArgs(argc, argv); } 
  catch (const std::exception& ex) { return 1; }

  std::mt19937 rng(params.seed);
  GridWorld world(params.dimX, params.dimY);
  world.populate(params, rng);
  AutonomousVehicle vehicle(params.gpsPath.front(), params.gpsPath);
  Visualization vis;

  // Visualization START (Full)
  vis.displayFull(world, vehicle);

  for (int tick = 0; tick < params.simulationTicks; ++tick) {
    clearScreen(); 
    
    world.updateAllObjects();
    vehicle.update();
    vehicle.sense(world, params.minConfidenceThreshold);
    vis.displayPOV(world, vehicle); 
    
    // HUD
    std::cout << "[Tick: " << tick << "] Speed: " << vehicle.speed() 
              << " | Target: (" << vehicle.target().x << ", " << vehicle.target().y << ")\n";

    sleepMs(200); 

    if (!world.inBounds(vehicle.position())) {
      std::cout << "Vehicle left the grid at tick " << tick << ".\n";
      break;
    }
  }

  // Visualization END (Full)
  vis.displayFull(world, vehicle);
  return 0;
}