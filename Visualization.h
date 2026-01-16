#pragma once
#include "GridWorld.h"
#include "Vehicle.h"
#include <string>

class Visualization {
 public:
  void displayFull(const GridWorld& world, const AutonomousVehicle& vehicle) const;
  void displayPOV(const GridWorld& world, const AutonomousVehicle& vehicle) const;

 private:
  void printColored(char glyph) const;
  char glyphAt(const GridWorld& world, const Position& position, const AutonomousVehicle* vehicle) const;
  
  // ANSI Color Codes
  const std::string RESET = "\033[0m";
  const std::string RED = "\033[31m";
  const std::string GREEN = "\033[32m";
  const std::string YELLOW = "\033[33m";
  const std::string BLUE = "\033[34m";
  const std::string MAGENTA = "\033[35m";
  const std::string CYAN = "\033[36m";
};