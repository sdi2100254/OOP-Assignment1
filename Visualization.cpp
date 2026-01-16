#include "Visualization.h"
#include "StaticObjects.h"
#include "MovingObjects.h"
#include <iostream>

void Visualization::displayFull(const GridWorld& world, const AutonomousVehicle& vehicle) const {
  std::cout << "=== Full Grid View ===\n";
  for (int y = 0; y < world.height(); ++y) {
    for (int x = 0; x < world.width(); ++x) {
      printColored(glyphAt(world, {x, y}, &vehicle));
    }
    std::cout << "\n";
  }
}

void Visualization::displayPOV(const GridWorld& world, const AutonomousVehicle& vehicle) const {
  constexpr int kRadius = 5;
  const int view_size = kRadius * 2 + 1;
  std::cout << "=== Vehicle Camera Feed ===\n";
  std::cout << "+" << std::string(view_size, '-') << "+\n";
  for (int dy = -kRadius; dy <= kRadius; ++dy) {
    std::cout << "|";
    for (int dx = -kRadius; dx <= kRadius; ++dx) {
      Position pos{vehicle.position().x + dx, vehicle.position().y + dy};
      if (!world.inBounds(pos)) {
        std::cout << " ";
        continue;
      }
      printColored(glyphAt(world, pos, &vehicle));
    }
    std::cout << "|\n";
  }
  std::cout << "+" << std::string(view_size, '-') << "+\n";
}

void Visualization::printColored(char glyph) const {
    if (glyph == 'R' || glyph == 'S') std::cout << RED << glyph << RESET;
    else if (glyph == 'G') std::cout << GREEN << glyph << RESET;
    else if (glyph == 'Y') std::cout << YELLOW << glyph << RESET;
    else if (glyph == 'C') std::cout << BLUE << glyph << RESET;
    else if (glyph == 'B') std::cout << MAGENTA << glyph << RESET;
    else if (glyph == '@') std::cout << CYAN << glyph << RESET;
    else std::cout << glyph;
}

char Visualization::glyphAt(const GridWorld& world, const Position& position, const AutonomousVehicle* vehicle) const {
    // 1. Vehicle (@) - Highest Priority
    if (vehicle && vehicle->position().x == position.x && vehicle->position().y == position.y) return '@';

    // 2. Moving Bike (B)
    for (const auto& object : world.getObjects()) {
      if (object && object->position().x == position.x && object->position().y == position.y && dynamic_cast<const MovingBike*>(object.get())) return 'B';
    }

    // 3. Moving Car (C)
    for (const auto& object : world.getObjects()) {
      if (object && object->position().x == position.x && object->position().y == position.y && dynamic_cast<const MovingCar*>(object.get())) return 'C';
    }
    
    // 4. Traffic Lights (R/G/Y)
    for (const auto& object : world.getObjects()) {
      if (object && object->position().x == position.x && object->position().y == position.y) {
          if (const auto* light = dynamic_cast<const TrafficLight*>(object.get())) {
             std::string state = light->currentStateName();
             if (state == "RED") return 'R';
             if (state == "GREEN") return 'G';
             if (state == "YELLOW") return 'Y';
             return 'L';
          }
      }
    }
    
    // 5. Stop Sign (S)
    for (const auto& object : world.getObjects()) {
      if (object && object->position().x == position.x && object->position().y == position.y && dynamic_cast<const StopSign*>(object.get())) return 'S';
    }

    // 6. Parked Car (P)
    for (const auto& object : world.getObjects()) {
      if (object && object->position().x == position.x && object->position().y == position.y && dynamic_cast<const ParkedCar*>(object.get())) return 'P';
    }
    return '.';
}