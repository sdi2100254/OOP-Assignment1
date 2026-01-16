#pragma once
#include <string>
#include "Common.h"

// Forward declaration prevents circular dependency
class GridWorld; 

class WorldObject {
 public:
  WorldObject(std::string id, char glyph, Position position, const GridWorld* world);
  virtual ~WorldObject() = default;

  const std::string& id() const;
  char glyph() const;
  const Position& position() const;
  bool markedForRemoval() const;

  virtual void update();

 protected:
  void setPosition(const Position& position);
  void markForRemoval();
  const GridWorld* world() const;

 private:
  std::string id_;
  char glyph_;
  Position position_;
  bool marked_for_removal_{false};
  const GridWorld* world_;
};