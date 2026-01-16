#include "WorldObject.h"
#include "GridWorld.h" 

WorldObject::WorldObject(std::string id, char glyph, Position position, const GridWorld* world)
    : id_(std::move(id)), glyph_(glyph), position_(position), world_(world) {}

const std::string& WorldObject::id() const { return id_; }
char WorldObject::glyph() const { return glyph_; }
const Position& WorldObject::position() const { return position_; }
bool WorldObject::markedForRemoval() const { return marked_for_removal_; }

void WorldObject::update() {}

void WorldObject::setPosition(const Position& position) { position_ = position; }
void WorldObject::markForRemoval() { marked_for_removal_ = true; }
const GridWorld* WorldObject::world() const { return world_; }