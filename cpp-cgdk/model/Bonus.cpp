#include "Bonus.h"

using namespace model;

Bonus::Bonus()
    : CircularUnit(0, 0.0, 0.0, 0.0, 0.0, 0.0, _FACTION_UNKNOWN_, 0.0), type(_BONUS_UNKNOWN_) { }

Bonus::Bonus(long long id, double x, double y, double speedX, double speedY, double angle, Faction faction,
    double radius, BonusType type)
    : CircularUnit(id, x, y, speedX, speedY, angle, faction, radius), type(type) { }

BonusType Bonus::getType() const {
    return type;
}
