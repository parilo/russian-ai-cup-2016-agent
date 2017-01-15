#include "Building.h"

using namespace model;
using namespace std;

Building::Building()
    : LivingUnit(0, 0.0, 0.0, 0.0, 0.0, 0.0, _FACTION_UNKNOWN_, 0.0, 0, 0, vector<Status>()), type(_BUILDING_UNKNOWN_),
    visionRange(-1.0), attackRange(-1.0), damage(-1), cooldownTicks(-1), remainingActionCooldownTicks(-1) { }

Building::Building(long long id, double x, double y, double speedX, double speedY, double angle, Faction faction,
    double radius, int life, int maxLife, const vector<Status>& statuses, BuildingType type, double visionRange,
    double attackRange, int damage, int cooldownTicks, int remainingActionCooldownTicks)
    : LivingUnit(id, x, y, speedX, speedY, angle, faction, radius, life, maxLife, statuses), type(type),
    visionRange(visionRange), attackRange(attackRange), damage(damage), cooldownTicks(cooldownTicks),
    remainingActionCooldownTicks(remainingActionCooldownTicks) { }

BuildingType Building::getType() const {
    return type;
}

double Building::getVisionRange() const {
    return visionRange;
}

double Building::getAttackRange() const {
    return attackRange;
}

int Building::getDamage() const {
    return damage;
}

int Building::getCooldownTicks() const {
    return cooldownTicks;
}

int Building::getRemainingActionCooldownTicks() const {
    return remainingActionCooldownTicks;
}
