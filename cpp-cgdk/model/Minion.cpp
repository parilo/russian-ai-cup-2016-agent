#include "Minion.h"

using namespace model;
using namespace std;

Minion::Minion()
    : LivingUnit(0, 0.0, 0.0, 0.0, 0.0, 0.0, _FACTION_UNKNOWN_, 0.0, 0, 0, vector<Status>()), type(_MINION_UNKNOWN_),
    visionRange(-1.0), damage(-1), cooldownTicks(-1), remainingActionCooldownTicks(-1) { }

Minion::Minion(long long id, double x, double y, double speedX, double speedY, double angle, Faction faction,
    double radius, int life, int maxLife, const vector<Status>& statuses, MinionType type, double visionRange,
    int damage, int cooldownTicks, int remainingActionCooldownTicks)
    : LivingUnit(id, x, y, speedX, speedY, angle, faction, radius, life, maxLife, statuses), type(type),
    visionRange(visionRange), damage(damage), cooldownTicks(cooldownTicks),
    remainingActionCooldownTicks(remainingActionCooldownTicks) { }

MinionType Minion::getType() const {
    return type;
}

double Minion::getVisionRange() const {
    return visionRange;
}

int Minion::getDamage() const {
    return damage;
}

int Minion::getCooldownTicks() const {
    return cooldownTicks;
}

int Minion::getRemainingActionCooldownTicks() const {
    return remainingActionCooldownTicks;
}
