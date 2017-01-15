#include "LivingUnit.h"

using namespace model;
using namespace std;

LivingUnit::LivingUnit(long long id, double x, double y, double speedX, double speedY, double angle, Faction faction,
    double radius, int life, int maxLife, const vector<Status>& statuses)
    : CircularUnit(id, x, y, speedX, speedY, angle, faction, radius), life(life), maxLife(maxLife),
    statuses(statuses) { }

int LivingUnit::getLife() const {
    return life;
}

int LivingUnit::getMaxLife() const {
    return maxLife;
}

const vector<Status>& LivingUnit::getStatuses() const {
    return statuses;
}

LivingUnit::~LivingUnit() { }
