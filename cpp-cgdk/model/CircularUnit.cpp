#include "CircularUnit.h"

using namespace model;

CircularUnit::CircularUnit(long long id, double x, double y, double speedX, double speedY, double angle,
    Faction faction, double radius)
    : Unit(id, x, y, speedX, speedY, angle, faction), radius(radius) { }

double CircularUnit::getRadius() const {
    return radius;
}

CircularUnit::~CircularUnit() { }
