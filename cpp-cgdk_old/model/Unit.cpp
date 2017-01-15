#include "Unit.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include <cmath>

using namespace model;

Unit::Unit(long long id, double x, double y, double speedX, double speedY, double angle, Faction faction)
    : id(id), x(x), y(y), speedX(speedX), speedY(speedY), angle(angle), faction(faction) { }

long long Unit::getId() const {
    return id;
}

double Unit::getX() const {
    return x;
}

double Unit::getY() const {
    return y;
}

double Unit::getSpeedX() const {
    return speedX;
}

double Unit::getSpeedY() const {
    return speedY;
}

double Unit::getAngle() const {
    return angle;
}

Faction Unit::getFaction() const {
    return faction;
}

double Unit::getAngleTo(double x, double y) const {
    double absoluteAngleTo = atan2(y - this->y, x - this->x);
    double relativeAngleTo = absoluteAngleTo - this->angle;

    while (relativeAngleTo > PI) {
        relativeAngleTo -= 2.0 * PI;
    }

    while (relativeAngleTo < -PI) {
        relativeAngleTo += 2.0 * PI;
    }

    return relativeAngleTo;
}

double Unit::getAngleTo(const Unit& unit) const {
    return this->getAngleTo(unit.x, unit.y);
}

double Unit::getDistanceTo(double x, double y) const {
    double xRange = x - this->x;
    double yRange = y - this->y;
    return sqrt(xRange * xRange + yRange * yRange);
}

double Unit::getDistanceTo(const Unit& unit) const {
    return this->getDistanceTo(unit.x, unit.y);
}

Unit::~Unit() { }
