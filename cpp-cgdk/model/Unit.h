#pragma once

#ifndef _UNIT_H_
#define _UNIT_H_

#include "Faction.h"

namespace model {
    class Unit {
    private:
        long long id;
        double x;
        double y;
        double speedX;
        double speedY;
        double angle;
        Faction faction;
    protected:
        Unit(long long id, double x, double y, double speedX, double speedY, double angle, Faction faction);
    public:
        long long getId() const;
        double getX() const;
        double getY() const;
        double getSpeedX() const;
        double getSpeedY() const;
        double getAngle() const;
        Faction getFaction() const;

        double getAngleTo(double x, double y) const;
        double getAngleTo(const Unit& unit) const;
        double getDistanceTo(double x, double y) const;
        double getDistanceTo(const Unit& unit) const;

        virtual ~Unit();
    };
}

#endif
