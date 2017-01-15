#pragma once

#ifndef _BUILDING_H_
#define _BUILDING_H_

#include "BuildingType.h"
#include "Faction.h"
#include "LivingUnit.h"
#include "Status.h"

namespace model {
    class Building : public LivingUnit {
    private:
        BuildingType type;
        double visionRange;
        double attackRange;
        int damage;
        int cooldownTicks;
        int remainingActionCooldownTicks;
    public:
        Building();
        Building(long long id, double x, double y, double speedX, double speedY, double angle, Faction faction,
                double radius, int life, int maxLife, const std::vector<Status>& statuses, BuildingType type,
                double visionRange, double attackRange, int damage, int cooldownTicks,
                int remainingActionCooldownTicks);

        BuildingType getType() const;
        double getVisionRange() const;
        double getAttackRange() const;
        int getDamage() const;
        int getCooldownTicks() const;
        int getRemainingActionCooldownTicks() const;
    };
}

#endif
