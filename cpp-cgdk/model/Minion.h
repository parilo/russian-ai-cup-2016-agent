#pragma once

#ifndef _MINION_H_
#define _MINION_H_

#include "Faction.h"
#include "LivingUnit.h"
#include "MinionType.h"
#include "Status.h"

namespace model {
    class Minion : public LivingUnit {
    private:
        MinionType type;
        double visionRange;
        int damage;
        int cooldownTicks;
        int remainingActionCooldownTicks;
    public:
        Minion();
        Minion(long long id, double x, double y, double speedX, double speedY, double angle, Faction faction,
                double radius, int life, int maxLife, const std::vector<Status>& statuses, MinionType type,
                double visionRange, int damage, int cooldownTicks, int remainingActionCooldownTicks);

        MinionType getType() const;
        double getVisionRange() const;
        int getDamage() const;
        int getCooldownTicks() const;
        int getRemainingActionCooldownTicks() const;
    };
}

#endif
