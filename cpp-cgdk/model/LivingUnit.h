#pragma once

#ifndef _LIVING_UNIT_H_
#define _LIVING_UNIT_H_

#include <vector>

#include "CircularUnit.h"
#include "Faction.h"
#include "Status.h"

namespace model {
    class LivingUnit : public CircularUnit {
    private:
        int life;
        int maxLife;
        std::vector<Status> statuses;
    protected:
        LivingUnit(long long id, double x, double y, double speedX, double speedY, double angle, Faction faction,
                double radius, int life, int maxLife, const std::vector<Status>& statuses);
    public:
        int getLife() const;
        int getMaxLife() const;
        const std::vector<Status>& getStatuses() const;

        virtual ~LivingUnit();
    };
}

#endif
