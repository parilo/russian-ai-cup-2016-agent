#pragma once

#ifndef _BONUS_H_
#define _BONUS_H_

#include "BonusType.h"
#include "CircularUnit.h"
#include "Faction.h"

namespace model {
    class Bonus : public CircularUnit {
    private:
        BonusType type;
    public:
        Bonus();
        Bonus(long long id, double x, double y, double speedX, double speedY, double angle, Faction faction,
                double radius, BonusType type);

        BonusType getType() const;
    };
}

#endif
