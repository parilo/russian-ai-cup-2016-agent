#pragma once

#ifndef _PROJECTILE_H_
#define _PROJECTILE_H_

#include "CircularUnit.h"
#include "Faction.h"
#include "ProjectileType.h"

namespace model {
    class Projectile : public CircularUnit {
    private:
        ProjectileType type;
        long long ownerUnitId;
        long long ownerPlayerId;
    public:
        Projectile();
        Projectile(long long id, double x, double y, double speedX, double speedY, double angle, Faction faction,
                double radius, ProjectileType type, long long ownerUnitId, long long ownerPlayerId);

        ProjectileType getType() const;
        long long getOwnerUnitId() const;
        long long getOwnerPlayerId() const;
    };
}

#endif
