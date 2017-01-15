#include "Projectile.h"

using namespace model;

Projectile::Projectile()
    : CircularUnit(0, 0.0, 0.0, 0.0, 0.0, 0.0, _FACTION_UNKNOWN_, 0.0), type(_PROJECTILE_UNKNOWN_), ownerUnitId(-1),
    ownerPlayerId(-1) { }

Projectile::Projectile(long long id, double x, double y, double speedX, double speedY, double angle, Faction faction,
    double radius, ProjectileType type, long long ownerUnitId, long long ownerPlayerId)
    : CircularUnit(id, x, y, speedX, speedY, angle, faction, radius), type(type), ownerUnitId(ownerUnitId),
    ownerPlayerId(ownerPlayerId) { }

ProjectileType Projectile::getType() const {
    return type;
}

long long Projectile::getOwnerUnitId() const {
    return ownerUnitId;
}

long long Projectile::getOwnerPlayerId() const {
    return ownerPlayerId;
}
