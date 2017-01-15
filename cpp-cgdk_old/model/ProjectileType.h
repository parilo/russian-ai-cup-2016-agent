#pragma once

#ifndef _PROJECTILE_TYPE_H_
#define _PROJECTILE_TYPE_H_

namespace model {
    enum ProjectileType {
        _PROJECTILE_UNKNOWN_ = -1,
        PROJECTILE_MAGIC_MISSILE = 0,
        PROJECTILE_FROST_BOLT = 1,
        PROJECTILE_FIREBALL = 2,
        PROJECTILE_DART = 3,
        _PROJECTILE_COUNT_ = 4
    };
}

#endif
