#pragma once

#ifndef _ACTION_TYPE_H_
#define _ACTION_TYPE_H_

namespace model {
    enum ActionType {
        _ACTION_UNKNOWN_ = -1,
        ACTION_NONE = 0,
        ACTION_STAFF = 1,
        ACTION_MAGIC_MISSILE = 2,
        ACTION_FROST_BOLT = 3,
        ACTION_FIREBALL = 4,
        ACTION_HASTE = 5,
        ACTION_SHIELD = 6,
        _ACTION_COUNT_ = 7
    };
}

#endif
