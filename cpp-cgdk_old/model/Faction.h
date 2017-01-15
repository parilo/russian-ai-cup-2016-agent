#pragma once

#ifndef _FACTION_H_
#define _FACTION_H_

namespace model {
    enum Faction {
        _FACTION_UNKNOWN_ = -1,
        FACTION_ACADEMY = 0,
        FACTION_RENEGADES = 1,
        FACTION_NEUTRAL = 2,
        FACTION_OTHER = 3,
        _FACTION_COUNT_ = 4
    };
}

#endif
