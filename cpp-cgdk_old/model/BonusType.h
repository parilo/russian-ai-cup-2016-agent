#pragma once

#ifndef _BONUS_TYPE_H_
#define _BONUS_TYPE_H_

namespace model {
    enum BonusType {
        _BONUS_UNKNOWN_ = -1,
        BONUS_EMPOWER = 0,
        BONUS_HASTE = 1,
        BONUS_SHIELD = 2,
        _BONUS_COUNT_ = 3
    };
}

#endif
