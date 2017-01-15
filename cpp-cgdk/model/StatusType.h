#pragma once

#ifndef _STATUS_TYPE_H_
#define _STATUS_TYPE_H_

namespace model {
    enum StatusType {
        _STATUS_UNKNOWN_ = -1,
        STATUS_BURNING = 0,
        STATUS_EMPOWERED = 1,
        STATUS_FROZEN = 2,
        STATUS_HASTENED = 3,
        STATUS_SHIELDED = 4,
        _STATUS_COUNT_ = 5
    };
}

#endif
