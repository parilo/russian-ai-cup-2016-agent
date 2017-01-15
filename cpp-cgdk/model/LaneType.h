#pragma once

#ifndef _LANE_TYPE_H_
#define _LANE_TYPE_H_

namespace model {
    enum LaneType {
        _LANE_UNKNOWN_ = -1,
        LANE_TOP = 0,
        LANE_MIDDLE = 1,
        LANE_BOTTOM = 2,
        _LANE_COUNT_ = 3
    };
}

#endif
