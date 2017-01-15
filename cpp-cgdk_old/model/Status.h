#pragma once

#ifndef _STATUS_H_
#define _STATUS_H_

#include "StatusType.h"

namespace model {
    class Status {
    private:
        long long id;
        StatusType type;
        long long wizardId;
        long long playerId;
        int remainingDurationTicks;
    public:
        Status();
        Status(long long id, StatusType type, long long wizardId, long long playerId, int remainingDurationTicks);

        long long getId() const;
        StatusType getType() const;
        long long getWizardId() const;
        long long getPlayerId() const;
        int getRemainingDurationTicks() const;
    };
}

#endif
