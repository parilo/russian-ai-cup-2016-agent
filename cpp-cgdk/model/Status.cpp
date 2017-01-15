#include "Status.h"

using namespace model;

Status::Status()
    : id(-1), type(_STATUS_UNKNOWN_), wizardId(-1), playerId(-1), remainingDurationTicks(-1) { }

Status::Status(long long id, StatusType type, long long wizardId, long long playerId, int remainingDurationTicks)
    : id(id), type(type), wizardId(wizardId), playerId(playerId), remainingDurationTicks(remainingDurationTicks) { }

long long Status::getId() const {
    return id;
}

StatusType Status::getType() const {
    return type;
}

long long Status::getWizardId() const {
    return wizardId;
}

long long Status::getPlayerId() const {
    return playerId;
}

int Status::getRemainingDurationTicks() const {
    return remainingDurationTicks;
}
