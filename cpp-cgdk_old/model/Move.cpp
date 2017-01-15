#include "Move.h"

using namespace model;
using namespace std;

Move::Move()
    : speed(0.0), strafeSpeed(0.0), turn(0.0), action(_ACTION_UNKNOWN_), castAngle(0.0), minCastDistance(0.0),
    maxCastDistance(10000.0), statusTargetId(-1), skillToLearn(_SKILL_UNKNOWN_), messages(std::vector<Message>()) { }

double Move::getSpeed() const {
    return speed;
}

void Move::setSpeed(const double speed) {
    this->speed = speed;
}

double Move::getStrafeSpeed() const {
    return strafeSpeed;
}

void Move::setStrafeSpeed(const double strafeSpeed) {
    this->strafeSpeed = strafeSpeed;
}

double Move::getTurn() const {
    return turn;
}

void Move::setTurn(const double turn) {
    this->turn = turn;
}

ActionType Move::getAction() const {
    return action;
}

void Move::setAction(const ActionType action) {
    this->action = action;
}

double Move::getCastAngle() const {
    return castAngle;
}

void Move::setCastAngle(const double castAngle) {
    this->castAngle = castAngle;
}

double Move::getMinCastDistance() const {
    return minCastDistance;
}

void Move::setMinCastDistance(const double minCastDistance) {
    this->minCastDistance = minCastDistance;
}

double Move::getMaxCastDistance() const {
    return maxCastDistance;
}

void Move::setMaxCastDistance(const double maxCastDistance) {
    this->maxCastDistance = maxCastDistance;
}

long long Move::getStatusTargetId() const {
    return statusTargetId;
}

void Move::setStatusTargetId(const long long statusTargetId) {
    this->statusTargetId = statusTargetId;
}

SkillType Move::getSkillToLearn() const {
    return skillToLearn;
}

void Move::setSkillToLearn(const SkillType skillToLearn) {
    this->skillToLearn = skillToLearn;
}

const vector<Message>& Move::getMessages() const {
    return messages;
}

void Move::setMessages(const vector<Message> messages) {
    this->messages = messages;
}
