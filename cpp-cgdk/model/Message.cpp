#include "Message.h"

using namespace model;
using namespace std;

Message::Message()
    : lane(_LANE_UNKNOWN_), skillToLearn(_SKILL_UNKNOWN_), rawMessage(vector<signed char>()) { }

Message::Message(LaneType lane, SkillType skillToLearn, const vector<signed char>& rawMessage)
    : lane(lane), skillToLearn(skillToLearn), rawMessage(rawMessage) { }

LaneType Message::getLane() const {
    return lane;
}

SkillType Message::getSkillToLearn() const {
    return skillToLearn;
}

const vector<signed char>& Message::getRawMessage() const {
    return rawMessage;
}
