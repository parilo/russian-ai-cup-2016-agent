#pragma once

#ifndef _MESSAGE_H_
#define _MESSAGE_H_

#include <vector>

#include "LaneType.h"
#include "SkillType.h"

namespace model {
    class Message {
    private:
        LaneType lane;
        SkillType skillToLearn;
        std::vector<signed char> rawMessage;
    public:
        Message();
        Message(LaneType lane, SkillType skillToLearn, const std::vector<signed char>& rawMessage);

        LaneType getLane() const;
        SkillType getSkillToLearn() const;
        const std::vector<signed char>& getRawMessage() const;
    };
}

#endif
