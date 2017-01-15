#pragma once

#ifndef _MOVE_H_
#define _MOVE_H_

#include <vector>

#include "ActionType.h"
#include "Message.h"
#include "SkillType.h"

namespace model {
    class Move {
    private:
        double speed;
        double strafeSpeed;
        double turn;
        ActionType action;
        double castAngle;
        double minCastDistance;
        double maxCastDistance;
        long long statusTargetId;
        SkillType skillToLearn;
        std::vector<Message> messages;
    public:
        Move();

        double getSpeed() const;
        void setSpeed(const double speed);
        double getStrafeSpeed() const;
        void setStrafeSpeed(const double strafeSpeed);
        double getTurn() const;
        void setTurn(const double turn);
        ActionType getAction() const;
        void setAction(const ActionType action);
        double getCastAngle() const;
        void setCastAngle(const double castAngle);
        double getMinCastDistance() const;
        void setMinCastDistance(const double minCastDistance);
        double getMaxCastDistance() const;
        void setMaxCastDistance(const double maxCastDistance);
        long long getStatusTargetId() const;
        void setStatusTargetId(const long long statusTargetId);
        SkillType getSkillToLearn() const;
        void setSkillToLearn(const SkillType skillToLearn);
        const std::vector<Message>& getMessages() const;
        void setMessages(const std::vector<Message> messages);
    };
}

#endif
