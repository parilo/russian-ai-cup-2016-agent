#pragma once

#ifndef _WIZARD_H_
#define _WIZARD_H_

#include <vector>

#include "Faction.h"
#include "LivingUnit.h"
#include "Message.h"
#include "SkillType.h"
#include "Status.h"

namespace model {
    class Wizard : public LivingUnit {
    private:
        long long ownerPlayerId;
        bool me;
        int mana;
        int maxMana;
        double visionRange;
        double castRange;
        int xp;
        int level;
        std::vector<SkillType> skills;
        int remainingActionCooldownTicks;
        std::vector<int> remainingCooldownTicksByAction;
        bool master;
        std::vector<Message> messages;
    public:
        Wizard();
        Wizard(long long id, double x, double y, double speedX, double speedY, double angle, Faction faction,
                double radius, int life, int maxLife, const std::vector<Status>& statuses, long long ownerPlayerId,
                bool me, int mana, int maxMana, double visionRange, double castRange, int xp, int level,
                const std::vector<SkillType>& skills, int remainingActionCooldownTicks,
                const std::vector<int>& remainingCooldownTicksByAction, bool master,
                const std::vector<Message>& messages);

        long long getOwnerPlayerId() const;
        bool isMe() const;
        int getMana() const;
        int getMaxMana() const;
        double getVisionRange() const;
        double getCastRange() const;
        int getXp() const;
        int getLevel() const;
        const std::vector<SkillType>& getSkills() const;
        int getRemainingActionCooldownTicks() const;
        const std::vector<int>& getRemainingCooldownTicksByAction() const;
        bool isMaster() const;
        const std::vector<Message>& getMessages() const;
    };
}

#endif
