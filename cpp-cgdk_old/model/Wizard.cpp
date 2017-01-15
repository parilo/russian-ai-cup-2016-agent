#include "Wizard.h"

using namespace model;
using namespace std;

Wizard::Wizard()
    : LivingUnit(0, 0.0, 0.0, 0.0, 0.0, 0.0, _FACTION_UNKNOWN_, 0.0, 0, 0, vector<Status>()), ownerPlayerId(-1),
    me(false), mana(-1), maxMana(-1), visionRange(-1.0), castRange(-1.0), xp(-1), level(-1),
    skills(vector<SkillType>()), remainingActionCooldownTicks(-1), remainingCooldownTicksByAction(vector<int>()),
    master(false), messages(vector<Message>()) { }

Wizard::Wizard(long long id, double x, double y, double speedX, double speedY, double angle, Faction faction,
    double radius, int life, int maxLife, const vector<Status>& statuses, long long ownerPlayerId, bool me, int mana,
    int maxMana, double visionRange, double castRange, int xp, int level, const vector<SkillType>& skills,
    int remainingActionCooldownTicks, const vector<int>& remainingCooldownTicksByAction, bool master,
    const vector<Message>& messages)
    : LivingUnit(id, x, y, speedX, speedY, angle, faction, radius, life, maxLife, statuses),
    ownerPlayerId(ownerPlayerId), me(me), mana(mana), maxMana(maxMana), visionRange(visionRange), castRange(castRange),
    xp(xp), level(level), skills(skills), remainingActionCooldownTicks(remainingActionCooldownTicks),
    remainingCooldownTicksByAction(remainingCooldownTicksByAction), master(master), messages(messages) { }

long long Wizard::getOwnerPlayerId() const {
    return ownerPlayerId;
}

bool Wizard::isMe() const {
    return me;
}

int Wizard::getMana() const {
    return mana;
}

int Wizard::getMaxMana() const {
    return maxMana;
}

double Wizard::getVisionRange() const {
    return visionRange;
}

double Wizard::getCastRange() const {
    return castRange;
}

int Wizard::getXp() const {
    return xp;
}

int Wizard::getLevel() const {
    return level;
}

const vector<SkillType>& Wizard::getSkills() const {
    return skills;
}

int Wizard::getRemainingActionCooldownTicks() const {
    return remainingActionCooldownTicks;
}

const vector<int>& Wizard::getRemainingCooldownTicksByAction() const {
    return remainingCooldownTicksByAction;
}

bool Wizard::isMaster() const {
    return master;
}

const vector<Message>& Wizard::getMessages() const {
    return messages;
}
