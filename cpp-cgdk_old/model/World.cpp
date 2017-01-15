#include "World.h"

using namespace model;
using namespace std;

World::World()
    : tickIndex(-1), tickCount(-1), width(-1.0), height(-1.0), players(vector<Player>()), wizards(vector<Wizard>()),
    minions(vector<Minion>()), projectiles(vector<Projectile>()), bonuses(vector<Bonus>()),
    buildings(vector<Building>()), trees(vector<Tree>()) { }

World::World(int tickIndex, int tickCount, double width, double height, const vector<Player>& players,
    const vector<Wizard>& wizards, const vector<Minion>& minions, const vector<Projectile>& projectiles,
    const vector<Bonus>& bonuses, const vector<Building>& buildings, const vector<Tree>& trees)
    : tickIndex(tickIndex), tickCount(tickCount), width(width), height(height), players(players), wizards(wizards),
    minions(minions), projectiles(projectiles), bonuses(bonuses), buildings(buildings), trees(trees) { }

int World::getTickIndex() const {
    return tickIndex;
}

int World::getTickCount() const {
    return tickCount;
}

double World::getWidth() const {
    return width;
}

double World::getHeight() const {
    return height;
}

const vector<Player>& World::getPlayers() const {
    return players;
}

const vector<Wizard>& World::getWizards() const {
    return wizards;
}

const vector<Minion>& World::getMinions() const {
    return minions;
}

const vector<Projectile>& World::getProjectiles() const {
    return projectiles;
}

const vector<Bonus>& World::getBonuses() const {
    return bonuses;
}

const vector<Building>& World::getBuildings() const {
    return buildings;
}

const vector<Tree>& World::getTrees() const {
    return trees;
}

Player World::getMyPlayer() const {
    for (int playerIndex = players.size() - 1; playerIndex >= 0; --playerIndex) {
        Player player = players[playerIndex];
        if (player.isMe()) {
            return player;
        }
    }

    throw;
}
