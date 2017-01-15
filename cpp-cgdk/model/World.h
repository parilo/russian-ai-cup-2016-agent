#pragma once

#ifndef _WORLD_H_
#define _WORLD_H_

#include <vector>

#include "Bonus.h"
#include "Building.h"
#include "Minion.h"
#include "Player.h"
#include "Projectile.h"
#include "Tree.h"
#include "Wizard.h"

namespace model {
    class World {
    private:
        int tickIndex;
        int tickCount;
        double width;
        double height;
        std::vector<Player> players;
        std::vector<Wizard> wizards;
        std::vector<Minion> minions;
        std::vector<Projectile> projectiles;
        std::vector<Bonus> bonuses;
        std::vector<Building> buildings;
        std::vector<Tree> trees;
    public:
        World();
        World(int tickIndex, int tickCount, double width, double height, const std::vector<Player>& players,
                const std::vector<Wizard>& wizards, const std::vector<Minion>& minions,
                const std::vector<Projectile>& projectiles, const std::vector<Bonus>& bonuses,
                const std::vector<Building>& buildings, const std::vector<Tree>& trees);

        int getTickIndex() const;
        int getTickCount() const;
        double getWidth() const;
        double getHeight() const;
        const std::vector<Player>& getPlayers() const;
        const std::vector<Wizard>& getWizards() const;
        const std::vector<Minion>& getMinions() const;
        const std::vector<Projectile>& getProjectiles() const;
        const std::vector<Bonus>& getBonuses() const;
        const std::vector<Building>& getBuildings() const;
        const std::vector<Tree>& getTrees() const;

        Player getMyPlayer() const;
    };
}

#endif
