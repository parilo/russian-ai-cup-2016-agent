#pragma once

#ifndef _PLAYER_CONTEXT_H_
#define _PLAYER_CONTEXT_H_

#include <vector>

#include "Wizard.h"
#include "World.h"

namespace model {
    class PlayerContext {
    private:
        std::vector<Wizard> wizards;
        World world;
    public:
        PlayerContext();
        PlayerContext(const std::vector<Wizard>& wizards, const World& world);

        const std::vector<Wizard>& getWizards() const;
        const World& getWorld() const;
    };
}

#endif
