#pragma once

#ifndef _TREE_H_
#define _TREE_H_

#include "Faction.h"
#include "LivingUnit.h"
#include "Status.h"

namespace model {
    class Tree : public LivingUnit {
    private:
    public:
        Tree();
        Tree(long long id, double x, double y, double speedX, double speedY, double angle, Faction faction,
                double radius, int life, int maxLife, const std::vector<Status>& statuses);

    };
}

#endif
