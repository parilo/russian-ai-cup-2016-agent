#include "PlayerContext.h"

using namespace model;
using namespace std;

PlayerContext::PlayerContext()
    : wizards(vector<Wizard>()), world(World()) { }

PlayerContext::PlayerContext(const vector<Wizard>& wizards, const World& world)
    : wizards(wizards), world(world) { }

const vector<Wizard>& PlayerContext::getWizards() const {
    return wizards;
}

const World& PlayerContext::getWorld() const {
    return world;
}
