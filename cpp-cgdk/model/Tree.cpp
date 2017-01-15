#include "Tree.h"

using namespace model;
using namespace std;

Tree::Tree()
    : LivingUnit(0, 0.0, 0.0, 0.0, 0.0, 0.0, _FACTION_UNKNOWN_, 0.0, 0, 0, vector<Status>()) { }

Tree::Tree(long long id, double x, double y, double speedX, double speedY, double angle, Faction faction, double radius,
    int life, int maxLife, const vector<Status>& statuses)
    : LivingUnit(id, x, y, speedX, speedY, angle, faction, radius, life, maxLife, statuses) { }
