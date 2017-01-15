#include "MyStrategy.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <condition_variable>
#include <map>

#include "SimpleClient.hpp"
#include "SimpleStrategy.h"

using namespace model;
using namespace std;
using namespace WizardTrainer;

std::shared_ptr<SimpleClient> cl;

const int stateSize = 1184;
const int actionSize = 6;

vector<double> prevState;
vector<double> nextState;
vector<double> action;
double reward;
int xp;
int life;
int mainBaseLife;
bool reseted = true;

std::condition_variable cvAction;
std::mutex mtxAction;
bool actionReady = false;

SimpleStrategy simpleStrategy; 

double getRandom (double LO, double HI) {
    return LO + std::rand() /(RAND_MAX/(HI-LO));
}

void fillPos (const Unit& me, const Unit& other, vector<double>& state, int& index) {
    double r = other.getDistanceTo (me);
    state [index++] = r * 0.01;
    state [index++] = 10 * (other.getX () - me.getX ()) / r;
    state [index++] = 10 * (other.getY () - me.getY ()) / r;
}

void fillUnit (const Unit& me, const Unit& other, vector<double>& state, int& index) {
    state [index++] = other.getAngle () * 10 / PI;
    state [index++] = other.getFaction () == me.getFaction ()?5:(other.getFaction () == FACTION_NEUTRAL?10:0);
    state [index++] = 2 * (other.getSpeedX () - me.getSpeedX ());
    state [index++] = 2 * (other.getSpeedY () - me.getSpeedY ());

    state [index++] = other.getDistanceTo (me) * 0.01;
    state [index++] = (other.getX () - me.getX ()) * 0.01;
    state [index++] = (other.getY () - me.getY ()) * 0.01;
}

void fillLivingUnit (const LivingUnit& u, vector<double>& state, int& index) {
    state [index++] = 0.05 * u.getLife ();
    int statusesRealCount = 0;
    for(auto& s : u.getStatuses ()) {
        if (statusesRealCount == 8) break;
        state [index++] = s.getRemainingDurationTicks ();
        state [index++] = 2 * s.getType ();
            //BURNING
            //EMPOWERED
            //FROZEN
            //HASTENED
            //SHIELDED
        statusesRealCount++;
    }
    if (statusesRealCount < 8) {
        index += (8-statusesRealCount) * 2;
    }
    //8 statuses
}


int argmax (std::vector<double>::iterator begin, std::vector<double>::iterator end) {
    double maxval = *begin;
    int maxi = 0;
    for (int i=0; i<end-begin; i++) {
        double val = *(begin+i);
        if (val > maxval) {
            maxval = val;
            maxi = i;
        }
    }
    return maxi;
}

double prevR = 0;

void fillMove (Move& move) {
/*
    move.setAction (static_cast<model::ActionType>(argmax (action.begin(), action.begin()+3)));
//        //NONE
//        //STAFF
//        //MAGIC_MISSILE
//        //FROST_BOLT
//        //FIREBALL
//        //HASTE
//        //SHIELD
//    
    move.setCastAngle (action [7]);
    move.setMaxCastDistance (10000); //action [8]);
    move.setMinCastDistance (0); //action [9]);
//    move.setSkillToLearn (argmax (action.begin(), action.begin()+7));
    move.setSpeed (action [10]);
    move.setStrafeSpeed (action [11]);
    int needToTurn = argmax (action.begin()+8, action.begin()+10);
    if (needToTurn == 0) {
    } else {
	move.setTurn (action [12]);
    }
*/

/*
cout 
    << action [0] << " "
    << action [0] << " "
    << action [0] << " "
    << endl;
*/

    move.setSpeed (0.005 * action [0]); //0.002
    move.setStrafeSpeed (0.005 * action [1]);

    if (!simpleStrategy.isTurnSet ()) {
	move.setTurn (0.005 * action [2]);
    }

    if (!simpleStrategy.isActionSet ()) {
	move.setAction (static_cast<model::ActionType>(argmax (action.begin()+3, action.begin()+6)));
    }

}

double targetX;
double targetY;

void MyStrategy::move(const Wizard& self, const World& world, const Game& game, Move& move) {

//    simpleStrategy.move (self, world, game, move);
//return;
/*    if (world.getTickIndex () % 10 != 0) {
	fillMove (move);
	return;
    }
*/

    if (reseted) {

            double mapSize = game.getMapSize();

            switch ((int) self.getId()) {
                case 1:
                case 2:
                case 6:
                case 7:
                    //LaneType.TOP;
		    targetX = 200;
		    targetY = 200;
                    break;
                case 3:
                case 8:
                    //LaneType.MIDDLE;
		    targetX = mapSize / 2;
		    targetY = mapSize / 2;
                    break;
                case 4:
                case 5:
                case 9:
                case 10:
                    //LaneType.BOTTOM;
		    targetX = mapSize - 200;
		    targetY = mapSize - 200;
                    break;
                default:
                    break;
            }
    }

    Faction myFaction = self.getFaction ();
    int myWIndex;
    auto ws1 = world.getWizards ();
    for (int i=0; i<ws1.size(); i++) {
        if (ws1[i].isMe ()) {
            myWIndex = i;
        }
    }
    const Wizard& me = ws1 [myWIndex];

/*
    int mainBaseNewLife = mainBaseLife;
    for (auto& b : world.getBuildings ()) {
	if (b.getType () == BUILDING_FACTION_BASE)
	if (b.getFaction () == myFaction) {
	    mainBaseNewLife = b.getLife ();
	}
    }
*/
    
    std::fill(nextState.begin(), nextState.end(), 0);
    
    int si = 0;

    for (auto& b : world.getBonuses ()) {
        nextState [si++] = (int) b.getType ();
            //EMPOWER
            //HASTE
            //SHIELD
        //1
        
        //circular unit
        //unit
        fillPos (me, b, nextState, si);
        //3
    }
    //2 x 4

//if (si >= 1083) cout << "--- si1: " << si << endl;
    si = 8;

    std::vector<Building> bs = world.getBuildings ();
    std::sort (bs.begin (), bs.end (), [&me](const Building& o1, const Building& o2){
	if (o1.getFaction () == o2.getFaction ()) {
	    return o1.getId () < o2.getId ();
	} else {
	    if (o1.getFaction () == me.getFaction ()) {
		return true;
	    } else {
		return false;
	    }
	}
    });

    int bi = 0;
    int mainBaseNewLife = mainBaseLife;
//cout << "--- friend buildings" << endl;
    for (auto& b : bs) {

	if (b.getType () == BUILDING_FACTION_BASE)
	if (b.getFaction () == myFaction) {
	    mainBaseNewLife = b.getLife ();
	}

	if (b.getFaction () != me.getFaction ()) {
	    break;
	}

        if (bi == 7) break;
//        double ar = b.getAttackRange ();
//        int ct = b.getCooldownTicks ();
//        int d = b.getDamage ();
        nextState [si++] = 0.1 * b.getRemainingActionCooldownTicks ();
        nextState [si++] = 5 * b.getType ();
//cout << " id: " << b.getId () << " t: " << b.getType () << " f: " << b.getFaction () << " mf: " << me.getFaction () << endl;
            //GUARDIAN_TOWER
            //FACTION_BASE
//        double r = b.getVisionRange ();
        
        //living unit
        nextState [si++] = 0.05 * b.getLife ();
//        int ml = b.getMaxLife ();
//        cout << "---" << endl;
//        for(auto& s : b.getStatuses ()) {
//            int rt = s.getRemainingDurationTicks ();
//            int st = s.getType ();
//                //BURNING
//                //EMPOWERED
//                //FROZEN
//                //HASTENED
//                //SHIELDED
//            cout << s.getType () << " " << rt << endl;
//        }
        
        //circular unit
//        double radius = b.getRadius ();
        
        //unit
//        double a = b.getAngle ();
//        double dist = b.getDistanceTo (unit);
        nextState [si++] = b.getFaction () == myFaction?5:0;
//        double sx = b.getSpeedX ();
//        double sy = b.getSpeedY ();
//        double x = b.getX ();
//        double y = b.getY ();
        fillPos (me, b, nextState, si);
        bi++;
    }

    si = 8+7*7;

    std::sort (bs.begin (), bs.end (), [&me](const Building& o1, const Building& o2){
	if (o1.getFaction () == o2.getFaction ()) {
	    return o1.getId () < o2.getId ();
	} else {
	    if (o1.getFaction () == me.getFaction ()) {
		return false;
	    } else {
		return true;
	    }
	}
    });

    bi = 0;
//cout << "--- enemy buildings" << endl;
    for (auto& b : bs) {

	if (b.getFaction () == me.getFaction ()) {
	    break;
	}

        if (bi == 7) break;
//        double ar = b.getAttackRange ();
//        int ct = b.getCooldownTicks ();
//        int d = b.getDamage ();
        nextState [si++] = 0.1 * b.getRemainingActionCooldownTicks ();
        nextState [si++] = 5 * b.getType ();
//cout << " id: " << b.getId () << " t: " << b.getType () << " f: " << b.getFaction () << " mf: " << me.getFaction () << endl;
            //GUARDIAN_TOWER
            //FACTION_BASE
//        double r = b.getVisionRange ();
        
        //living unit
        nextState [si++] = 0.05 * b.getLife ();
//        int ml = b.getMaxLife ();
//        cout << "---" << endl;
//        for(auto& s : b.getStatuses ()) {
//            int rt = s.getRemainingDurationTicks ();
//            int st = s.getType ();
//                //BURNING
//                //EMPOWERED
//                //FROZEN
//                //HASTENED
//                //SHIELDED
//            cout << s.getType () << " " << rt << endl;
//        }
        
        //circular unit
//        double radius = b.getRadius ();
        
        //unit
//        double a = b.getAngle ();
//        double dist = b.getDistanceTo (unit);
        nextState [si++] = b.getFaction () == myFaction?5:0;
//        double sx = b.getSpeedX ();
//        double sy = b.getSpeedY ();
//        double x = b.getX ();
//        double y = b.getY ();
        fillPos (me, b, nextState, si);
        bi++;
    }

    //14 x 7 = 98
    
//if (si >= 1083) cout << "--- si2: " << si << endl;
    si = 98 + 8;

    std::vector<Minion> ms;
    for ( auto& m : world.getMinions ()) {
	if (m.getFaction () != FACTION_NEUTRAL) {
	    ms.push_back (m);
	}
    }
    std::sort (ms.begin (), ms.end (), [&me](const Minion& o1, const Minion& o2){
	if (o1.getFaction () == o2.getFaction ()) {
	    return o1.getId () < o2.getId ();
	} else {
	    if (o1.getFaction () == me.getFaction ()) {
		return true;
	    } else {
		return false;
	    }
	}
    });

    int mi = 0;
//cout << "--- friend minions" << endl;
    for (auto& m : ms) {

	if (m.getFaction () != me.getFaction ()) {
	    break;
	}

        if (mi == 30) break;
//        int ct = m.getCooldownTicks ();
//        int d = m.getDamage ();
        nextState [si++] = 0.1 * m.getRemainingActionCooldownTicks ();
        nextState [si++] = 5 * m.getType ();
//cout << " id: " << m.getId () << " t: " << m.getType () << " f: " << m.getFaction () << " mf: " << me.getFaction () << endl;
            //ORC_WOODCUTTER
            //FETISH_BLOWDART
//        double vr = m.getVisionRange ();
        
        //living unit
        nextState [si++] = 0.05 * m.getLife ();
        //unit
        fillUnit (me, m, nextState, si);
        mi++;
    }


    si = 98 + 8 + 300;
    std::sort (ms.begin (), ms.end (), [&me](const Minion& o1, const Minion& o2){
	if (o1.getFaction () == o2.getFaction ()) {
	    return o1.getId () < o2.getId ();
	} else {
	    if (o1.getFaction () == me.getFaction ()) {
		return false;
	    } else {
		return true;
	    }
	}
    });

    mi = 0;
//cout << "--- enemy minions" << endl;
    for (auto& m : ms) {

	if (m.getFaction () == me.getFaction ()) {
	    break;
	}

        if (mi == 30) break;
//        int ct = m.getCooldownTicks ();
//        int d = m.getDamage ();
        nextState [si++] = 0.1 * m.getRemainingActionCooldownTicks ();
        nextState [si++] = 5 * m.getType ();
//cout << " id: " << m.getId () << " t: " << m.getType () << " f: " << m.getFaction () << " mf: " << me.getFaction () << endl;
            //ORC_WOODCUTTER
            //FETISH_BLOWDART
//        double vr = m.getVisionRange ();
        
        //living unit
        nextState [si++] = 0.05 * m.getLife ();
        //unit
        fillUnit (me, m, nextState, si);
        mi++;
    }

    //60 x 10 = 600

//    int faction = (int) world.getMyPlayer ().getFaction ();
//        //ACADEMY
//        //RENEGADES
//        //NEUTRAL
//        //OTHER
//    
//    world.getPlayers ();
    
//if (si >= 1083) cout << "--- si3: " << si << endl;
    si = 8+98+600;
    int pi = 0;
    for (auto& p : world.getProjectiles ()) {
        if (pi == 8) break;
//        nextState [si++] = p.getOwnerPlayerId () == me.getOwnerPlayerId ()?5:0;
        
        //Возвращает идентификатор игрока, юнит которого создал данный снаряд или -1
//        long oid = p.getOwnerUnitId ();
        
        //Возвращает идентификатор юнита, создавшего данный снаряд.
        nextState [si++] = 3 * p.getType ();
            //MAGIC_MISSILE
            //FROST_BOLT
            //FIREBALL
            //DART
        
        fillUnit (me, p, nextState, si);
        pi++;
    }
    //8 x 8 = 64
    

//if (si >= 1083) cout << "--- si4: " << si << endl;
    si = 8+98+600+64;
    std::vector<Tree> ts = world.getTrees ();
    std::sort (ts.begin (), ts.end (), [&me](const Tree& t1, const Tree& t2){
        return t1.getDistanceTo (me) < t2.getDistanceTo (me);
    });
    int ti = 0;
    for (auto& t : ts) {
        if (ti == 16) break;
        //living unit
        nextState [si++] = 0.05 * t.getLife ();
        //unit
        fillPos (me, t, nextState, si);
        ti++;
    }
    //16 x 4 = 64


//if (si >= 1083) cout << "--- si5: " << si << endl;
    si = 8+98+600+64+64;

    vector<Wizard> ws = world.getWizards ();
    std::sort (ws.begin (), ws.end (), [&me](const Wizard& o1, const Wizard& o2){
	if (o1.getFaction () == o2.getFaction ()) {
	    return o1.getId () < o2.getId ();
	} else {
	    if (o1.getFaction () == me.getFaction ()) {
		return true;
	    } else {
		return false;
	    }
	}
    });

//cout << "--- friends wizards" << endl;
    for (auto& w : ws) {

        if (w.isMe ()) continue;

	if (w.getFaction () != me.getFaction ()) {
	    break;
	}

//cout << " id: " << w.getId () << " f: " << w.getFaction () << " mf: " << me.getFaction () << endl;

//        double r = w.getCastRange ();
        nextState [si++] = w.getLevel ();
        nextState [si++] = 0.1 * w.getMana ();
//        int mm = w.getMaxMana ();
//        long oid = w.getOwnerPlayerId ();
        nextState [si++] = 0.1 * w.getRemainingActionCooldownTicks ();
//        double vr = w.getVisionRange ();
        nextState [si++] = 0.2 * w.getXp ();
        nextState [si++] = w.isMaster ()?5:0;
        //5
        

        const vector<int>& rcta = w.getRemainingCooldownTicksByAction ();
        int l = rcta.size ();
        for (int i=1; i<l; i++) {
            nextState [si++] = 0.1 * rcta [i];
        }

        //STAFF
        //MAGIC_MISSILE
        //FROST_BOLT
        //FIREBALL
        //HASTE
        //SHIELD
        //6
        
//        for (auto& s : w.getSkills ()) {
//            //s == SkillType
//            //RANGE_BONUS_PASSIVE_1
//            //RANGE_BONUS_AURA_1
//            //RANGE_BONUS_PASSIVE_2
//            //RANGE_BONUS_AURA_2
//            //ADVANCED_MAGIC_MISSILE
//            //MAGICAL_DAMAGE_BONUS_PASSIVE_1
//            //MAGICAL_DAMAGE_BONUS_AURA_1
//            //MAGICAL_DAMAGE_BONUS_PASSIVE_2
//            //MAGICAL_DAMAGE_BONUS_AURA_2
//            //FROST_BOLT
//            //STAFF_DAMAGE_BONUS_PASSIVE_1
//            //STAFF_DAMAGE_BONUS_AURA_1
//            //STAFF_DAMAGE_BONUS_PASSIVE_2
//            //STAFF_DAMAGE_BONUS_AURA_2
//            //FIREBALL
//            //MOVEMENT_BONUS_FACTOR_PASSIVE_1
//            //MOVEMENT_BONUS_FACTOR_AURA_1
//            //MOVEMENT_BONUS_FACTOR_PASSIVE_2
//            //MOVEMENT_BONUS_FACTOR_AURA_2
//            //HASTE
//            //MAGICAL_DAMAGE_ABSORPTION_PASSIVE_1
//            //MAGICAL_DAMAGE_ABSORPTION_AURA_1
//            //MAGICAL_DAMAGE_ABSORPTION_PASSIVE_2
//            //MAGICAL_DAMAGE_ABSORPTION_AURA_2
//            //SHIELD
//        }
        
        //living unit
        fillLivingUnit (w, nextState, si);

        //17
        //unit
        fillUnit (me, w, nextState, si);
        //7
    }


    si = 8+98+600+64+64+4*35;

    std::sort (ws.begin (), ws.end (), [&me](const Wizard& o1, const Wizard& o2){
	if (o1.getFaction () == o2.getFaction ()) {
	    return o1.getId () < o2.getId ();
	} else {
	    if (o1.getFaction () == me.getFaction ()) {
		return false;
	    } else {
		return true;
	    }
	}
    });

//cout << "--- enemy wizards" << endl;
    for (auto& w : ws) {

	if (w.getFaction () == me.getFaction ()) {
	    break;
	}

//cout << " id: " << w.getId () << " f: " << w.getFaction () << " mf: " << me.getFaction () << endl;

//        double r = w.getCastRange ();
        nextState [si++] = w.getLevel ();
        nextState [si++] = 0.1 * w.getMana ();
//        int mm = w.getMaxMana ();
//        long oid = w.getOwnerPlayerId ();
        nextState [si++] = 0.1 * w.getRemainingActionCooldownTicks ();
//        double vr = w.getVisionRange ();
        nextState [si++] = 0.2 * w.getXp ();
        nextState [si++] = w.isMaster ()?5:0;
        //5
        

        const vector<int>& rcta = w.getRemainingCooldownTicksByAction ();
        int l = rcta.size ();
        for (int i=1; i<l; i++) {
            nextState [si++] = 0.1 * rcta [i];
        }

        //STAFF
        //MAGIC_MISSILE
        //FROST_BOLT
        //FIREBALL
        //HASTE
        //SHIELD
        //6
        
//        for (auto& s : w.getSkills ()) {
//            //s == SkillType
//            //RANGE_BONUS_PASSIVE_1
//            //RANGE_BONUS_AURA_1
//            //RANGE_BONUS_PASSIVE_2
//            //RANGE_BONUS_AURA_2
//            //ADVANCED_MAGIC_MISSILE
//            //MAGICAL_DAMAGE_BONUS_PASSIVE_1
//            //MAGICAL_DAMAGE_BONUS_AURA_1
//            //MAGICAL_DAMAGE_BONUS_PASSIVE_2
//            //MAGICAL_DAMAGE_BONUS_AURA_2
//            //FROST_BOLT
//            //STAFF_DAMAGE_BONUS_PASSIVE_1
//            //STAFF_DAMAGE_BONUS_AURA_1
//            //STAFF_DAMAGE_BONUS_PASSIVE_2
//            //STAFF_DAMAGE_BONUS_AURA_2
//            //FIREBALL
//            //MOVEMENT_BONUS_FACTOR_PASSIVE_1
//            //MOVEMENT_BONUS_FACTOR_AURA_1
//            //MOVEMENT_BONUS_FACTOR_PASSIVE_2
//            //MOVEMENT_BONUS_FACTOR_AURA_2
//            //HASTE
//            //MAGICAL_DAMAGE_ABSORPTION_PASSIVE_1
//            //MAGICAL_DAMAGE_ABSORPTION_AURA_1
//            //MAGICAL_DAMAGE_ABSORPTION_PASSIVE_2
//            //MAGICAL_DAMAGE_ABSORPTION_AURA_2
//            //SHIELD
//        }
        
        //living unit
        fillLivingUnit (w, nextState, si);

        //17
        //unit
        fillUnit (me, w, nextState, si);
        //7
    }

    // 9 x 35 = 315

//if (si >= 1083) cout << "--- si6: " << si << endl;
    si = 8+98+600+64+64+315;

    //my wizard
    nextState [si++] = me.getLevel ();
    nextState [si++] = 0.1 * me.getMana ();
    nextState [si++] = 0.1 * me.getRemainingActionCooldownTicks ();
    nextState [si++] = 0.2 * me.getXp ();
    nextState [si++] = me.isMaster ()?5:0;
    //5
//if (si >= 984) cout << "--- si6.1: " << si << endl;
        
    const vector<int>& rcta = me.getRemainingCooldownTicksByAction ();
    int l = rcta.size ();
    for (int i=1; i<l; i++) {
        nextState [si++] = 0.1 * rcta [i];
    }
    //STAFF
    //MAGIC_MISSILE
    //FROST_BOLT
    //FIREBALL
    //HASTE
    //SHIELD
    //6
//if (si >= 990) cout << "--- si6.2: " << si << endl;
        
    //living unit
    fillLivingUnit (me, nextState, si);
    //17
//if (si >= 1007) cout << "--- si6.3: " << si << endl;
    //unit
    nextState [si++] = me.getAngle () * 10 / PI;
    nextState [si++] = 5 * me.getFaction ();
    nextState [si++] = 2 * me.getSpeedX ();
    nextState [si++] = 2 * me.getSpeedY ();

    nextState [si++] = me.getX () * 0.01;
    nextState [si++] = me.getY () * 0.01;
    nextState [si++] = 2 * (int)self.getId ();
    //6
/*
    for (int i=0; i<stateSize; i++) {
	nextState [i] *= 5;
    }
*/

/*    for (int i=0; i<stateSize/2; i++) {
	nextState [i+stateSize/2] = fabs (nextState [i]);
    }
*/
/*
    cout << "state" << endl;
    for (int i=0; i<stateSize; i++) {
        cout << nextState [i] << " ";
    }
    cout << "end state " << endl;
*/

//if (si >= 1083) cout << "--- si7: " << si << endl;

    //34
    
//    for (int i=8+98+500+64+64+245; i<8+98+500+64+64+245+34; i++) {
//        cout << nextState [i] << " ";
//        if ((i-8-98-500-64-64-245) % 34 == 33) cout << endl;
//    }
//    cout << endl;
    
    //input = 8+98+500+64+64+315+34 = 1083
    

    double x = me.getX ();
    double y = me.getY ();
    double r = sqrt((targetX-x)*(targetX-x) + (targetY-y)*(targetY-y));

    //if prev state
    if (!reseted) {
        //calc reward

	double targetReward = 0.025 * (prevR - r);
	if (fabs(targetReward)>1) {
	    targetReward = 0;
	}
//	reward = targetReward;

        double dxp = (me.getXp () - xp);// * 1.0 / 25.0;
        double dlife = (me.getLife () - life);// * 1.0 / 25.0;
	if (dlife>0) dlife = 0;

	double dmlife = (mainBaseNewLife - mainBaseLife);// * 1.0 / 25.0;

        reward = 0.01 * (1.0 * dxp + 0.5 * dlife);// + 0.1 * dmlife);

	if (reward == 0) {
	    reward = targetReward;
	}

/*	if (reward>5) reward = 5;
	if (reward<-5) reward = -5;
*/
//	if (reward == 0) reward = - 0.2;

        
        //send reward
        cl->sendReward (reward, nextState, action);

    }
    
    xp = me.getXp ();
    life = me.getLife ();
    mainBaseLife = mainBaseNewLife;
    prevR = r;
    
    actionReady = false;
    cl->sendState (nextState);
    
	std::unique_lock<std::mutex> lock(mtxAction);
	while (!actionReady) {
		cvAction.wait(lock);
	}
    
    

//    if (getRandom (0, 1) > 1) {

//    double prevT = move.getTurn ();
    simpleStrategy.move (self, world, game, move);
/*    double nextT = move.getTurn ();

    move.setMaxCastDistance (10000);
    move.setMinCastDistance (0);

    std::fill (action.begin (), action.end (), 0);
    int actionIndex = (int)move.getAction ();
    if (actionIndex >= 0 && actionIndex <7) {
        action [actionIndex] = 5.0;
    } else {
        action [0] = 5.0;
    }
    action [7] = move.getCastAngle ();
    if (prevT != nextT) {
	action [8] = 0;//move.getMaxCastDistance ();
	action [9] = 5;//move.getMinCastDistance ();
    } else {
	action [8] = 5;
	action [9] = 0;
    }
    action [10] = move.getSpeed ();
    action [11] = move.getStrafeSpeed ();
    action [12] = move.getTurn ();
*/    
//    } else {

	fillMove (move);

//    }
    
    //13
    //output = 38

    prevState = nextState;
    reseted = false;
    
}

MyStrategy::MyStrategy() {
    cl = std::shared_ptr<SimpleClient> (new SimpleClient (stateSize, actionSize));
    prevState.resize (stateSize);
    nextState.resize (stateSize);
    action.resize (actionSize);
    
    cl->connectToServer ();
    cl->setActionListener ([](const std::vector<double>& receivedAction){
//cout << "--- received action: " << receivedAction.size () << endl;
        action = receivedAction;
        actionReady = true;
        cvAction.notify_all ();
    });

}
