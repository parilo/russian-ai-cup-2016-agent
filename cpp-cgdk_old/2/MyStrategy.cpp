#include "MyStrategy.h"

#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES


#include <cmath>
#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <condition_variable>
#include <map>

#include "Strategy.h"

#include "SimpleClient.hpp"
#include "SimpleStrategy.h"

using namespace model;
using namespace std;
using namespace WizardTrainer;

SimpleStrategy simpleStrategy;

std::shared_ptr<WizardTrainer::SimpleClient> cl;

const int stateSize = 3144;
const int actionSize = 6;

std::vector<double> prevState;
std::vector<double> nextState;
std::vector<double> action;
double reward;
int xp;
int life;
int mainBaseLife;
bool reseted = true;

std::condition_variable cvAction;
std::mutex mtxAction;
bool actionReady = false;

double prevR = 0;

double targetX;
double targetY;

const int segmentsCount = 64;
const int segmentCapacity = 4;
const int segmentObjSize = 4;
const Wizard* myWizard;

double getRandom (double LO, double HI) {
    return LO + std::rand() /(RAND_MAX/(HI-LO));
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

    move.setSpeed (0.01 * action [0]); //0.002
    move.setStrafeSpeed (0.01 * action [1]);

    if (!simpleStrategy.isTurnSet ()) {
	move.setTurn (0.01 * action [2]);
    }

    if (!simpleStrategy.isActionSet ()) {
	move.setAction (static_cast<model::ActionType>(argmax (action.begin()+3, action.begin()+6)));
    }

}

int getSegment (const Unit& u) {
    double angle = myWizard->getAngleTo (u);
    int s = floor (segmentsCount * (angle + PI) / 2 / PI);
    return s==segmentsCount?segmentsCount-1:s;
}

int getSegment (double x, double y) {
    double angle = myWizard->getAngleTo (x, y);
    int s = floor (segmentsCount * (angle + PI) / 2 / PI);
    return s==segmentsCount?segmentsCount-1:s;
}

int getRCT (const Wizard& w) {
    int rct = w.getRemainingActionCooldownTicks ();

    const vector<int>& rcta = w.getRemainingCooldownTicksByAction ();
    for (auto& rcti : rcta) {
	if (rcti < rct) rct = rcti;
    }
    return rct;
}

enum class SegUnitType {
    Wizard = 2,
    MinionMelee = 4,
    MinionRange = 6,
    Tower = 8,
    Base = 10,
    Tree = 12,
    Bonus = 14,
    Projectile = 16
};

class SegInfo {
public:
    SegUnitType type;
    int index;
    double r;
};

void MyStrategy::move(const Wizard& self, const World& world, const Game& game, Move& move) {

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
    vector<Wizard> ws = world.getWizards ();
    for (int i=0; i<ws.size(); i++) {
        if (ws[i].isMe ()) {
            myWIndex = i;
        }
    }
    const Wizard& me = ws [myWIndex];
    myWizard = &me;
    auto myFac = me.getFaction ();

    std::fill(nextState.begin(), nextState.end(), 0);
    
    //friends
	//wizards 2
	//minions 4 6
	//towers 8
	//base 10

    //enemy
	//wizards
	//minions
	//towers
	//base

    //neutral
	//trees 12
	//minions 
	//bonuses 14

    //0 - type
    //1 - rct
    //2 - life
    //3 - r

    vector<Bonus> bns = world.getBonuses ();
    vector<Minion> ms = world.getMinions ();
    vector<Building> bs = world.getBuildings ();
    vector<Tree> ts = world.getTrees ();
    vector<Projectile> ps = world.getProjectiles ();

    vector<vector<SegInfo>> fsegs (segmentsCount);
    vector<vector<SegInfo>> esegs (segmentsCount);
    vector<vector<SegInfo>> nsegs (segmentsCount);

    {
    int wi = 0;
    for (auto& w : ws) {
	if (w.isMe ()) continue;

	SegInfo info;
	info.type = SegUnitType::Wizard;
	info.index = wi;
	info.r = me.getDistanceTo (w);
	
	int segment = getSegment (w);
//cout << "segment: " << segment;
	if (w.getFaction () == myFac) {
//cout << " f" << endl;
	    fsegs [segment].push_back (info);
	} else {
//cout << " e" << endl;
	    esegs [segment].push_back (info);
	}
	continue;

	wi++;
    }}

    {
    int mi = 0;
    for (auto& m : ms) {
	SegInfo info;
	info.type = m.getType () == MINION_ORC_WOODCUTTER? SegUnitType::MinionMelee : SegUnitType::MinionRange;
	info.index = mi;
	info.r = me.getDistanceTo (m);
	
	int segment = getSegment (m);
	auto fac = m.getFaction ();
	if (fac == FACTION_NEUTRAL) {
	    nsegs [segment].push_back (info);
	} else if (fac == myFac) {
	    fsegs [segment].push_back (info);
	} else {
	    esegs [segment].push_back (info);
	}

	mi++;
    }}

    {
    int bi = 0;
    for (auto& b : bs) {
	SegInfo info;
	info.type = b.getType () == BUILDING_GUARDIAN_TOWER? SegUnitType::Tower : SegUnitType::Base;
	info.index = bi;
	info.r = me.getDistanceTo (b);
	
	int segment = getSegment (b);
	auto fac = b.getFaction ();
	if (fac == myFac) {
	    fsegs [segment].push_back (info);
	} else {
	    esegs [segment].push_back (info);
	}

	bi++;
    }}

    {
    int bnsi = 0;
    for (auto& b : bns) {
	SegInfo info;
	info.type = SegUnitType::Bonus;
	info.index = bnsi;
	info.r = me.getDistanceTo (b);
	
	int segment = getSegment (b);
	nsegs [segment].push_back (info);

	bnsi++;
    }}

    {
    int pi = 0;
    for (auto& p : ps) {
	SegInfo info;
	info.type = SegUnitType::Projectile;
	info.index = pi;
	info.r = me.getDistanceTo (p);
	
	int segment = getSegment (p);
	nsegs [segment].push_back (info);

	pi++;
    }}

    {
    int ti = 0;
    for (auto& t : ts) {
	SegInfo info;
	info.type = SegUnitType::Tree;
	info.index = ti;
	info.r = me.getDistanceTo (t);
	
	int segment = getSegment (t);
	nsegs [segment].push_back (info);

	ti++;
    }}


    auto sortFunc = [](const SegInfo& o1, const SegInfo& o2){
	return o1.r < o2.r;
    };

    for (int i=0; i<segmentsCount; i++) {
	
	auto& fs = fsegs [i];
	auto& es = esegs [i];
	auto& ns = nsegs [i];

	std::sort (fs.begin (), fs.end (), sortFunc);
	std::sort (es.begin (), es.end (), sortFunc);
	std::sort (ns.begin (), ns.end (), sortFunc);

//	cout << fs.size () << "|" << es.size () << "|" << ns.size() << "  ";
    }
//    cout << endl;

    {
    const int fsegB = 0;
    int segi = 0;
    for (auto& fseg : fsegs) {

	int obji = 0;
	for (auto& obj : fseg) {
	
	    if (obji == segmentCapacity) break;

	    int objB = fsegB + (segi * segmentCapacity + obji) * segmentObjSize;

	    switch (obj.type) {
		case SegUnitType::Wizard:
		    {
		    auto& w = ws [obj.index];
		    nextState [objB    ] = (int)obj.type;
		    nextState [objB + 1] = 0.1 * getRCT (w);
		    nextState [objB + 2] = w.getLife ();
		    nextState [objB + 3] = 0.1 * obj.r;
		    }
		    break;

		case SegUnitType::MinionMelee:
		case SegUnitType::MinionRange:
		    {
		    auto& m = ms [obj.index];
		    nextState [objB    ] = (int)obj.type;
		    nextState [objB + 1] = 0.1 * m.getRemainingActionCooldownTicks ();
		    nextState [objB + 2] = m.getLife ();
		    nextState [objB + 3] = 0.1 * obj.r;
		    }
		    break;

		case SegUnitType::Tower:
		case SegUnitType::Base:
		    {
		    auto& b = bs [obj.index];
		    nextState [objB    ] = (int)obj.type;
		    nextState [objB + 1] = 0.1 * b.getRemainingActionCooldownTicks ();
		    nextState [objB + 2] = b.getLife ();
		    nextState [objB + 3] = 0.1 * obj.r;
		    }
		    break;
		default:
		    break;
	    }
	    obji++;
	}
	segi++;
    }}

    {
    const int esegB = segmentsCount * segmentCapacity * segmentObjSize;
    int segi = 0;
    for (auto& eseg : esegs) {

	int obji = 0;
	for (auto& obj : eseg) {
	
	    if (obji == segmentCapacity) break;

	    int objB = esegB + (segi * segmentCapacity + obji) * segmentObjSize;

	    switch (obj.type) {
		case SegUnitType::Wizard:
		    {
		    auto& w = ws [obj.index];
		    nextState [objB    ] = (int)obj.type;
		    nextState [objB + 1] = 0.1 * getRCT (w);
		    nextState [objB + 2] = w.getLife ();
		    nextState [objB + 3] = 0.1 * obj.r;
		    }
		    break;

		case SegUnitType::MinionMelee:
		case SegUnitType::MinionRange:
		    {
		    auto& m = ms [obj.index];
		    nextState [objB    ] = (int)obj.type;
		    nextState [objB + 1] = 0.1 * m.getRemainingActionCooldownTicks ();
		    nextState [objB + 2] = m.getLife ();
		    nextState [objB + 3] = 0.1 * obj.r;
		    }
		    break;

		case SegUnitType::Tower:
		case SegUnitType::Base:
		    {
		    auto& b = bs [obj.index];
		    nextState [objB    ] = (int)obj.type;
		    nextState [objB + 1] = 0.1 * b.getRemainingActionCooldownTicks ();
		    nextState [objB + 2] = b.getLife ();
		    nextState [objB + 3] = 0.1 * obj.r;
		    }
		    break;
		default:
		    break;
	    }
	    obji++;
	}
	segi++;
    }}

    {
    const int nsegB = 2 * segmentsCount * segmentCapacity * segmentObjSize;
    int segi = 0;
    for (auto& nseg : nsegs) {

	int obji = 0;
	for (auto& obj : nseg) {
	
	    if (obji == segmentCapacity) break;

	    int objB = nsegB + (segi * segmentCapacity + obji) * segmentObjSize;

	    switch (obj.type) {

		case SegUnitType::MinionMelee:
		case SegUnitType::MinionRange:
		    {
		    auto& m = ms [obj.index];
		    nextState [objB    ] = (int)obj.type;
		    nextState [objB + 1] = 0.1 * m.getRemainingActionCooldownTicks ();
		    nextState [objB + 2] = m.getLife ();
		    nextState [objB + 3] = 0.1 * obj.r;
		    }
		    break;

		case SegUnitType::Projectile:
		    {
		    auto& p = ps [obj.index];
		    nextState [objB    ] = (int)obj.type;
		    nextState [objB + 1] = p.getSpeedX ();
		    nextState [objB + 2] = p.getSpeedY ();
		    nextState [objB + 3] = 0.1 * obj.r;
		    }
		    break;
		case SegUnitType::Tree:
		    {
		    auto& t = ts [obj.index];
		    nextState [objB    ] = (int)obj.type;
		    nextState [objB + 2] = t.getLife ();
		    nextState [objB + 3] = 0.1 * obj.r;
		    }
		    break;
		default:
		    break;
	    }
	    obji++;
	}
	segi++;
    }}

    //target
    {
	int si = 3 * segmentsCount * segmentCapacity * segmentObjSize;
	int segment = getSegment (targetX, targetY);
	nextState [si + segment] = 0.01 * me.getDistanceTo (targetX, targetY);
    }

    //my wizard
    {
	int si = 3 * segmentsCount * segmentCapacity * segmentObjSize + 64;
	
	nextState [si++] = 0.1 * me.getRemainingActionCooldownTicks ();
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

	nextState [si++] = me.getLife ();
    }
    //8

/*
    {
    int b = 3 * segmentsCount * segmentCapacity * segmentObjSize;
    for (int i=b; i<b+64; i++) {
	cout << nextState [i] << " ";
    }
    cout << endl;
    }
*/

    //3 * 1024 + 64 + 8 = 3144
/*
std::vector<int> counts = {
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
std::vector<float> countsLimits = {
    5000, 2000, 1000, 500 ,200, 100, 50, 20, 10, 5, 2, 1, 0.5, 0.2, 0.1
};
const int countsSize = 15;

int vi = 0;
for (auto& v : nextState) {
    for (int i=0; i<countsSize; i++) {
	if (v > countsLimits [i]) {
	    counts [i] ++;
	    break;
	}
    }

    vi++;
}

	for (int i=0; i<countsSize; i++) {
	    std::cerr << " " << counts [i];
	}
	std::cerr << std::endl;
*/





    double x = me.getX ();
    double y = me.getY ();
    double r = sqrt((targetX-x)*(targetX-x) + (targetY-y)*(targetY-y));

    //if prev state
    if (!reseted) {
        //calc reward

	double targetReward = 0.1 * (prevR - r);
	if (targetReward>0) targetReward *= 0.8;
	if (fabs(targetReward)>1) {
	    targetReward = 0;
	}
//	reward = targetReward;

        double dxp = (me.getXp () - xp);// * 1.0 / 25.0;
        double dlife = (me.getLife () - life);// * 1.0 / 25.0;
	if (dlife>0) dlife = 0;

//	double dmlife = (mainBaseNewLife - mainBaseLife);// * 1.0 / 25.0;

        reward = 0.2 * (1.0 * dxp + 0.2 * dlife);// + 0.1 * dmlife);

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
//    mainBaseLife = mainBaseNewLife;
    prevR = r;
    
    actionReady = false;
    cl->sendState (nextState);
    
    std::unique_lock<std::mutex> lock(mtxAction);
    while (!actionReady) {
	cvAction.wait(lock);
    }
    

//    if (getRandom (0, 1) > 1) {

    simpleStrategy.move (self, world, game, move);

//    double prevT = move.getTurn ();
//    simpleStrategy.move (self, world, game, move);
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
