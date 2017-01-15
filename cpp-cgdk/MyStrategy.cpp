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

const int stateSize = 394; //234;//394;//714; //1354;//714;//394;//3144;
const int actionSize = 6;

std::vector<float> prevState;
std::vector<float> nextState;
std::vector<float> action;
vector<float> prevSensors;
float reward;
int xp;
int life;
int mainBaseLife;
bool reseted = true;

std::condition_variable cvAction;
std::mutex mtxAction;
bool actionReady = false;

double prevR = 0;

double baseX;
double baseY;
double targetX;
double targetY;

const int segmentsCount = 64;
const int segmentCapacity = 1;
const int segmentObjSize = 5;
const Wizard* myWizard;
int tickIndex = 0;

double getRandom (double LO, double HI) {
    return LO + std::rand() /(RAND_MAX/(HI-LO));
}

int argmax (std::vector<float>::iterator begin, std::vector<float>::iterator end) {
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


bool externalStrategyUsed;

void fillMove (Move& move) {

    move.setSpeed (action [0]); //0.002
    move.setStrafeSpeed (action [1]);


    if (simpleStrategy.isTurnSet ()) {
//	action [2] = move.getTurn ();
    } else {
	move.setTurn (action [2]);
    }


    if (simpleStrategy.isActionSet ()) {
//	fill (action.begin ()+3, action.begin ()+6, 0);
//	action [3 + (int)move.getAction ()] = 1;
    } else {
	move.setAction (static_cast<model::ActionType>(argmax (action.begin()+3, action.begin()+6)));
    }


}

int getSegment (double x, double y, int segCount = segmentsCount) {
    double angle = myWizard->getAngleTo (x, y);
    int s = floor (segCount * (angle + PI) / 2 / PI);
    s = (segCount - s + segCount/2) % segCount;
//    return (segmentsCount - (s==segmentsCount?segmentsCount-1:s) + segmentsCount/2) % segmentsCount;
//    int s = floor (segCount * (angle + PI) / 2 / PI);
//    s = (s==segCount?segCount-1:s);
//if (s < 0 || s>=128)
//cout << "--- seg: " << s << endl;
    return s;
}

int getSegment (const Unit& u) {
    return getSegment (u.getX (), u.getY ());
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
    Wizard = 1,
    MinionMelee = 2,
    MinionRange = 3,
    Tower = 4,
    Base = 5,
    Tree = 6,
    Bonus = 7,
    Projectile = 8,
    Empty = 9
};

class SegInfo {
public:
    SegUnitType type;
    int index;
    double r;
    double radius;
    double life;
    double rcta;
    double faction;
};

bool getLineSegmentsIntersection (
    float p0_x, float p0_y, //first line points
    float p1_x, float p1_y,
    float p2_x, float p2_y, //sencond lise points
    float p3_x, float p3_y,
    float& i_x, float& i_y,
    float& t//intersection point in part of length of first line
) {
    float s02_x, s02_y, s10_x, s10_y, s32_x, s32_y, s_numer, t_numer, denom;
    s10_x = p1_x - p0_x;
    s10_y = p1_y - p0_y;
    s32_x = p3_x - p2_x;
    s32_y = p3_y - p2_y;

    denom = s10_x * s32_y - s32_x * s10_y;
    if (denom == 0)
        return false; // Collinear
    bool denomPositive = denom > 0;

    s02_x = p0_x - p2_x;
    s02_y = p0_y - p2_y;
    s_numer = s10_x * s02_y - s10_y * s02_x;
    if ((s_numer < 0) == denomPositive)
        return false; // No collision

    t_numer = s32_x * s02_y - s32_y * s02_x;
    if ((t_numer < 0) == denomPositive)
        return false; // No collision

    if (((s_numer > denom) == denomPositive) || ((t_numer > denom) == denomPositive))
        return false; // No collision
    // Collision detected
    t = t_numer / denom;
    i_x = p0_x + (t * s10_x);
    i_y = p0_y + (t * s10_y);

    return true;
}

void MyStrategy::move(const Wizard& self, const World& world, const Game& game, Move& move) {


    bool died = false;
    if (world.getTickIndex () != tickIndex) {
	died = true;
	life = self.getLife ();
	xp = self.getXp ();
	tickIndex = world.getTickIndex ();
    }
    tickIndex++;


    simpleStrategy.move (self, world, game, move);
    externalStrategyUsed = simpleStrategy.isTurnSet () || simpleStrategy.isActionSet ();
    bool enemyNear = simpleStrategy.isEnemyNear (self, world, game, move);


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


    double mapSize = game.getMapSize();
    if (reseted) {
        switch ((int) self.getId()) {
                case 1:
                case 2:
                case 3:
                case 4:
                case 5:
                case 6:
                case 7:
                case 8:
                case 9:
                case 10:
		    baseX = 200;
		    baseY = mapSize - 200;
                    break;
/*                case 6:
                case 7:
                case 8:
                case 9:
                case 10:
		    baseX = mapSize - 200;
		    baseY = 200;
                    break;
*/                default:
                    break;
            }
    }

    if ( (me.getDistanceTo (baseX, baseY) < 1000) || died || reseted) {

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

    if (me.getDistanceTo (targetX, targetY) < 200) {
	//next target

        switch ((int) self.getId()) {
                case 1:
                case 2:
                case 3:
                case 4:
                case 5:
                case 6:
                case 7:
                case 8:
                case 9:
                case 10:
		    targetX = mapSize - 200;
		    targetY = 200;
                    break;
/*                case 6:
                case 7:
                case 8:
                case 9:
                case 10:
		    targetX = 200;
		    targetY = mapSize - 200;
                    break;
*/                default:
                    break;
            }
    }


    std::fill(nextState.begin(), nextState.end(), 0);
//    std::fill(nextState.begin(), nextState.begin() + segmentsCount * segmentCapacity * segmentObjSize, 0.0);
    
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

    vector<vector<SegInfo>> segs (segmentsCount);
//    vector<vector<SegInfo>> fsegs (segmentsCount);
//    vector<vector<SegInfo>> esegs (segmentsCount);
//    vector<vector<SegInfo>> nsegs (segmentsCount);

    {
    int wi = 0;
    for (auto& w : ws) {
	if (w.isMe ()) continue;

	SegInfo info;
	info.type = SegUnitType::Wizard;
	info.index = wi;
	info.r = me.getDistanceTo (w);
	info.radius = w.getRadius ();
	info.rcta = 0.0083 * getRCT (w);
	info.life = 0.01 * w.getLife ();
	Faction f = w.getFaction ();
	info.faction = f == FACTION_NEUTRAL?1.0:(f==myFac?0.0:0.5);
	
	int segment = getSegment (w);
	segs [segment].push_back (info);

	wi++;
    }}

    {
    int mi = 0;
    for (auto& m : ms) {
	SegInfo info;
	info.type = m.getType () == MINION_ORC_WOODCUTTER? SegUnitType::MinionMelee : SegUnitType::MinionRange;
	info.index = mi;
	info.r = me.getDistanceTo (m);
	info.radius = m.getRadius ();

	info.rcta = 0.0083 * m.getRemainingActionCooldownTicks ();
	info.life = 0.01 * m.getLife ();
	Faction f = m.getFaction ();
	info.faction = f == FACTION_NEUTRAL?1.0:(f==myFac?0.0:0.5);
	
	int segment = getSegment (m);
	segs [segment].push_back (info);

	mi++;
    }}

    {
    int bi = 0;
    for (auto& b : bs) {
	SegInfo info;
	info.type = b.getType () == BUILDING_GUARDIAN_TOWER? SegUnitType::Tower : SegUnitType::Base;
	info.index = bi;
	info.r = me.getDistanceTo (b);
	info.radius = b.getRadius ();

	info.rcta = 0.0083 * b.getRemainingActionCooldownTicks ();
	info.life = 0.01 * b.getLife ();
	Faction f = b.getFaction ();
	info.faction = f == FACTION_NEUTRAL?1.0:(f==myFac?0.0:0.5);
	
	int segment = getSegment (b);
	segs [segment].push_back (info);

	bi++;
    }}

    {
    int bnsi = 0;
    for (auto& b : bns) {
	SegInfo info;
	info.type = SegUnitType::Bonus;
	info.index = bnsi;
	info.r = me.getDistanceTo (b);
	info.radius = b.getRadius ();
	info.rcta = 1;
	info.life = 1;
	info.faction = 1;
	
	int segment = getSegment (b);
	segs [segment].push_back (info);

	bnsi++;
    }}

/*
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
*/

    {
    int ti = 0;
    for (auto& t : ts) {
	SegInfo info;
	info.type = SegUnitType::Tree;
	info.index = ti;
	info.r = me.getDistanceTo (t);
	info.radius = t.getRadius ();
	info.rcta = 1;
	info.life = 0.01 * t.getLife ();
	info.faction = 1;
	
	int segment = getSegment (t);
	segs [segment].push_back (info);

	ti++;
    }}


    auto sortFunc = [](const SegInfo& o1, const SegInfo& o2){
	return o1.r < o2.r;
    };

    for (int i=0; i<segmentsCount; i++) {
	
	auto& s = segs [i];

	std::sort (s.begin (), s.end (), sortFunc);

//	cout << s.size () << "  ";
    }
//    cout << endl;


    const int sensorsCount = segmentsCount;
    const float sensorsLength = 1000;
    vector<float> sensors (sensorsCount);
{
    float x0 = 0;
    float y0 = 0;
    float x1 = 4000;
    float y1 = 0;
    float x2 = 4000;
    float y2 = 4000;
    float x3 = 0;
    float y3 = 4000;
//    o.getPoints (x0, y0, x1, y1, x2, y2, x3, y3);

    float posX = me.getX ();
    float posY = me.getY ();
    float angle = me.getAngle () - 1*PI/2;// - PI/sensorsCount/2;
//    getMainCoords(posX, posY, angle);


    for (int i=0; i<sensorsCount; i++) {
    float sx = sensorsLength * sinf (i * 2 * M_PI / sensorsCount - angle) + posX;
    float sy = sensorsLength * cosf (i * 2 * M_PI / sensorsCount - angle) + posY;

    float ix, iy, t;
    float minT = 1;
    if (getLineSegmentsIntersection(posX, posY, sx, sy, x0, y0, x1, y1, ix, iy, t)) {
        if (t < minT) minT = t;
    }
    if (getLineSegmentsIntersection(posX, posY, sx, sy, x1, y1, x2, y2, ix, iy, t)) {
        if (t < minT) minT = t;
    }
    if (getLineSegmentsIntersection(posX, posY, sx, sy, x2, y2, x3, y3, ix, iy, t)) {
        if (t < minT) minT = t;
    }
    if (getLineSegmentsIntersection(posX, posY, sx, sy, x3, y3, x0, y0, ix, iy, t)) {
        if (t < minT) minT = t;
    }
    
//    minT *= sensorsLength;

    sensors [i] = minT;
    
    }
}


    {
    const int segB = 0;
    int segi = 0;
    for (auto& seg : segs) {

	int obji = 0;
	for (auto& obj : seg) {
	
	    if (obji == segmentCapacity) break;

	    int objB = segB + (segi * segmentCapacity + obji) * segmentObjSize;

	    nextState [objB    ] = ((float)(int)obj.type)/9.0;
	    nextState [objB + 1] = (float)obj.rcta;
	    nextState [objB + 2] = (float)obj.life;
	    nextState [objB + 3] = (float)0.001 * obj.r;
	    nextState [objB + 4] = (float)obj.faction;

	    obji++;
	}

	if (seg.size () == 0) {
	    int objB = segB + (segi * segmentCapacity + obji) * segmentObjSize;

	    nextState [objB    ] = 1;
	    nextState [objB + 1] = 0;//(float)obj.rcta;
	    nextState [objB + 2] = 0;//(float)obj.life;
	    nextState [objB + 3] = sensors [segi];//(float)0.001 * obj.r;
	    nextState [objB + 4] = 1;//(float)obj.faction;
	}

	segi++;
    }}


    //my wizard
    {
	int si = segmentsCount * segmentCapacity * segmentObjSize;
	
	nextState [si++] = (float)0.0083 * me.getRemainingActionCooldownTicks ();
	const vector<int>& rcta = me.getRemainingCooldownTicksByAction ();
	int l = rcta.size ();
	for (int i=1; i<l; i++) {
	    nextState [si++] = (float) 0.0083 * rcta [i];
	}
	//STAFF
	//MAGIC_MISSILE
	//FROST_BOLT
	//FIREBALL
	//HASTE
	//SHIELD
	//6

	nextState [si++] = (float) 0.01 * me.getLife ();

//	nextState [si++] = (float) sin(me.getAngle ());
//	nextState [si++] = (float) cos(me.getAngle ());

//	double tx = targetX - me.getX ();
//	double ty = targetY - me.getY ();
//	double tr = me.getDistanceTo (targetX, targetY);
//	nextState [si++] = (float) (tx / tr);
//	nextState [si++] = (float) (ty / tr);

	nextState [si++] = (float) (externalStrategyUsed?1:0);
	nextState [si++] = (float) (enemyNear?1:0);

    }
    //10

//    fill (nextState.begin () , nextState.begin () + segmentsCount * segmentCapacity * segmentObjSize + 10, 0);

    //target
    {
	int si = segmentsCount * segmentCapacity * segmentObjSize + 10;
	fill (nextState.begin () + si, nextState.begin () + si + 64, 0);
	int segment = getSegment (targetX, targetY, 64);
	nextState [si + segment] = 1;//0.00025 * me.getDistanceTo (targetX, targetY);
    }

    for (auto& v : nextState) {
	v *= 10.0;
    }

    //256 * 5 + 14 = 1294

/*
    for (auto& v : nextState) {
	cout << v << " ";
    }
    cout << endl << endl;
*/

/*
{
    int si = 0;
    for (auto& v : nextState) {
	if (
	    si == 3 * segmentsCount * segmentCapacity * segmentObjSize + 64 + 8
	) cout << endl;

	cout << v << " ";

	if (
	    si % 64 == 63
	) cout << endl;
	si++;
    }
    cout << endl;
    cout << endl;
}
*/

/*
    {
    int s = segmentsCount * segmentCapacity * segmentObjSize;
    for (int j=0; j<4; j++) {
    for (int i=s*j; i<s*j+64; i++) {
	cout << nextState [i] << " ";
    }
    cout << endl;
    }
    cout << endl;
    }
*/

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

	double targetReward = 0;
//	if (tickIndex % 10 == 9) {
	    targetReward = 0.06 * (prevR - r);
//	    if (targetReward>0) targetReward *= 0.6;
	    if (fabs(targetReward)>10) {
		targetReward = 0;
	    }

	    prevR = r;
//	}
	reward = targetReward;

        double dxp = (me.getXp () - xp);// * 1.0 / 25.0;
        double dlife = (me.getLife () - life);// * 1.0 / 25.0;
	if (dlife>0) dlife = 0;

//	double dmlife = (mainBaseNewLife - mainBaseLife);// * 1.0 / 25.0;
	if (dxp < 30) dxp *= 2;

        reward = 0.06 * (dxp + 0.2 * dlife);// + 0.1 * dmlife);

	if (reward >= 0 && simpleStrategy.isActionSet ()) {
	    reward += 0.5;
//cout << "--- reward for action: " << reward << endl;
	}

	if (!enemyNear) {
	    reward += targetReward;
	}

	if (died) {
	    reward = -5;
	    prevR = r;
	    targetReward = 0;
	}

	if (reward > 15) reward = 15;

        //send reward
        cl->sendReward (reward, prevState, nextState, action);

    }
    
    xp = me.getXp ();
    life = me.getLife ();
//    mainBaseLife = mainBaseNewLife;
    
    actionReady = false;
    cl->sendState (nextState);
    
    std::unique_lock<std::mutex> lock(mtxAction);
    while (!actionReady) {
	cvAction.wait(lock);
    }
    

    fillMove (move);

    prevState = nextState;
    reseted = false;

}

MyStrategy::MyStrategy() {
    cl = std::shared_ptr<SimpleClient> (new SimpleClient (stateSize, actionSize));
    prevState.resize (stateSize);
    nextState.resize (stateSize);
    action.resize (actionSize);
    
    cl->connectToServer ();
    cl->setActionListener ([](const std::vector<float>& receivedAction){
//cout << "--- received action: " << receivedAction.size () << endl;
        action = receivedAction;
        actionReady = true;
        cvAction.notify_all ();
    });

}
