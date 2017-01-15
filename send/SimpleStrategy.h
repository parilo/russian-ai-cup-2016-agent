#define PI 3.14159265358979323846
#define _USE_MATH_DEFINES

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <algorithm>
#include <condition_variable>
#include <map>

using namespace model;
using namespace std;
//using namespace WizardTrainer;


/**
 * Вспомогательный класс для хранения позиций на карте.
 */
class Point2D {
private:
    double x;
    double y;
    
public:
    Point2D(double x, double y) :
        x (x),
        y (y)
    {}

    double getX() const {
        return x;
    }

    double getY() const {
        return y;
    }

    double getDistanceTo(double x, double y) const {
        return hypot(this->x - x, this->y - y);
    }

    double getDistanceTo(const Point2D& point) const {
        return getDistanceTo(point.x, point.y);
    }

    double getDistanceTo(const Unit& unit) const {
        return getDistanceTo(unit.getX(), unit.getY());
    }
};

class Random {
private:
public:
    Random (long long seed) {
        srand ((unsigned int)seed);
    }
    
    bool nextBoolean () {
        return nextDouble (0, 1)>=0.5?1:0;
    }

    double nextDouble (double LO, double HI) {
        return LO + std::rand() /(RAND_MAX/(HI-LO));
    }

};

class SimpleStrategy : public Strategy {
private:
    const double WAYPOINT_RADIUS = 100.0;

    const double LOW_HP_FACTOR = 0.0;

    /**
     * Ключевые точки для каждой линии, позволяющие упростить управление перемещением волшебника.
     * <p>
     * Если всё хорошо, двигаемся к следующей точке и атакуем противников.
     * Если осталось мало жизненной энергии, отступаем к предыдущей точке.
     */
    map<int, vector<Point2D>> waypointsByLane;

    std::shared_ptr<Random> random;

    //LaneType lane;
    int lane;
    vector<Point2D> waypoints;

    const Wizard* self;
    const World* world;
    const Game* game;
    Move* moveObj;

    bool turnSet;
    bool actionSet;
    
    vector<SkillType> skillsSchedule;
    int skillsScheduleIndex = 0;

public:
    
    SimpleStrategy ()
    {
	skillsSchedule = {
        SKILL_RANGE_BONUS_PASSIVE_1,
        SKILL_RANGE_BONUS_AURA_1,
        SKILL_RANGE_BONUS_PASSIVE_2,
        SKILL_RANGE_BONUS_AURA_2,
        SKILL_ADVANCED_MAGIC_MISSILE,
        SKILL_MAGICAL_DAMAGE_BONUS_PASSIVE_1,
        SKILL_MAGICAL_DAMAGE_BONUS_AURA_1,
        SKILL_MAGICAL_DAMAGE_BONUS_PASSIVE_2,
        SKILL_MAGICAL_DAMAGE_BONUS_AURA_2,
        SKILL_FROST_BOLT,
        SKILL_MOVEMENT_BONUS_FACTOR_PASSIVE_1,
        SKILL_MOVEMENT_BONUS_FACTOR_AURA_1,
        SKILL_MOVEMENT_BONUS_FACTOR_PASSIVE_2,
        SKILL_MOVEMENT_BONUS_FACTOR_AURA_2,
        SKILL_HASTE,
        SKILL_MAGICAL_DAMAGE_ABSORPTION_PASSIVE_1,
        SKILL_MAGICAL_DAMAGE_ABSORPTION_AURA_1,
        SKILL_MAGICAL_DAMAGE_ABSORPTION_PASSIVE_2,
        SKILL_MAGICAL_DAMAGE_ABSORPTION_AURA_2,
        SKILL_SHIELD,
        SKILL_STAFF_DAMAGE_BONUS_PASSIVE_1,
        SKILL_STAFF_DAMAGE_BONUS_AURA_1,
        SKILL_STAFF_DAMAGE_BONUS_PASSIVE_2,
        SKILL_STAFF_DAMAGE_BONUS_AURA_2,
        SKILL_FIREBALL
	};
    }

    bool isTurnSet () {
	return turnSet;
    }

    bool isActionSet () {
	return actionSet;
    }
    
    /**
     * Основной метод стратегии, осуществляющий управление волшебником.
     * Вызывается каждый тик для каждого волшебника.
     *
     * @param self  Волшебник, которым данный метод будет осуществлять управление.
     * @param world Текущее состояние мира.
     * @param game  Различные игровые константы.
     * @param move  Результатом работы метода является изменение полей данного объекта.
     */
    void move (const Wizard& self, const World& world, const Game& game, Move& move) override {
	turnSet = false;
	actionSet = false;
        initializeStrategy(self, game);
        initializeTick(self, world, game, move);
        
        if (game.isSkillsEnabled ()) {
		auto currentSkillToLearn = skillsSchedule [skillsScheduleIndex];
		for (auto s : self.getSkills ()) {
		    if (s == currentSkillToLearn) {
//cout << "--- learned skill: " << s << endl;
			skillsScheduleIndex++;
			currentSkillToLearn = skillsSchedule [skillsScheduleIndex];
		    }
		}
		move.setSkillToLearn (currentSkillToLearn);
        }

        // Постоянно двигаемся из-стороны в сторону, чтобы по нам было сложнее попасть.
        // Считаете, что сможете придумать более эффективный алгоритм уклонения? Попробуйте! ;)
        move.setStrafeSpeed(random->nextBoolean() ? game.getWizardStrafeSpeed() : -game.getWizardStrafeSpeed());

        // Если осталось мало жизненной энергии, отступаем к предыдущей ключевой точке на линии.
/*        if (self.getLife() < self.getMaxLife() * LOW_HP_FACTOR) {
            goTo(getPreviousWaypoint());
            return;
        }
*/
        const LivingUnit* nearestTarget = getWeakestTarget();

/*
namespace model {
    enum ActionType {
        _ACTION_UNKNOWN_ = -1,
        ACTION_NONE = 0,
        ACTION_STAFF = 1,
        ACTION_MAGIC_MISSILE = 2,
        ACTION_FROST_BOLT = 3,
        ACTION_FIREBALL = 4,
        ACTION_HASTE = 5,
        ACTION_SHIELD = 6,
        _ACTION_COUNT_ = 7
    };
}
*/

	vector<ActionType> priority = {ACTION_SHIELD, ACTION_HASTE, ACTION_FIREBALL, ACTION_FROST_BOLT, ACTION_MAGIC_MISSILE};
	vector<bool> haveSkill = {false, false, false, false, false};
	int mana = self.getMana ();
	auto rcta = self.getRemainingCooldownTicksByAction ();
	
	haveSkill [int(ACTION_MAGIC_MISSILE) - 2] = true;
	for (auto s : self.getSkills ()) {
	    switch (s) {
		case SKILL_FROST_BOLT:
		    haveSkill [int(ACTION_FROST_BOLT) - 2] = true;
		    break;
		case SKILL_HASTE:
		    haveSkill [int(ACTION_HASTE) - 2] = true;
		    break;
		case SKILL_SHIELD:
		    haveSkill [int(ACTION_SHIELD) - 2] = true;
		    break;
		case SKILL_FIREBALL:
		    haveSkill [int(ACTION_FIREBALL) - 2] = true;
		    break;
		default:
		    break;
	    }
	}
	
	vector<bool> haveMana = {
	    game.getMagicMissileManacost () < mana,
	    game.getFrostBoltManacost () < mana,
	    game.getFireballManacost () < mana,
	    game.getHasteManacost () < mana,
	    game.getShieldManacost () < mana
	};
	
	ActionType choosedAction = ACTION_NONE;
	for (auto a : priority) {
	    int aint = int (a) - 2;
	    if (
		haveSkill [aint] &&
		haveMana [aint] &&
		rcta [a] == 0
	    ) {
		choosedAction = a;
		break;
	    }
	}
	
	if (choosedAction == ACTION_HASTE || choosedAction == ACTION_SHIELD) {
//cout << "--- use spell: " << choosedAction << endl;
	    move.setAction (choosedAction);
	    actionSet = true;
	}

        // Если видим противника ...
        if (nearestTarget != nullptr) {
            double distance = self.getDistanceTo(*nearestTarget);

            // ... и он в пределах досягаемости наших заклинаний, ...
            if (distance <= self.getCastRange()) {
                double angle = self.getAngleTo(*nearestTarget);

                // ... то поворачиваемся к цели.
                move.setTurn(angle);
		turnSet = true;

                // Если цель перед нами, ...
                if (fabs(angle) < game.getStaffSector() / 2.0) {
                    // ... то атакуем.

	if (choosedAction != ACTION_NONE) {
cout << "--- use attack: " << choosedAction << endl;
	}

                    move.setAction(choosedAction);
		    actionSet = true;
                    move.setCastAngle(angle);
                    move.setMinCastDistance(distance - nearestTarget->getRadius() + game.getMagicMissileRadius());
                }

                return;
            }
        }

        // Если нет других действий, просто продвигаемся вперёд.
//        goTo(getNextWaypoint());
    }

    bool isEnemyNear (const Wizard& self, const World& world, const Game& game, Move& move) {

        const LivingUnit* nearestTarget = getNearestTarget();

        if (nearestTarget != nullptr) {
            double distance = self.getDistanceTo(*nearestTarget);

            return distance < 0.95 * self.getCastRange();
        }

	return false;
    }

    /**
     * Инциализируем стратегию.
     * <p>
     * Для этих целей обычно можно использовать конструктор, однако в данном случае мы хотим инициализировать генератор
     * случайных чисел значением, полученным от симулятора игры.
     */
    void initializeStrategy(const Wizard& self, const Game& game) {
        if (!random) {
            random = std::shared_ptr<Random>(new Random(game.getRandomSeed()));

            double mapSize = game.getMapSize();

            waypointsByLane.insert(std::pair<int, vector<Point2D>>((int)1/*LaneType.MIDDLE*/, {
                Point2D(100.0, mapSize - 100.0),
                random->nextBoolean()
                        ? Point2D(600.0, mapSize - 200.0)
                        : Point2D(200.0, mapSize - 600.0),
                Point2D(800.0, mapSize - 800.0),
                Point2D(mapSize - 600.0, 600.0)
            }));

            //LaneType.TOP
            waypointsByLane.insert(std::pair<int, vector<Point2D>>((int)0, {
                Point2D(100.0, mapSize - 100.0),
                Point2D(100.0, mapSize - 400.0),
                Point2D(200.0, mapSize - 800.0),
                Point2D(200.0, mapSize * 0.75),
                Point2D(200.0, mapSize * 0.5),
                Point2D(200.0, mapSize * 0.25),
                Point2D(200.0, 200.0),
                Point2D(mapSize * 0.25, 200.0),
                Point2D(mapSize * 0.5, 200.0),
                Point2D(mapSize * 0.75, 200.0),
                Point2D(mapSize - 200.0, 200.0)
            }));
            
            //LaneType.BOTTOM
            waypointsByLane.insert(std::pair<int, vector<Point2D>>((int)2, {
                Point2D(100.0, mapSize - 100.0),
                Point2D(400.0, mapSize - 100.0),
                Point2D(800.0, mapSize - 200.0),
                Point2D(mapSize * 0.25, mapSize - 200.0),
                Point2D(mapSize * 0.5, mapSize - 200.0),
                Point2D(mapSize * 0.75, mapSize - 200.0),
                Point2D(mapSize - 200.0, mapSize - 200.0),
                Point2D(mapSize - 200.0, mapSize * 0.75),
                Point2D(mapSize - 200.0, mapSize * 0.5),
                Point2D(mapSize - 200.0, mapSize * 0.25),
                Point2D(mapSize - 200.0, 200.0)
            }));

            switch ((int) self.getId()) {
                case 1:
                case 2:
                case 6:
                case 7:
                    lane = 0;//LaneType.TOP;
                    break;
                case 3:
                case 8:
                    lane = 1;//LaneType.MIDDLE;
                    break;
                case 4:
                case 5:
                case 9:
                case 10:
                    lane = 2;//LaneType.BOTTOM;
                    break;
                default:
                    break;
            }

            waypoints = waypointsByLane.at(lane);

            // Наша стратегия исходит из предположения, что заданные нами ключевые точки упорядочены по убыванию
            // дальности до последней ключевой точки. Сейчас проверка этого факта отключена, однако вы можете
            // написать свою проверку, если решите изменить координаты ключевых точек.

            /*Point2D lastWaypoint = waypoints[waypoints.length - 1];

            Preconditions.checkState(ArrayUtils.isSorted(waypoints, (waypointA, waypointB) -> Double.compare(
                    waypointB.getDistanceTo(lastWaypoint), waypointA.getDistanceTo(lastWaypoint)
            )));*/
        }
    }

    /**
     * Сохраняем все входные данные в полях класса для упрощения доступа к ним.
     */
    void initializeTick(const Wizard& self, const World& world, const Game& game, Move& move) {
        this->self = &self;
        this->world = &world;
        this->game = &game;
        this->moveObj = &move;
    }

    /**
     * Данный метод предполагает, что все ключевые точки на линии упорядочены по уменьшению дистанции до последней
     * ключевой точки. Перебирая их по порядку, находим первую попавшуюся точку, которая находится ближе к последней
     * точке на линии, чем волшебник. Это и будет следующей ключевой точкой.
     * <p>
     * Дополнительно проверяем, не находится ли волшебник достаточно близко к какой-либо из ключевых точек. Если это
     * так, то мы сразу возвращаем следующую ключевую точку.
     */
    Point2D getNextWaypoint() {
        int lastWaypointIndex = waypoints.size () - 1;
        Point2D lastWaypoint = waypoints[lastWaypointIndex];

        for (int waypointIndex = 0; waypointIndex < lastWaypointIndex; ++waypointIndex) {
            Point2D waypoint = waypoints[waypointIndex];

            if (waypoint.getDistanceTo(*self) <= WAYPOINT_RADIUS) {
                return waypoints[waypointIndex + 1];
            }

            if (lastWaypoint.getDistanceTo(waypoint) < lastWaypoint.getDistanceTo(*self)) {
                return waypoint;
            }
        }

        return lastWaypoint;
    }

    /**
     * Действие данного метода абсолютно идентично действию метода {@code getNextWaypoint}, если перевернуть массив
     * {@code waypoints}.
     */
    Point2D getPreviousWaypoint() {
        Point2D firstWaypoint = waypoints[0];

        for (int waypointIndex = waypoints.size () - 1; waypointIndex > 0; --waypointIndex) {
            Point2D waypoint = waypoints[waypointIndex];

            if (waypoint.getDistanceTo(*self) <= WAYPOINT_RADIUS) {
                return waypoints[waypointIndex - 1];
            }

            if (firstWaypoint.getDistanceTo(waypoint) < firstWaypoint.getDistanceTo(*self)) {
                return waypoint;
            }
        }

        return firstWaypoint;
    }

    /**
     * Простейший способ перемещения волшебника.
     */
    void goTo(const Point2D& point) {
        double angle = self->getAngleTo(point.getX(), point.getY());

        moveObj->setTurn(angle);

        if (fabs(angle) < game->getStaffSector() / 4.0) {
            moveObj->setSpeed(game->getWizardForwardSpeed());
        }
    }

    /**
     * Находим ближайшую цель для атаки, независимо от её типа и других характеристик.
     */
    const LivingUnit* getNearestTarget() {
        vector<const LivingUnit*> targets;
        for (auto& o : world->getBuildings()) {
            targets.push_back (&o);
        }
        for (auto& o : world->getWizards()) {
            targets.push_back (&o);
        }
        for (auto& o : world->getMinions()) {
            targets.push_back (&o);
        }

        const LivingUnit* nearestTarget = nullptr;
//        int nearestTargetIndex = -1;
        double nearestTargetDistance = 100000;

        int i = 0;
        for (auto& target : targets) {
            if (target->getFaction() == FACTION_NEUTRAL || target->getFaction() == self->getFaction()) {
                continue;
            }

            double distance = self->getDistanceTo(*target);

            if (distance < nearestTargetDistance) {
//                nearestTargetIndex = i;
                nearestTarget = target;
                nearestTargetDistance = distance;
            }
            
            i++;
        }

        return nearestTarget;
    }

    const LivingUnit* getWeakestTarget () {

	auto myFac = self->getFaction ();

        vector<const LivingUnit*> targets;
        for (auto& o : world->getBuildings()) {
	    if (o.getFaction () != myFac)
	    if (o.getFaction () != FACTION_NEUTRAL)
	    {
		targets.push_back (&o);
	    }
        }
        for (auto& o : world->getWizards()) {
	    if (o.getFaction () != myFac)
	    if (o.getFaction () != FACTION_NEUTRAL)
	    {
		targets.push_back (&o);
	    }
        }
        for (auto& o : world->getMinions()) {
	    if (o.getFaction () != myFac)
	    if (o.getFaction () != FACTION_NEUTRAL)
	    {
        	targets.push_back (&o);
	    }
        }

        const LivingUnit* choosedTarget = nullptr;
	vector<const LivingUnit*> targetsInRange;

	for (auto& t : targets) {
	    if (self->getDistanceTo (*t) <= self->getCastRange()) {
		targetsInRange.push_back (t);
	    }
	}

	if (targetsInRange.size () == 0) return nullptr;

	sort (targetsInRange.begin (), targetsInRange.end (), [] (auto& t1, auto& t2) {
	    return t1->getLife () < t2->getLife ();
	});

	return targetsInRange [0];
    }

};
