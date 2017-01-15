#include "RemoteProcessClient.h"

#include <algorithm>
#include <cstdlib>

using namespace model;
using namespace std;

const bool LITTLE_ENDIAN_BYTE_ORDER = true;
const int INTEGER_SIZE_BYTES = sizeof(int);
const int LONG_SIZE_BYTES = sizeof(long long);

RemoteProcessClient::RemoteProcessClient(string host, int port)
    : cachedBoolFlag(false), cachedBoolValue(false), previousTrees(vector<Tree>()) {
    socket.Initialize();
    socket.DisableNagleAlgoritm();

    if (!socket.Open(reinterpret_cast<const uint8*>(host.c_str()), static_cast<int16>(port))) {
        exit(10001);
    }
}

void RemoteProcessClient::writeTokenMessage(const string& token) {
    writeEnum<MessageType>(AUTHENTICATION_TOKEN);
    writeString(token);
}

void RemoteProcessClient::writeProtocolVersionMessage() {
    writeEnum<MessageType>(PROTOCOL_VERSION);
    writeInt(1);
}

int RemoteProcessClient::readTeamSizeMessage() {
    ensureMessageType(readEnum<MessageType>(), TEAM_SIZE);
    return readInt();
}

Game RemoteProcessClient::readGameContextMessage() {
    ensureMessageType(readEnum<MessageType>(), GAME_CONTEXT);
    return readGame();
}

PlayerContext* RemoteProcessClient::readPlayerContextMessage() {
    MessageType messageType = readEnum<MessageType>();
    if (messageType == GAME_OVER) {
        return nullptr;
    }

    ensureMessageType(messageType, PLAYER_CONTEXT);

    if (!readBoolean()) {
        return nullptr;
    }

    cachedBoolFlag = true;
    cachedBoolValue = true;

    PlayerContext* playerContext = new PlayerContext;
    *playerContext = readPlayerContext();
    return playerContext;
}

void RemoteProcessClient::writeMovesMessage(const vector<Move>& moves) {
    writeEnum<MessageType>(MOVES_MESSAGE);
    writeMoves(moves);
}

void RemoteProcessClient::close() {
    socket.Close();
}

Bonus RemoteProcessClient::readBonus() {
    if (!readBoolean()) {
        exit(20001);
    }

    long long id = readLong();
    double x = readDouble();
    double y = readDouble();
    double speedX = readDouble();
    double speedY = readDouble();
    double angle = readDouble();
    Faction faction = readEnum<Faction>();
    double radius = readDouble();
    BonusType type = readEnum<BonusType>();

    return Bonus(id, x, y, speedX, speedY, angle, faction, radius, type);
}

void RemoteProcessClient::writeBonus(const Bonus& bonus) {
    writeBoolean(true);

    writeLong(bonus.getId());
    writeDouble(bonus.getX());
    writeDouble(bonus.getY());
    writeDouble(bonus.getSpeedX());
    writeDouble(bonus.getSpeedY());
    writeDouble(bonus.getAngle());
    writeEnum<Faction>(bonus.getFaction());
    writeDouble(bonus.getRadius());
    writeEnum<BonusType>(bonus.getType());
}

vector<Bonus> RemoteProcessClient::readBonuses() {
    int bonusCount = readInt();
    if (bonusCount < 0) {
        exit(20002);
    }

    vector<Bonus> bonuses;
    bonuses.reserve(bonusCount);

    for (int bonusIndex = 0; bonusIndex < bonusCount; ++bonusIndex) {
        bonuses.push_back(readBonus());
    }

    return bonuses;
}

void RemoteProcessClient::writeBonuses(const vector<Bonus>& bonuses) {
    int bonusCount = bonuses.size();
    writeInt(bonusCount);

    for (int bonusIndex = 0; bonusIndex < bonusCount; ++bonusIndex) {
        writeBonus(bonuses[bonusIndex]);
    }
}

Building RemoteProcessClient::readBuilding() {
    if (!readBoolean()) {
        exit(20003);
    }

    long long id = readLong();
    double x = readDouble();
    double y = readDouble();
    double speedX = readDouble();
    double speedY = readDouble();
    double angle = readDouble();
    Faction faction = readEnum<Faction>();
    double radius = readDouble();
    int life = readInt();
    int maxLife = readInt();
    vector<Status> statuses = readStatuses();
    BuildingType type = readEnum<BuildingType>();
    double visionRange = readDouble();
    double attackRange = readDouble();
    int damage = readInt();
    int cooldownTicks = readInt();
    int remainingActionCooldownTicks = readInt();

    return Building(id, x, y, speedX, speedY, angle, faction, radius, life, maxLife, statuses, type, visionRange,
        attackRange, damage, cooldownTicks, remainingActionCooldownTicks);
}

void RemoteProcessClient::writeBuilding(const Building& building) {
    writeBoolean(true);

    writeLong(building.getId());
    writeDouble(building.getX());
    writeDouble(building.getY());
    writeDouble(building.getSpeedX());
    writeDouble(building.getSpeedY());
    writeDouble(building.getAngle());
    writeEnum<Faction>(building.getFaction());
    writeDouble(building.getRadius());
    writeInt(building.getLife());
    writeInt(building.getMaxLife());
    writeStatuses(building.getStatuses());
    writeEnum<BuildingType>(building.getType());
    writeDouble(building.getVisionRange());
    writeDouble(building.getAttackRange());
    writeInt(building.getDamage());
    writeInt(building.getCooldownTicks());
    writeInt(building.getRemainingActionCooldownTicks());
}

vector<Building> RemoteProcessClient::readBuildings() {
    int buildingCount = readInt();
    if (buildingCount < 0) {
        exit(20004);
    }

    vector<Building> buildings;
    buildings.reserve(buildingCount);

    for (int buildingIndex = 0; buildingIndex < buildingCount; ++buildingIndex) {
        buildings.push_back(readBuilding());
    }

    return buildings;
}

void RemoteProcessClient::writeBuildings(const vector<Building>& buildings) {
    int buildingCount = buildings.size();
    writeInt(buildingCount);

    for (int buildingIndex = 0; buildingIndex < buildingCount; ++buildingIndex) {
        writeBuilding(buildings[buildingIndex]);
    }
}

Game RemoteProcessClient::readGame() {
    if (!readBoolean()) {
        exit(20005);
    }

    long long randomSeed = readLong();
    int tickCount = readInt();
    double mapSize = readDouble();
    bool skillsEnabled = readBoolean();
    bool rawMessagesEnabled = readBoolean();
    double friendlyFireDamageFactor = readDouble();
    double buildingDamageScoreFactor = readDouble();
    double buildingEliminationScoreFactor = readDouble();
    double minionDamageScoreFactor = readDouble();
    double minionEliminationScoreFactor = readDouble();
    double wizardDamageScoreFactor = readDouble();
    double wizardEliminationScoreFactor = readDouble();
    double teamWorkingScoreFactor = readDouble();
    int victoryScore = readInt();
    double scoreGainRange = readDouble();
    int rawMessageMaxLength = readInt();
    double rawMessageTransmissionSpeed = readDouble();
    double wizardRadius = readDouble();
    double wizardCastRange = readDouble();
    double wizardVisionRange = readDouble();
    double wizardForwardSpeed = readDouble();
    double wizardBackwardSpeed = readDouble();
    double wizardStrafeSpeed = readDouble();
    int wizardBaseLife = readInt();
    int wizardLifeGrowthPerLevel = readInt();
    int wizardBaseMana = readInt();
    int wizardManaGrowthPerLevel = readInt();
    double wizardBaseLifeRegeneration = readDouble();
    double wizardLifeRegenerationGrowthPerLevel = readDouble();
    double wizardBaseManaRegeneration = readDouble();
    double wizardManaRegenerationGrowthPerLevel = readDouble();
    double wizardMaxTurnAngle = readDouble();
    int wizardMaxResurrectionDelayTicks = readInt();
    int wizardMinResurrectionDelayTicks = readInt();
    int wizardActionCooldownTicks = readInt();
    int staffCooldownTicks = readInt();
    int magicMissileCooldownTicks = readInt();
    int frostBoltCooldownTicks = readInt();
    int fireballCooldownTicks = readInt();
    int hasteCooldownTicks = readInt();
    int shieldCooldownTicks = readInt();
    int magicMissileManacost = readInt();
    int frostBoltManacost = readInt();
    int fireballManacost = readInt();
    int hasteManacost = readInt();
    int shieldManacost = readInt();
    int staffDamage = readInt();
    double staffSector = readDouble();
    double staffRange = readDouble();
    vector<int> levelUpXpValues = readIntArray();
    double minionRadius = readDouble();
    double minionVisionRange = readDouble();
    double minionSpeed = readDouble();
    double minionMaxTurnAngle = readDouble();
    int minionLife = readInt();
    int factionMinionAppearanceIntervalTicks = readInt();
    int orcWoodcutterActionCooldownTicks = readInt();
    int orcWoodcutterDamage = readInt();
    double orcWoodcutterAttackSector = readDouble();
    double orcWoodcutterAttackRange = readDouble();
    int fetishBlowdartActionCooldownTicks = readInt();
    double fetishBlowdartAttackRange = readDouble();
    double fetishBlowdartAttackSector = readDouble();
    double bonusRadius = readDouble();
    int bonusAppearanceIntervalTicks = readInt();
    int bonusScoreAmount = readInt();
    double dartRadius = readDouble();
    double dartSpeed = readDouble();
    int dartDirectDamage = readInt();
    double magicMissileRadius = readDouble();
    double magicMissileSpeed = readDouble();
    int magicMissileDirectDamage = readInt();
    double frostBoltRadius = readDouble();
    double frostBoltSpeed = readDouble();
    int frostBoltDirectDamage = readInt();
    double fireballRadius = readDouble();
    double fireballSpeed = readDouble();
    double fireballExplosionMaxDamageRange = readDouble();
    double fireballExplosionMinDamageRange = readDouble();
    int fireballExplosionMaxDamage = readInt();
    int fireballExplosionMinDamage = readInt();
    double guardianTowerRadius = readDouble();
    double guardianTowerVisionRange = readDouble();
    double guardianTowerLife = readDouble();
    double guardianTowerAttackRange = readDouble();
    int guardianTowerDamage = readInt();
    int guardianTowerCooldownTicks = readInt();
    double factionBaseRadius = readDouble();
    double factionBaseVisionRange = readDouble();
    double factionBaseLife = readDouble();
    double factionBaseAttackRange = readDouble();
    int factionBaseDamage = readInt();
    int factionBaseCooldownTicks = readInt();
    int burningDurationTicks = readInt();
    int burningSummaryDamage = readInt();
    int empoweredDurationTicks = readInt();
    double empoweredDamageFactor = readDouble();
    int frozenDurationTicks = readInt();
    int hastenedDurationTicks = readInt();
    double hastenedBonusDurationFactor = readDouble();
    double hastenedMovementBonusFactor = readDouble();
    double hastenedRotationBonusFactor = readDouble();
    int shieldedDurationTicks = readInt();
    double shieldedBonusDurationFactor = readDouble();
    double shieldedDirectDamageAbsorptionFactor = readDouble();
    double auraSkillRange = readDouble();
    double rangeBonusPerSkillLevel = readDouble();
    int magicalDamageBonusPerSkillLevel = readInt();
    int staffDamageBonusPerSkillLevel = readInt();
    double movementBonusFactorPerSkillLevel = readDouble();
    int magicalDamageAbsorptionPerSkillLevel = readInt();

    return Game(randomSeed, tickCount, mapSize, skillsEnabled, rawMessagesEnabled, friendlyFireDamageFactor,
        buildingDamageScoreFactor, buildingEliminationScoreFactor, minionDamageScoreFactor,
        minionEliminationScoreFactor, wizardDamageScoreFactor, wizardEliminationScoreFactor, teamWorkingScoreFactor,
        victoryScore, scoreGainRange, rawMessageMaxLength, rawMessageTransmissionSpeed, wizardRadius, wizardCastRange,
        wizardVisionRange, wizardForwardSpeed, wizardBackwardSpeed, wizardStrafeSpeed, wizardBaseLife,
        wizardLifeGrowthPerLevel, wizardBaseMana, wizardManaGrowthPerLevel, wizardBaseLifeRegeneration,
        wizardLifeRegenerationGrowthPerLevel, wizardBaseManaRegeneration, wizardManaRegenerationGrowthPerLevel,
        wizardMaxTurnAngle, wizardMaxResurrectionDelayTicks, wizardMinResurrectionDelayTicks, wizardActionCooldownTicks,
        staffCooldownTicks, magicMissileCooldownTicks, frostBoltCooldownTicks, fireballCooldownTicks,
        hasteCooldownTicks, shieldCooldownTicks, magicMissileManacost, frostBoltManacost, fireballManacost,
        hasteManacost, shieldManacost, staffDamage, staffSector, staffRange, levelUpXpValues, minionRadius,
        minionVisionRange, minionSpeed, minionMaxTurnAngle, minionLife, factionMinionAppearanceIntervalTicks,
        orcWoodcutterActionCooldownTicks, orcWoodcutterDamage, orcWoodcutterAttackSector, orcWoodcutterAttackRange,
        fetishBlowdartActionCooldownTicks, fetishBlowdartAttackRange, fetishBlowdartAttackSector, bonusRadius,
        bonusAppearanceIntervalTicks, bonusScoreAmount, dartRadius, dartSpeed, dartDirectDamage, magicMissileRadius,
        magicMissileSpeed, magicMissileDirectDamage, frostBoltRadius, frostBoltSpeed, frostBoltDirectDamage,
        fireballRadius, fireballSpeed, fireballExplosionMaxDamageRange, fireballExplosionMinDamageRange,
        fireballExplosionMaxDamage, fireballExplosionMinDamage, guardianTowerRadius, guardianTowerVisionRange,
        guardianTowerLife, guardianTowerAttackRange, guardianTowerDamage, guardianTowerCooldownTicks, factionBaseRadius,
        factionBaseVisionRange, factionBaseLife, factionBaseAttackRange, factionBaseDamage, factionBaseCooldownTicks,
        burningDurationTicks, burningSummaryDamage, empoweredDurationTicks, empoweredDamageFactor, frozenDurationTicks,
        hastenedDurationTicks, hastenedBonusDurationFactor, hastenedMovementBonusFactor, hastenedRotationBonusFactor,
        shieldedDurationTicks, shieldedBonusDurationFactor, shieldedDirectDamageAbsorptionFactor, auraSkillRange,
        rangeBonusPerSkillLevel, magicalDamageBonusPerSkillLevel, staffDamageBonusPerSkillLevel,
        movementBonusFactorPerSkillLevel, magicalDamageAbsorptionPerSkillLevel);
}

void RemoteProcessClient::writeGame(const Game& game) {
    writeBoolean(true);

    writeLong(game.getRandomSeed());
    writeInt(game.getTickCount());
    writeDouble(game.getMapSize());
    writeBoolean(game.isSkillsEnabled());
    writeBoolean(game.isRawMessagesEnabled());
    writeDouble(game.getFriendlyFireDamageFactor());
    writeDouble(game.getBuildingDamageScoreFactor());
    writeDouble(game.getBuildingEliminationScoreFactor());
    writeDouble(game.getMinionDamageScoreFactor());
    writeDouble(game.getMinionEliminationScoreFactor());
    writeDouble(game.getWizardDamageScoreFactor());
    writeDouble(game.getWizardEliminationScoreFactor());
    writeDouble(game.getTeamWorkingScoreFactor());
    writeInt(game.getVictoryScore());
    writeDouble(game.getScoreGainRange());
    writeInt(game.getRawMessageMaxLength());
    writeDouble(game.getRawMessageTransmissionSpeed());
    writeDouble(game.getWizardRadius());
    writeDouble(game.getWizardCastRange());
    writeDouble(game.getWizardVisionRange());
    writeDouble(game.getWizardForwardSpeed());
    writeDouble(game.getWizardBackwardSpeed());
    writeDouble(game.getWizardStrafeSpeed());
    writeInt(game.getWizardBaseLife());
    writeInt(game.getWizardLifeGrowthPerLevel());
    writeInt(game.getWizardBaseMana());
    writeInt(game.getWizardManaGrowthPerLevel());
    writeDouble(game.getWizardBaseLifeRegeneration());
    writeDouble(game.getWizardLifeRegenerationGrowthPerLevel());
    writeDouble(game.getWizardBaseManaRegeneration());
    writeDouble(game.getWizardManaRegenerationGrowthPerLevel());
    writeDouble(game.getWizardMaxTurnAngle());
    writeInt(game.getWizardMaxResurrectionDelayTicks());
    writeInt(game.getWizardMinResurrectionDelayTicks());
    writeInt(game.getWizardActionCooldownTicks());
    writeInt(game.getStaffCooldownTicks());
    writeInt(game.getMagicMissileCooldownTicks());
    writeInt(game.getFrostBoltCooldownTicks());
    writeInt(game.getFireballCooldownTicks());
    writeInt(game.getHasteCooldownTicks());
    writeInt(game.getShieldCooldownTicks());
    writeInt(game.getMagicMissileManacost());
    writeInt(game.getFrostBoltManacost());
    writeInt(game.getFireballManacost());
    writeInt(game.getHasteManacost());
    writeInt(game.getShieldManacost());
    writeInt(game.getStaffDamage());
    writeDouble(game.getStaffSector());
    writeDouble(game.getStaffRange());
    writeIntArray(game.getLevelUpXpValues());
    writeDouble(game.getMinionRadius());
    writeDouble(game.getMinionVisionRange());
    writeDouble(game.getMinionSpeed());
    writeDouble(game.getMinionMaxTurnAngle());
    writeInt(game.getMinionLife());
    writeInt(game.getFactionMinionAppearanceIntervalTicks());
    writeInt(game.getOrcWoodcutterActionCooldownTicks());
    writeInt(game.getOrcWoodcutterDamage());
    writeDouble(game.getOrcWoodcutterAttackSector());
    writeDouble(game.getOrcWoodcutterAttackRange());
    writeInt(game.getFetishBlowdartActionCooldownTicks());
    writeDouble(game.getFetishBlowdartAttackRange());
    writeDouble(game.getFetishBlowdartAttackSector());
    writeDouble(game.getBonusRadius());
    writeInt(game.getBonusAppearanceIntervalTicks());
    writeInt(game.getBonusScoreAmount());
    writeDouble(game.getDartRadius());
    writeDouble(game.getDartSpeed());
    writeInt(game.getDartDirectDamage());
    writeDouble(game.getMagicMissileRadius());
    writeDouble(game.getMagicMissileSpeed());
    writeInt(game.getMagicMissileDirectDamage());
    writeDouble(game.getFrostBoltRadius());
    writeDouble(game.getFrostBoltSpeed());
    writeInt(game.getFrostBoltDirectDamage());
    writeDouble(game.getFireballRadius());
    writeDouble(game.getFireballSpeed());
    writeDouble(game.getFireballExplosionMaxDamageRange());
    writeDouble(game.getFireballExplosionMinDamageRange());
    writeInt(game.getFireballExplosionMaxDamage());
    writeInt(game.getFireballExplosionMinDamage());
    writeDouble(game.getGuardianTowerRadius());
    writeDouble(game.getGuardianTowerVisionRange());
    writeDouble(game.getGuardianTowerLife());
    writeDouble(game.getGuardianTowerAttackRange());
    writeInt(game.getGuardianTowerDamage());
    writeInt(game.getGuardianTowerCooldownTicks());
    writeDouble(game.getFactionBaseRadius());
    writeDouble(game.getFactionBaseVisionRange());
    writeDouble(game.getFactionBaseLife());
    writeDouble(game.getFactionBaseAttackRange());
    writeInt(game.getFactionBaseDamage());
    writeInt(game.getFactionBaseCooldownTicks());
    writeInt(game.getBurningDurationTicks());
    writeInt(game.getBurningSummaryDamage());
    writeInt(game.getEmpoweredDurationTicks());
    writeDouble(game.getEmpoweredDamageFactor());
    writeInt(game.getFrozenDurationTicks());
    writeInt(game.getHastenedDurationTicks());
    writeDouble(game.getHastenedBonusDurationFactor());
    writeDouble(game.getHastenedMovementBonusFactor());
    writeDouble(game.getHastenedRotationBonusFactor());
    writeInt(game.getShieldedDurationTicks());
    writeDouble(game.getShieldedBonusDurationFactor());
    writeDouble(game.getShieldedDirectDamageAbsorptionFactor());
    writeDouble(game.getAuraSkillRange());
    writeDouble(game.getRangeBonusPerSkillLevel());
    writeInt(game.getMagicalDamageBonusPerSkillLevel());
    writeInt(game.getStaffDamageBonusPerSkillLevel());
    writeDouble(game.getMovementBonusFactorPerSkillLevel());
    writeInt(game.getMagicalDamageAbsorptionPerSkillLevel());
}

vector<Game> RemoteProcessClient::readGames() {
    int gameCount = readInt();
    if (gameCount < 0) {
        exit(20006);
    }

    vector<Game> games;
    games.reserve(gameCount);

    for (int gameIndex = 0; gameIndex < gameCount; ++gameIndex) {
        games.push_back(readGame());
    }

    return games;
}

void RemoteProcessClient::writeGames(const vector<Game>& games) {
    int gameCount = games.size();
    writeInt(gameCount);

    for (int gameIndex = 0; gameIndex < gameCount; ++gameIndex) {
        writeGame(games[gameIndex]);
    }
}

Message RemoteProcessClient::readMessage() {
    if (!readBoolean()) {
        exit(20007);
    }

    LaneType lane = readEnum<LaneType>();
    SkillType skillToLearn = readEnum<SkillType>();
    vector<signed char> rawMessage = readByteArray();

    return Message(lane, skillToLearn, rawMessage);
}

void RemoteProcessClient::writeMessage(const Message& message) {
    writeBoolean(true);

    writeEnum<LaneType>(message.getLane());
    writeEnum<SkillType>(message.getSkillToLearn());
    writeByteArray(message.getRawMessage());
}

vector<Message> RemoteProcessClient::readMessages() {
    int messageCount = readInt();
    if (messageCount < 0) {
        exit(20008);
    }

    vector<Message> messages;
    messages.reserve(messageCount);

    for (int messageIndex = 0; messageIndex < messageCount; ++messageIndex) {
        messages.push_back(readMessage());
    }

    return messages;
}

void RemoteProcessClient::writeMessages(const vector<Message>& messages) {
    int messageCount = messages.size();
    writeInt(messageCount);

    for (int messageIndex = 0; messageIndex < messageCount; ++messageIndex) {
        writeMessage(messages[messageIndex]);
    }
}

Minion RemoteProcessClient::readMinion() {
    if (!readBoolean()) {
        exit(20009);
    }

    long long id = readLong();
    double x = readDouble();
    double y = readDouble();
    double speedX = readDouble();
    double speedY = readDouble();
    double angle = readDouble();
    Faction faction = readEnum<Faction>();
    double radius = readDouble();
    int life = readInt();
    int maxLife = readInt();
    vector<Status> statuses = readStatuses();
    MinionType type = readEnum<MinionType>();
    double visionRange = readDouble();
    int damage = readInt();
    int cooldownTicks = readInt();
    int remainingActionCooldownTicks = readInt();

    return Minion(id, x, y, speedX, speedY, angle, faction, radius, life, maxLife, statuses, type, visionRange, damage,
        cooldownTicks, remainingActionCooldownTicks);
}

void RemoteProcessClient::writeMinion(const Minion& minion) {
    writeBoolean(true);

    writeLong(minion.getId());
    writeDouble(minion.getX());
    writeDouble(minion.getY());
    writeDouble(minion.getSpeedX());
    writeDouble(minion.getSpeedY());
    writeDouble(minion.getAngle());
    writeEnum<Faction>(minion.getFaction());
    writeDouble(minion.getRadius());
    writeInt(minion.getLife());
    writeInt(minion.getMaxLife());
    writeStatuses(minion.getStatuses());
    writeEnum<MinionType>(minion.getType());
    writeDouble(minion.getVisionRange());
    writeInt(minion.getDamage());
    writeInt(minion.getCooldownTicks());
    writeInt(minion.getRemainingActionCooldownTicks());
}

vector<Minion> RemoteProcessClient::readMinions() {
    int minionCount = readInt();
    if (minionCount < 0) {
        exit(20010);
    }

    vector<Minion> minions;
    minions.reserve(minionCount);

    for (int minionIndex = 0; minionIndex < minionCount; ++minionIndex) {
        minions.push_back(readMinion());
    }

    return minions;
}

void RemoteProcessClient::writeMinions(const vector<Minion>& minions) {
    int minionCount = minions.size();
    writeInt(minionCount);

    for (int minionIndex = 0; minionIndex < minionCount; ++minionIndex) {
        writeMinion(minions[minionIndex]);
    }
}

void RemoteProcessClient::writeMove(const Move& move) {
    writeBoolean(true);

    writeDouble(move.getSpeed());
    writeDouble(move.getStrafeSpeed());
    writeDouble(move.getTurn());
    writeEnum(move.getAction());
    writeDouble(move.getCastAngle());
    writeDouble(move.getMinCastDistance());
    writeDouble(move.getMaxCastDistance());
    writeLong(move.getStatusTargetId());
    writeEnum(move.getSkillToLearn());
    writeMessages(move.getMessages());
}

void RemoteProcessClient::writeMoves(const vector<Move>& moves) {
    int moveCount = moves.size();
    writeInt(moveCount);

    for (int moveIndex = 0; moveIndex < moveCount; ++moveIndex) {
        writeMove(moves[moveIndex]);
    }
}

Player RemoteProcessClient::readPlayer() {
    if (!readBoolean()) {
        exit(20013);
    }

    long long id = readLong();
    bool me = readBoolean();
    string name = readString();
    bool strategyCrashed = readBoolean();
    int score = readInt();
    Faction faction = readEnum<Faction>();

    return Player(id, me, name, strategyCrashed, score, faction);
}

void RemoteProcessClient::writePlayer(const Player& player) {
    writeBoolean(true);

    writeLong(player.getId());
    writeBoolean(player.isMe());
    writeString(player.getName());
    writeBoolean(player.isStrategyCrashed());
    writeInt(player.getScore());
    writeEnum<Faction>(player.getFaction());
}

vector<Player> RemoteProcessClient::readPlayers() {
    int playerCount = readInt();
    if (playerCount < 0) {
        exit(20014);
    }

    vector<Player> players;
    players.reserve(playerCount);

    for (int playerIndex = 0; playerIndex < playerCount; ++playerIndex) {
        players.push_back(readPlayer());
    }

    return players;
}

void RemoteProcessClient::writePlayers(const vector<Player>& players) {
    int playerCount = players.size();
    writeInt(playerCount);

    for (int playerIndex = 0; playerIndex < playerCount; ++playerIndex) {
        writePlayer(players[playerIndex]);
    }
}

PlayerContext RemoteProcessClient::readPlayerContext() {
    if (!readBoolean()) {
        exit(20015);
    }

    vector<Wizard> wizards = readWizards();
    World world = readWorld();

    return PlayerContext(wizards, world);
}

void RemoteProcessClient::writePlayerContext(const PlayerContext& playerContext) {
    writeBoolean(true);

    writeWizards(playerContext.getWizards());
    writeWorld(playerContext.getWorld());
}

vector<PlayerContext> RemoteProcessClient::readPlayerContexts() {
    int playerContextCount = readInt();
    if (playerContextCount < 0) {
        exit(20016);
    }

    vector<PlayerContext> playerContexts;
    playerContexts.reserve(playerContextCount);

    for (int playerContextIndex = 0; playerContextIndex < playerContextCount; ++playerContextIndex) {
        playerContexts.push_back(readPlayerContext());
    }

    return playerContexts;
}

void RemoteProcessClient::writePlayerContexts(const vector<PlayerContext>& playerContexts) {
    int playerContextCount = playerContexts.size();
    writeInt(playerContextCount);

    for (int playerContextIndex = 0; playerContextIndex < playerContextCount; ++playerContextIndex) {
        writePlayerContext(playerContexts[playerContextIndex]);
    }
}

Projectile RemoteProcessClient::readProjectile() {
    if (!readBoolean()) {
        exit(20017);
    }

    long long id = readLong();
    double x = readDouble();
    double y = readDouble();
    double speedX = readDouble();
    double speedY = readDouble();
    double angle = readDouble();
    Faction faction = readEnum<Faction>();
    double radius = readDouble();
    ProjectileType type = readEnum<ProjectileType>();
    long long ownerUnitId = readLong();
    long long ownerPlayerId = readLong();

    return Projectile(id, x, y, speedX, speedY, angle, faction, radius, type, ownerUnitId, ownerPlayerId);
}

void RemoteProcessClient::writeProjectile(const Projectile& projectile) {
    writeBoolean(true);

    writeLong(projectile.getId());
    writeDouble(projectile.getX());
    writeDouble(projectile.getY());
    writeDouble(projectile.getSpeedX());
    writeDouble(projectile.getSpeedY());
    writeDouble(projectile.getAngle());
    writeEnum<Faction>(projectile.getFaction());
    writeDouble(projectile.getRadius());
    writeEnum<ProjectileType>(projectile.getType());
    writeLong(projectile.getOwnerUnitId());
    writeLong(projectile.getOwnerPlayerId());
}

vector<Projectile> RemoteProcessClient::readProjectiles() {
    int projectileCount = readInt();
    if (projectileCount < 0) {
        exit(20018);
    }

    vector<Projectile> projectiles;
    projectiles.reserve(projectileCount);

    for (int projectileIndex = 0; projectileIndex < projectileCount; ++projectileIndex) {
        projectiles.push_back(readProjectile());
    }

    return projectiles;
}

void RemoteProcessClient::writeProjectiles(const vector<Projectile>& projectiles) {
    int projectileCount = projectiles.size();
    writeInt(projectileCount);

    for (int projectileIndex = 0; projectileIndex < projectileCount; ++projectileIndex) {
        writeProjectile(projectiles[projectileIndex]);
    }
}

Status RemoteProcessClient::readStatus() {
    if (!readBoolean()) {
        exit(20019);
    }

    long long id = readLong();
    StatusType type = readEnum<StatusType>();
    long long wizardId = readLong();
    long long playerId = readLong();
    int remainingDurationTicks = readInt();

    return Status(id, type, wizardId, playerId, remainingDurationTicks);
}

void RemoteProcessClient::writeStatus(const Status& status) {
    writeBoolean(true);

    writeLong(status.getId());
    writeEnum<StatusType>(status.getType());
    writeLong(status.getWizardId());
    writeLong(status.getPlayerId());
    writeInt(status.getRemainingDurationTicks());
}

vector<Status> RemoteProcessClient::readStatuses() {
    int statusCount = readInt();
    if (statusCount < 0) {
        exit(20020);
    }

    vector<Status> statuses;
    statuses.reserve(statusCount);

    for (int statusIndex = 0; statusIndex < statusCount; ++statusIndex) {
        statuses.push_back(readStatus());
    }

    return statuses;
}

void RemoteProcessClient::writeStatuses(const vector<Status>& statuses) {
    int statusCount = statuses.size();
    writeInt(statusCount);

    for (int statusIndex = 0; statusIndex < statusCount; ++statusIndex) {
        writeStatus(statuses[statusIndex]);
    }
}

Tree RemoteProcessClient::readTree() {
    if (!readBoolean()) {
        exit(20021);
    }

    long long id = readLong();
    double x = readDouble();
    double y = readDouble();
    double speedX = readDouble();
    double speedY = readDouble();
    double angle = readDouble();
    Faction faction = readEnum<Faction>();
    double radius = readDouble();
    int life = readInt();
    int maxLife = readInt();
    vector<Status> statuses = readStatuses();

    return Tree(id, x, y, speedX, speedY, angle, faction, radius, life, maxLife, statuses);
}

void RemoteProcessClient::writeTree(const Tree& tree) {
    writeBoolean(true);

    writeLong(tree.getId());
    writeDouble(tree.getX());
    writeDouble(tree.getY());
    writeDouble(tree.getSpeedX());
    writeDouble(tree.getSpeedY());
    writeDouble(tree.getAngle());
    writeEnum<Faction>(tree.getFaction());
    writeDouble(tree.getRadius());
    writeInt(tree.getLife());
    writeInt(tree.getMaxLife());
    writeStatuses(tree.getStatuses());
}

vector<Tree> RemoteProcessClient::readTrees() {
    int treeCount = readInt();
    if (treeCount < 0) {
        return previousTrees;
    }

    vector<Tree> trees;
    trees.reserve(treeCount);

    for (int treeIndex = 0; treeIndex < treeCount; ++treeIndex) {
        trees.push_back(readTree());
    }

    previousTrees = trees;
    return trees;
}

void RemoteProcessClient::writeTrees(const vector<Tree>& trees) {
    int treeCount = trees.size();
    writeInt(treeCount);

    for (int treeIndex = 0; treeIndex < treeCount; ++treeIndex) {
        writeTree(trees[treeIndex]);
    }
}

Wizard RemoteProcessClient::readWizard() {
    if (!readBoolean()) {
        exit(20023);
    }

    long long id = readLong();
    double x = readDouble();
    double y = readDouble();
    double speedX = readDouble();
    double speedY = readDouble();
    double angle = readDouble();
    Faction faction = readEnum<Faction>();
    double radius = readDouble();
    int life = readInt();
    int maxLife = readInt();
    vector<Status> statuses = readStatuses();
    long long ownerPlayerId = readLong();
    bool me = readBoolean();
    int mana = readInt();
    int maxMana = readInt();
    double visionRange = readDouble();
    double castRange = readDouble();
    int xp = readInt();
    int level = readInt();
    vector<SkillType> skills = readEnumArray<SkillType>();
    int remainingActionCooldownTicks = readInt();
    vector<int> remainingCooldownTicksByAction = readIntArray();
    bool master = readBoolean();
    vector<Message> messages = readMessages();

    return Wizard(id, x, y, speedX, speedY, angle, faction, radius, life, maxLife, statuses, ownerPlayerId, me, mana,
        maxMana, visionRange, castRange, xp, level, skills, remainingActionCooldownTicks,
        remainingCooldownTicksByAction, master, messages);
}

void RemoteProcessClient::writeWizard(const Wizard& wizard) {
    writeBoolean(true);

    writeLong(wizard.getId());
    writeDouble(wizard.getX());
    writeDouble(wizard.getY());
    writeDouble(wizard.getSpeedX());
    writeDouble(wizard.getSpeedY());
    writeDouble(wizard.getAngle());
    writeEnum<Faction>(wizard.getFaction());
    writeDouble(wizard.getRadius());
    writeInt(wizard.getLife());
    writeInt(wizard.getMaxLife());
    writeStatuses(wizard.getStatuses());
    writeLong(wizard.getOwnerPlayerId());
    writeBoolean(wizard.isMe());
    writeInt(wizard.getMana());
    writeInt(wizard.getMaxMana());
    writeDouble(wizard.getVisionRange());
    writeDouble(wizard.getCastRange());
    writeInt(wizard.getXp());
    writeInt(wizard.getLevel());
    writeEnumArray<SkillType>(wizard.getSkills());
    writeInt(wizard.getRemainingActionCooldownTicks());
    writeIntArray(wizard.getRemainingCooldownTicksByAction());
    writeBoolean(wizard.isMaster());
    writeMessages(wizard.getMessages());
}

vector<Wizard> RemoteProcessClient::readWizards() {
    int wizardCount = readInt();
    if (wizardCount < 0) {
        exit(20024);
    }

    vector<Wizard> wizards;
    wizards.reserve(wizardCount);

    for (int wizardIndex = 0; wizardIndex < wizardCount; ++wizardIndex) {
        wizards.push_back(readWizard());
    }

    return wizards;
}

void RemoteProcessClient::writeWizards(const vector<Wizard>& wizards) {
    int wizardCount = wizards.size();
    writeInt(wizardCount);

    for (int wizardIndex = 0; wizardIndex < wizardCount; ++wizardIndex) {
        writeWizard(wizards[wizardIndex]);
    }
}

World RemoteProcessClient::readWorld() {
    if (!readBoolean()) {
        exit(20025);
    }

    int tickIndex = readInt();
    int tickCount = readInt();
    double width = readDouble();
    double height = readDouble();
    vector<Player> players = readPlayers();
    vector<Wizard> wizards = readWizards();
    vector<Minion> minions = readMinions();
    vector<Projectile> projectiles = readProjectiles();
    vector<Bonus> bonuses = readBonuses();
    vector<Building> buildings = readBuildings();
    vector<Tree> trees = readTrees();

    return World(tickIndex, tickCount, width, height, players, wizards, minions, projectiles, bonuses, buildings,
        trees);
}

void RemoteProcessClient::writeWorld(const World& world) {
    writeBoolean(true);

    writeInt(world.getTickIndex());
    writeInt(world.getTickCount());
    writeDouble(world.getWidth());
    writeDouble(world.getHeight());
    writePlayers(world.getPlayers());
    writeWizards(world.getWizards());
    writeMinions(world.getMinions());
    writeProjectiles(world.getProjectiles());
    writeBonuses(world.getBonuses());
    writeBuildings(world.getBuildings());
    writeTrees(world.getTrees());
}

vector<World> RemoteProcessClient::readWorlds() {
    int worldCount = readInt();
    if (worldCount < 0) {
        exit(20026);
    }

    vector<World> worlds;
    worlds.reserve(worldCount);

    for (int worldIndex = 0; worldIndex < worldCount; ++worldIndex) {
        worlds.push_back(readWorld());
    }

    return worlds;
}

void RemoteProcessClient::writeWorlds(const vector<World>& worlds) {
    int worldCount = worlds.size();
    writeInt(worldCount);

    for (int worldIndex = 0; worldIndex < worldCount; ++worldIndex) {
        writeWorld(worlds[worldIndex]);
    }
}

void RemoteProcessClient::ensureMessageType(MessageType actualType, MessageType expectedType) {
    if (actualType != expectedType) {
        exit(10011);
    }
}

vector<signed char> RemoteProcessClient::readByteArray() {
    int count = readInt();
    if (count <= 0) {
        return vector<signed char>();
    }

    return readBytes(count);
}

void RemoteProcessClient::writeByteArray(const vector<signed char>& value) {
    writeInt(value.size());
    writeBytes(value);
}

template <typename E> E RemoteProcessClient::readEnum() {
    return static_cast<E>(this->readBytes(1)[0]);
}

template <typename E> vector<E> RemoteProcessClient::readEnumArray() {
    int length = readInt();
    if (length < 0) {
        exit(10015);
    }

    vector<E> value;
    value.reserve(length);

    for (int i = 0; i < length; ++i) {
        value.push_back(readEnum<E>());
    }

    return value;
}

template <typename E> vector<vector<E> > RemoteProcessClient::readEnumArray2D() {
    int length = readInt();
    if (length < 0) {
        exit(10016);
    }

    vector<vector<E> > value(length);

    for (int i = 0; i < length; ++i) {
        int length2 = readInt();
        if (length2 < 0) {
            exit(10017);
        }

        vector<E>& value2 = value[i];
        value2.reserve(length2);

        for (int j = 0; j < length2; ++j) {
            value2.push_back(readEnum<E>());
        }
    }

    return value;
}

template <typename E> void RemoteProcessClient::writeEnum(E value) {
    vector<signed char> bytes;
    bytes.push_back(static_cast<signed char>(value));
    this->writeBytes(bytes);
}

template <typename E> void RemoteProcessClient::writeEnumArray(const vector<E>& value) {
    int length = value.size();
    writeInt(length);

    for (int i = 0; i < length; ++i) {
        writeEnum<E>(value[i]);
    }
}

template <typename E> void RemoteProcessClient::writeEnumArray2D(const vector<vector<E> >& value) {
    int length = value.size();
    writeInt(length);

    for (int i = 0; i < length; ++i) {
        writeEnumArray<E>(value[i]);
    }
}

string RemoteProcessClient::readString() {
    int length = this->readInt();
    if (length == -1) {
        exit(10014);
    }

    if (length == 0) {
        return "";
    }

    vector<signed char> bytes = this->readBytes(length);
    return string(reinterpret_cast<char*>(&bytes[0]), length);
}

void RemoteProcessClient::writeString(const string& value) {
    size_t length = value.size();
    if (length == 0) {
        this->writeInt(0);
        return;
    }

    vector<signed char> bytes(length);

    memcpy(&bytes[0], value.c_str(), length);

    this->writeInt(static_cast<int>(bytes.size()));
    this->writeBytes(bytes);
}

bool RemoteProcessClient::readBoolean() {
    if (cachedBoolFlag) {
        cachedBoolFlag = false;
        return cachedBoolValue;
    }
    return this->readBytes(1)[0] != 0;
}

void RemoteProcessClient::writeBoolean(bool value) {
    vector<signed char> bytes;
    bytes.push_back(static_cast<signed char>(value ? 1 : 0));
    this->writeBytes(bytes);
}

int RemoteProcessClient::readInt() {
    vector<signed char> bytes = this->readBytes(INTEGER_SIZE_BYTES);

    if (this->isLittleEndianMachine() != LITTLE_ENDIAN_BYTE_ORDER) {
        reverse(bytes.begin(), bytes.end());
    }

    int value;

    memcpy(&value, &bytes[0], INTEGER_SIZE_BYTES);

    return value;
}

vector<int> RemoteProcessClient::readIntArray() {
    int length = readInt();
    if (length < 0) {
        exit(10018);
    }

    vector<int> value;
    value.reserve(length);

    for (int i = 0; i < length; ++i) {
        value.push_back(readInt());
    }

    return value;
}

vector<vector<int> > RemoteProcessClient::readIntArray2D() {
    int length = readInt();
    if (length < 0) {
        exit(10019);
    }

    vector<vector<int> > value(length);

    for (int i = 0; i < length; ++i) {
        int length2 = readInt();
        if (length2 < 0) {
            exit(10020);
        }

        vector<int>& value2 = value[i];
        value2.reserve(length2);

        for (int j = 0; j < length2; ++j) {
            value2.push_back(readInt());
        }
    }

    return value;
}

void RemoteProcessClient::writeInt(int value) {
    vector<signed char> bytes(INTEGER_SIZE_BYTES);

    memcpy(&bytes[0], &value, INTEGER_SIZE_BYTES);

    if (this->isLittleEndianMachine() != LITTLE_ENDIAN_BYTE_ORDER) {
        reverse(bytes.begin(), bytes.end());
    }

    this->writeBytes(bytes);
}

void RemoteProcessClient::writeIntArray(const vector<int>& value) {
    int length = value.size();
    writeInt(length);

    for (int i = 0; i < length; ++i) {
        writeInt(value[i]);
    }
}

void RemoteProcessClient::writeIntArray2D(const vector<vector<int> >& value) {
    int length = value.size();
    writeInt(length);

    for (int i = 0; i < length; ++i) {
        writeIntArray(value[i]);
    }
}

long long RemoteProcessClient::readLong() {
    vector<signed char> bytes = this->readBytes(LONG_SIZE_BYTES);

    if (this->isLittleEndianMachine() != LITTLE_ENDIAN_BYTE_ORDER) {
        reverse(bytes.begin(), bytes.end());
    }

    long long value;

    memcpy(&value, &bytes[0], LONG_SIZE_BYTES);

    return value;
}

void RemoteProcessClient::writeLong(long long value) {
    vector<signed char> bytes(LONG_SIZE_BYTES);

    memcpy(&bytes[0], &value, LONG_SIZE_BYTES);

    if (this->isLittleEndianMachine() != LITTLE_ENDIAN_BYTE_ORDER) {
        reverse(bytes.begin(), bytes.end());
    }

    this->writeBytes(bytes);
}

double RemoteProcessClient::readDouble() {
    long long value = this->readLong();
    return *reinterpret_cast<double*>(&value);
}

void RemoteProcessClient::writeDouble(double value) {
    this->writeLong(*reinterpret_cast<long long*>(&value));
}

vector<signed char> RemoteProcessClient::readBytes(unsigned int byteCount) {
    vector<signed char> bytes(byteCount);
    unsigned int offset = 0;
    int receivedByteCount;

    while (offset < byteCount && (receivedByteCount = socket.Receive(byteCount - offset)) > 0) {
        memcpy(&bytes[offset], socket.GetData(), receivedByteCount);
        offset += receivedByteCount;
    }

    if (offset != byteCount) {
        exit(10012);
    }

    return bytes;
}

void RemoteProcessClient::writeBytes(const vector<signed char>& bytes) {
    vector<signed char>::size_type byteCount = bytes.size();
    unsigned int offset = 0;
    int sentByteCount;

    while (offset < byteCount && (sentByteCount = socket.Send(reinterpret_cast<const uint8*>(&bytes[offset]), byteCount - offset)) > 0) {
        offset += sentByteCount;
    }

    if (offset != byteCount) {
        exit(10013);
    }
}

bool RemoteProcessClient::isLittleEndianMachine() {
    union {
        uint16 value;
        unsigned char bytes[2];
    } test = {0x0201};

    return test.bytes[0] == 1;
}

RemoteProcessClient::~RemoteProcessClient() {
    this->close();
}
