#include "Game.h"

using namespace model;
using namespace std;

Game::Game()
    : randomSeed(-1), tickCount(-1), mapSize(-1.0), skillsEnabled(false), rawMessagesEnabled(false),
    friendlyFireDamageFactor(-1.0), buildingDamageScoreFactor(-1.0), buildingEliminationScoreFactor(-1.0),
    minionDamageScoreFactor(-1.0), minionEliminationScoreFactor(-1.0), wizardDamageScoreFactor(-1.0),
    wizardEliminationScoreFactor(-1.0), teamWorkingScoreFactor(-1.0), victoryScore(-1), scoreGainRange(-1.0),
    rawMessageMaxLength(-1), rawMessageTransmissionSpeed(-1.0), wizardRadius(-1.0), wizardCastRange(-1.0),
    wizardVisionRange(-1.0), wizardForwardSpeed(-1.0), wizardBackwardSpeed(-1.0), wizardStrafeSpeed(-1.0),
    wizardBaseLife(-1), wizardLifeGrowthPerLevel(-1), wizardBaseMana(-1), wizardManaGrowthPerLevel(-1),
    wizardBaseLifeRegeneration(-1.0), wizardLifeRegenerationGrowthPerLevel(-1.0), wizardBaseManaRegeneration(-1.0),
    wizardManaRegenerationGrowthPerLevel(-1.0), wizardMaxTurnAngle(-1.0), wizardMaxResurrectionDelayTicks(-1),
    wizardMinResurrectionDelayTicks(-1), wizardActionCooldownTicks(-1), staffCooldownTicks(-1),
    magicMissileCooldownTicks(-1), frostBoltCooldownTicks(-1), fireballCooldownTicks(-1), hasteCooldownTicks(-1),
    shieldCooldownTicks(-1), magicMissileManacost(-1), frostBoltManacost(-1), fireballManacost(-1), hasteManacost(-1),
    shieldManacost(-1), staffDamage(-1), staffSector(-1.0), staffRange(-1.0), levelUpXpValues(vector<int>()),
    minionRadius(-1.0), minionVisionRange(-1.0), minionSpeed(-1.0), minionMaxTurnAngle(-1.0), minionLife(-1),
    factionMinionAppearanceIntervalTicks(-1), orcWoodcutterActionCooldownTicks(-1), orcWoodcutterDamage(-1),
    orcWoodcutterAttackSector(-1.0), orcWoodcutterAttackRange(-1.0), fetishBlowdartActionCooldownTicks(-1),
    fetishBlowdartAttackRange(-1.0), fetishBlowdartAttackSector(-1.0), bonusRadius(-1.0),
    bonusAppearanceIntervalTicks(-1), bonusScoreAmount(-1), dartRadius(-1.0), dartSpeed(-1.0), dartDirectDamage(-1),
    magicMissileRadius(-1.0), magicMissileSpeed(-1.0), magicMissileDirectDamage(-1), frostBoltRadius(-1.0),
    frostBoltSpeed(-1.0), frostBoltDirectDamage(-1), fireballRadius(-1.0), fireballSpeed(-1.0),
    fireballExplosionMaxDamageRange(-1.0), fireballExplosionMinDamageRange(-1.0), fireballExplosionMaxDamage(-1),
    fireballExplosionMinDamage(-1), guardianTowerRadius(-1.0), guardianTowerVisionRange(-1.0), guardianTowerLife(-1.0),
    guardianTowerAttackRange(-1.0), guardianTowerDamage(-1), guardianTowerCooldownTicks(-1), factionBaseRadius(-1.0),
    factionBaseVisionRange(-1.0), factionBaseLife(-1.0), factionBaseAttackRange(-1.0), factionBaseDamage(-1),
    factionBaseCooldownTicks(-1), burningDurationTicks(-1), burningSummaryDamage(-1), empoweredDurationTicks(-1),
    empoweredDamageFactor(-1.0), frozenDurationTicks(-1), hastenedDurationTicks(-1), hastenedBonusDurationFactor(-1.0),
    hastenedMovementBonusFactor(-1.0), hastenedRotationBonusFactor(-1.0), shieldedDurationTicks(-1),
    shieldedBonusDurationFactor(-1.0), shieldedDirectDamageAbsorptionFactor(-1.0), auraSkillRange(-1.0),
    rangeBonusPerSkillLevel(-1.0), magicalDamageBonusPerSkillLevel(-1), staffDamageBonusPerSkillLevel(-1),
    movementBonusFactorPerSkillLevel(-1.0), magicalDamageAbsorptionPerSkillLevel(-1) { }

Game::Game(long long randomSeed, int tickCount, double mapSize, bool skillsEnabled, bool rawMessagesEnabled,
    double friendlyFireDamageFactor, double buildingDamageScoreFactor, double buildingEliminationScoreFactor,
    double minionDamageScoreFactor, double minionEliminationScoreFactor, double wizardDamageScoreFactor,
    double wizardEliminationScoreFactor, double teamWorkingScoreFactor, int victoryScore, double scoreGainRange,
    int rawMessageMaxLength, double rawMessageTransmissionSpeed, double wizardRadius, double wizardCastRange,
    double wizardVisionRange, double wizardForwardSpeed, double wizardBackwardSpeed, double wizardStrafeSpeed,
    int wizardBaseLife, int wizardLifeGrowthPerLevel, int wizardBaseMana, int wizardManaGrowthPerLevel,
    double wizardBaseLifeRegeneration, double wizardLifeRegenerationGrowthPerLevel, double wizardBaseManaRegeneration,
    double wizardManaRegenerationGrowthPerLevel, double wizardMaxTurnAngle, int wizardMaxResurrectionDelayTicks,
    int wizardMinResurrectionDelayTicks, int wizardActionCooldownTicks, int staffCooldownTicks,
    int magicMissileCooldownTicks, int frostBoltCooldownTicks, int fireballCooldownTicks, int hasteCooldownTicks,
    int shieldCooldownTicks, int magicMissileManacost, int frostBoltManacost, int fireballManacost, int hasteManacost,
    int shieldManacost, int staffDamage, double staffSector, double staffRange, const vector<int>& levelUpXpValues,
    double minionRadius, double minionVisionRange, double minionSpeed, double minionMaxTurnAngle, int minionLife,
    int factionMinionAppearanceIntervalTicks, int orcWoodcutterActionCooldownTicks, int orcWoodcutterDamage,
    double orcWoodcutterAttackSector, double orcWoodcutterAttackRange, int fetishBlowdartActionCooldownTicks,
    double fetishBlowdartAttackRange, double fetishBlowdartAttackSector, double bonusRadius,
    int bonusAppearanceIntervalTicks, int bonusScoreAmount, double dartRadius, double dartSpeed, int dartDirectDamage,
    double magicMissileRadius, double magicMissileSpeed, int magicMissileDirectDamage, double frostBoltRadius,
    double frostBoltSpeed, int frostBoltDirectDamage, double fireballRadius, double fireballSpeed,
    double fireballExplosionMaxDamageRange, double fireballExplosionMinDamageRange, int fireballExplosionMaxDamage,
    int fireballExplosionMinDamage, double guardianTowerRadius, double guardianTowerVisionRange,
    double guardianTowerLife, double guardianTowerAttackRange, int guardianTowerDamage, int guardianTowerCooldownTicks,
    double factionBaseRadius, double factionBaseVisionRange, double factionBaseLife, double factionBaseAttackRange,
    int factionBaseDamage, int factionBaseCooldownTicks, int burningDurationTicks, int burningSummaryDamage,
    int empoweredDurationTicks, double empoweredDamageFactor, int frozenDurationTicks, int hastenedDurationTicks,
    double hastenedBonusDurationFactor, double hastenedMovementBonusFactor, double hastenedRotationBonusFactor,
    int shieldedDurationTicks, double shieldedBonusDurationFactor, double shieldedDirectDamageAbsorptionFactor,
    double auraSkillRange, double rangeBonusPerSkillLevel, int magicalDamageBonusPerSkillLevel,
    int staffDamageBonusPerSkillLevel, double movementBonusFactorPerSkillLevel,
    int magicalDamageAbsorptionPerSkillLevel)
    : randomSeed(randomSeed), tickCount(tickCount), mapSize(mapSize), skillsEnabled(skillsEnabled),
    rawMessagesEnabled(rawMessagesEnabled), friendlyFireDamageFactor(friendlyFireDamageFactor),
    buildingDamageScoreFactor(buildingDamageScoreFactor),
    buildingEliminationScoreFactor(buildingEliminationScoreFactor), minionDamageScoreFactor(minionDamageScoreFactor),
    minionEliminationScoreFactor(minionEliminationScoreFactor), wizardDamageScoreFactor(wizardDamageScoreFactor),
    wizardEliminationScoreFactor(wizardEliminationScoreFactor), teamWorkingScoreFactor(teamWorkingScoreFactor),
    victoryScore(victoryScore), scoreGainRange(scoreGainRange), rawMessageMaxLength(rawMessageMaxLength),
    rawMessageTransmissionSpeed(rawMessageTransmissionSpeed), wizardRadius(wizardRadius),
    wizardCastRange(wizardCastRange), wizardVisionRange(wizardVisionRange), wizardForwardSpeed(wizardForwardSpeed),
    wizardBackwardSpeed(wizardBackwardSpeed), wizardStrafeSpeed(wizardStrafeSpeed), wizardBaseLife(wizardBaseLife),
    wizardLifeGrowthPerLevel(wizardLifeGrowthPerLevel), wizardBaseMana(wizardBaseMana),
    wizardManaGrowthPerLevel(wizardManaGrowthPerLevel), wizardBaseLifeRegeneration(wizardBaseLifeRegeneration),
    wizardLifeRegenerationGrowthPerLevel(wizardLifeRegenerationGrowthPerLevel),
    wizardBaseManaRegeneration(wizardBaseManaRegeneration),
    wizardManaRegenerationGrowthPerLevel(wizardManaRegenerationGrowthPerLevel), wizardMaxTurnAngle(wizardMaxTurnAngle),
    wizardMaxResurrectionDelayTicks(wizardMaxResurrectionDelayTicks),
    wizardMinResurrectionDelayTicks(wizardMinResurrectionDelayTicks),
    wizardActionCooldownTicks(wizardActionCooldownTicks), staffCooldownTicks(staffCooldownTicks),
    magicMissileCooldownTicks(magicMissileCooldownTicks), frostBoltCooldownTicks(frostBoltCooldownTicks),
    fireballCooldownTicks(fireballCooldownTicks), hasteCooldownTicks(hasteCooldownTicks),
    shieldCooldownTicks(shieldCooldownTicks), magicMissileManacost(magicMissileManacost),
    frostBoltManacost(frostBoltManacost), fireballManacost(fireballManacost), hasteManacost(hasteManacost),
    shieldManacost(shieldManacost), staffDamage(staffDamage), staffSector(staffSector), staffRange(staffRange),
    levelUpXpValues(levelUpXpValues), minionRadius(minionRadius), minionVisionRange(minionVisionRange),
    minionSpeed(minionSpeed), minionMaxTurnAngle(minionMaxTurnAngle), minionLife(minionLife),
    factionMinionAppearanceIntervalTicks(factionMinionAppearanceIntervalTicks),
    orcWoodcutterActionCooldownTicks(orcWoodcutterActionCooldownTicks), orcWoodcutterDamage(orcWoodcutterDamage),
    orcWoodcutterAttackSector(orcWoodcutterAttackSector), orcWoodcutterAttackRange(orcWoodcutterAttackRange),
    fetishBlowdartActionCooldownTicks(fetishBlowdartActionCooldownTicks),
    fetishBlowdartAttackRange(fetishBlowdartAttackRange), fetishBlowdartAttackSector(fetishBlowdartAttackSector),
    bonusRadius(bonusRadius), bonusAppearanceIntervalTicks(bonusAppearanceIntervalTicks),
    bonusScoreAmount(bonusScoreAmount), dartRadius(dartRadius), dartSpeed(dartSpeed),
    dartDirectDamage(dartDirectDamage), magicMissileRadius(magicMissileRadius), magicMissileSpeed(magicMissileSpeed),
    magicMissileDirectDamage(magicMissileDirectDamage), frostBoltRadius(frostBoltRadius),
    frostBoltSpeed(frostBoltSpeed), frostBoltDirectDamage(frostBoltDirectDamage), fireballRadius(fireballRadius),
    fireballSpeed(fireballSpeed), fireballExplosionMaxDamageRange(fireballExplosionMaxDamageRange),
    fireballExplosionMinDamageRange(fireballExplosionMinDamageRange),
    fireballExplosionMaxDamage(fireballExplosionMaxDamage), fireballExplosionMinDamage(fireballExplosionMinDamage),
    guardianTowerRadius(guardianTowerRadius), guardianTowerVisionRange(guardianTowerVisionRange),
    guardianTowerLife(guardianTowerLife), guardianTowerAttackRange(guardianTowerAttackRange),
    guardianTowerDamage(guardianTowerDamage), guardianTowerCooldownTicks(guardianTowerCooldownTicks),
    factionBaseRadius(factionBaseRadius), factionBaseVisionRange(factionBaseVisionRange),
    factionBaseLife(factionBaseLife), factionBaseAttackRange(factionBaseAttackRange),
    factionBaseDamage(factionBaseDamage), factionBaseCooldownTicks(factionBaseCooldownTicks),
    burningDurationTicks(burningDurationTicks), burningSummaryDamage(burningSummaryDamage),
    empoweredDurationTicks(empoweredDurationTicks), empoweredDamageFactor(empoweredDamageFactor),
    frozenDurationTicks(frozenDurationTicks), hastenedDurationTicks(hastenedDurationTicks),
    hastenedBonusDurationFactor(hastenedBonusDurationFactor), hastenedMovementBonusFactor(hastenedMovementBonusFactor),
    hastenedRotationBonusFactor(hastenedRotationBonusFactor), shieldedDurationTicks(shieldedDurationTicks),
    shieldedBonusDurationFactor(shieldedBonusDurationFactor),
    shieldedDirectDamageAbsorptionFactor(shieldedDirectDamageAbsorptionFactor), auraSkillRange(auraSkillRange),
    rangeBonusPerSkillLevel(rangeBonusPerSkillLevel), magicalDamageBonusPerSkillLevel(magicalDamageBonusPerSkillLevel),
    staffDamageBonusPerSkillLevel(staffDamageBonusPerSkillLevel),
    movementBonusFactorPerSkillLevel(movementBonusFactorPerSkillLevel),
    magicalDamageAbsorptionPerSkillLevel(magicalDamageAbsorptionPerSkillLevel) { }

long long Game::getRandomSeed() const {
    return randomSeed;
}

int Game::getTickCount() const {
    return tickCount;
}

double Game::getMapSize() const {
    return mapSize;
}

bool Game::isSkillsEnabled() const {
    return skillsEnabled;
}

bool Game::isRawMessagesEnabled() const {
    return rawMessagesEnabled;
}

double Game::getFriendlyFireDamageFactor() const {
    return friendlyFireDamageFactor;
}

double Game::getBuildingDamageScoreFactor() const {
    return buildingDamageScoreFactor;
}

double Game::getBuildingEliminationScoreFactor() const {
    return buildingEliminationScoreFactor;
}

double Game::getMinionDamageScoreFactor() const {
    return minionDamageScoreFactor;
}

double Game::getMinionEliminationScoreFactor() const {
    return minionEliminationScoreFactor;
}

double Game::getWizardDamageScoreFactor() const {
    return wizardDamageScoreFactor;
}

double Game::getWizardEliminationScoreFactor() const {
    return wizardEliminationScoreFactor;
}

double Game::getTeamWorkingScoreFactor() const {
    return teamWorkingScoreFactor;
}

int Game::getVictoryScore() const {
    return victoryScore;
}

double Game::getScoreGainRange() const {
    return scoreGainRange;
}

int Game::getRawMessageMaxLength() const {
    return rawMessageMaxLength;
}

double Game::getRawMessageTransmissionSpeed() const {
    return rawMessageTransmissionSpeed;
}

double Game::getWizardRadius() const {
    return wizardRadius;
}

double Game::getWizardCastRange() const {
    return wizardCastRange;
}

double Game::getWizardVisionRange() const {
    return wizardVisionRange;
}

double Game::getWizardForwardSpeed() const {
    return wizardForwardSpeed;
}

double Game::getWizardBackwardSpeed() const {
    return wizardBackwardSpeed;
}

double Game::getWizardStrafeSpeed() const {
    return wizardStrafeSpeed;
}

int Game::getWizardBaseLife() const {
    return wizardBaseLife;
}

int Game::getWizardLifeGrowthPerLevel() const {
    return wizardLifeGrowthPerLevel;
}

int Game::getWizardBaseMana() const {
    return wizardBaseMana;
}

int Game::getWizardManaGrowthPerLevel() const {
    return wizardManaGrowthPerLevel;
}

double Game::getWizardBaseLifeRegeneration() const {
    return wizardBaseLifeRegeneration;
}

double Game::getWizardLifeRegenerationGrowthPerLevel() const {
    return wizardLifeRegenerationGrowthPerLevel;
}

double Game::getWizardBaseManaRegeneration() const {
    return wizardBaseManaRegeneration;
}

double Game::getWizardManaRegenerationGrowthPerLevel() const {
    return wizardManaRegenerationGrowthPerLevel;
}

double Game::getWizardMaxTurnAngle() const {
    return wizardMaxTurnAngle;
}

int Game::getWizardMaxResurrectionDelayTicks() const {
    return wizardMaxResurrectionDelayTicks;
}

int Game::getWizardMinResurrectionDelayTicks() const {
    return wizardMinResurrectionDelayTicks;
}

int Game::getWizardActionCooldownTicks() const {
    return wizardActionCooldownTicks;
}

int Game::getStaffCooldownTicks() const {
    return staffCooldownTicks;
}

int Game::getMagicMissileCooldownTicks() const {
    return magicMissileCooldownTicks;
}

int Game::getFrostBoltCooldownTicks() const {
    return frostBoltCooldownTicks;
}

int Game::getFireballCooldownTicks() const {
    return fireballCooldownTicks;
}

int Game::getHasteCooldownTicks() const {
    return hasteCooldownTicks;
}

int Game::getShieldCooldownTicks() const {
    return shieldCooldownTicks;
}

int Game::getMagicMissileManacost() const {
    return magicMissileManacost;
}

int Game::getFrostBoltManacost() const {
    return frostBoltManacost;
}

int Game::getFireballManacost() const {
    return fireballManacost;
}

int Game::getHasteManacost() const {
    return hasteManacost;
}

int Game::getShieldManacost() const {
    return shieldManacost;
}

int Game::getStaffDamage() const {
    return staffDamage;
}

double Game::getStaffSector() const {
    return staffSector;
}

double Game::getStaffRange() const {
    return staffRange;
}

const vector<int>& Game::getLevelUpXpValues() const {
    return levelUpXpValues;
}

double Game::getMinionRadius() const {
    return minionRadius;
}

double Game::getMinionVisionRange() const {
    return minionVisionRange;
}

double Game::getMinionSpeed() const {
    return minionSpeed;
}

double Game::getMinionMaxTurnAngle() const {
    return minionMaxTurnAngle;
}

int Game::getMinionLife() const {
    return minionLife;
}

int Game::getFactionMinionAppearanceIntervalTicks() const {
    return factionMinionAppearanceIntervalTicks;
}

int Game::getOrcWoodcutterActionCooldownTicks() const {
    return orcWoodcutterActionCooldownTicks;
}

int Game::getOrcWoodcutterDamage() const {
    return orcWoodcutterDamage;
}

double Game::getOrcWoodcutterAttackSector() const {
    return orcWoodcutterAttackSector;
}

double Game::getOrcWoodcutterAttackRange() const {
    return orcWoodcutterAttackRange;
}

int Game::getFetishBlowdartActionCooldownTicks() const {
    return fetishBlowdartActionCooldownTicks;
}

double Game::getFetishBlowdartAttackRange() const {
    return fetishBlowdartAttackRange;
}

double Game::getFetishBlowdartAttackSector() const {
    return fetishBlowdartAttackSector;
}

double Game::getBonusRadius() const {
    return bonusRadius;
}

int Game::getBonusAppearanceIntervalTicks() const {
    return bonusAppearanceIntervalTicks;
}

int Game::getBonusScoreAmount() const {
    return bonusScoreAmount;
}

double Game::getDartRadius() const {
    return dartRadius;
}

double Game::getDartSpeed() const {
    return dartSpeed;
}

int Game::getDartDirectDamage() const {
    return dartDirectDamage;
}

double Game::getMagicMissileRadius() const {
    return magicMissileRadius;
}

double Game::getMagicMissileSpeed() const {
    return magicMissileSpeed;
}

int Game::getMagicMissileDirectDamage() const {
    return magicMissileDirectDamage;
}

double Game::getFrostBoltRadius() const {
    return frostBoltRadius;
}

double Game::getFrostBoltSpeed() const {
    return frostBoltSpeed;
}

int Game::getFrostBoltDirectDamage() const {
    return frostBoltDirectDamage;
}

double Game::getFireballRadius() const {
    return fireballRadius;
}

double Game::getFireballSpeed() const {
    return fireballSpeed;
}

double Game::getFireballExplosionMaxDamageRange() const {
    return fireballExplosionMaxDamageRange;
}

double Game::getFireballExplosionMinDamageRange() const {
    return fireballExplosionMinDamageRange;
}

int Game::getFireballExplosionMaxDamage() const {
    return fireballExplosionMaxDamage;
}

int Game::getFireballExplosionMinDamage() const {
    return fireballExplosionMinDamage;
}

double Game::getGuardianTowerRadius() const {
    return guardianTowerRadius;
}

double Game::getGuardianTowerVisionRange() const {
    return guardianTowerVisionRange;
}

double Game::getGuardianTowerLife() const {
    return guardianTowerLife;
}

double Game::getGuardianTowerAttackRange() const {
    return guardianTowerAttackRange;
}

int Game::getGuardianTowerDamage() const {
    return guardianTowerDamage;
}

int Game::getGuardianTowerCooldownTicks() const {
    return guardianTowerCooldownTicks;
}

double Game::getFactionBaseRadius() const {
    return factionBaseRadius;
}

double Game::getFactionBaseVisionRange() const {
    return factionBaseVisionRange;
}

double Game::getFactionBaseLife() const {
    return factionBaseLife;
}

double Game::getFactionBaseAttackRange() const {
    return factionBaseAttackRange;
}

int Game::getFactionBaseDamage() const {
    return factionBaseDamage;
}

int Game::getFactionBaseCooldownTicks() const {
    return factionBaseCooldownTicks;
}

int Game::getBurningDurationTicks() const {
    return burningDurationTicks;
}

int Game::getBurningSummaryDamage() const {
    return burningSummaryDamage;
}

int Game::getEmpoweredDurationTicks() const {
    return empoweredDurationTicks;
}

double Game::getEmpoweredDamageFactor() const {
    return empoweredDamageFactor;
}

int Game::getFrozenDurationTicks() const {
    return frozenDurationTicks;
}

int Game::getHastenedDurationTicks() const {
    return hastenedDurationTicks;
}

double Game::getHastenedBonusDurationFactor() const {
    return hastenedBonusDurationFactor;
}

double Game::getHastenedMovementBonusFactor() const {
    return hastenedMovementBonusFactor;
}

double Game::getHastenedRotationBonusFactor() const {
    return hastenedRotationBonusFactor;
}

int Game::getShieldedDurationTicks() const {
    return shieldedDurationTicks;
}

double Game::getShieldedBonusDurationFactor() const {
    return shieldedBonusDurationFactor;
}

double Game::getShieldedDirectDamageAbsorptionFactor() const {
    return shieldedDirectDamageAbsorptionFactor;
}

double Game::getAuraSkillRange() const {
    return auraSkillRange;
}

double Game::getRangeBonusPerSkillLevel() const {
    return rangeBonusPerSkillLevel;
}

int Game::getMagicalDamageBonusPerSkillLevel() const {
    return magicalDamageBonusPerSkillLevel;
}

int Game::getStaffDamageBonusPerSkillLevel() const {
    return staffDamageBonusPerSkillLevel;
}

double Game::getMovementBonusFactorPerSkillLevel() const {
    return movementBonusFactorPerSkillLevel;
}

int Game::getMagicalDamageAbsorptionPerSkillLevel() const {
    return magicalDamageAbsorptionPerSkillLevel;
}
