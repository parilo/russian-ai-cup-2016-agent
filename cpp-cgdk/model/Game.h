#pragma once

#ifndef _GAME_H_
#define _GAME_H_

#include <vector>

namespace model {
    class Game {
    private:
        long long randomSeed;
        int tickCount;
        double mapSize;
        bool skillsEnabled;
        bool rawMessagesEnabled;
        double friendlyFireDamageFactor;
        double buildingDamageScoreFactor;
        double buildingEliminationScoreFactor;
        double minionDamageScoreFactor;
        double minionEliminationScoreFactor;
        double wizardDamageScoreFactor;
        double wizardEliminationScoreFactor;
        double teamWorkingScoreFactor;
        int victoryScore;
        double scoreGainRange;
        int rawMessageMaxLength;
        double rawMessageTransmissionSpeed;
        double wizardRadius;
        double wizardCastRange;
        double wizardVisionRange;
        double wizardForwardSpeed;
        double wizardBackwardSpeed;
        double wizardStrafeSpeed;
        int wizardBaseLife;
        int wizardLifeGrowthPerLevel;
        int wizardBaseMana;
        int wizardManaGrowthPerLevel;
        double wizardBaseLifeRegeneration;
        double wizardLifeRegenerationGrowthPerLevel;
        double wizardBaseManaRegeneration;
        double wizardManaRegenerationGrowthPerLevel;
        double wizardMaxTurnAngle;
        int wizardMaxResurrectionDelayTicks;
        int wizardMinResurrectionDelayTicks;
        int wizardActionCooldownTicks;
        int staffCooldownTicks;
        int magicMissileCooldownTicks;
        int frostBoltCooldownTicks;
        int fireballCooldownTicks;
        int hasteCooldownTicks;
        int shieldCooldownTicks;
        int magicMissileManacost;
        int frostBoltManacost;
        int fireballManacost;
        int hasteManacost;
        int shieldManacost;
        int staffDamage;
        double staffSector;
        double staffRange;
        std::vector<int> levelUpXpValues;
        double minionRadius;
        double minionVisionRange;
        double minionSpeed;
        double minionMaxTurnAngle;
        int minionLife;
        int factionMinionAppearanceIntervalTicks;
        int orcWoodcutterActionCooldownTicks;
        int orcWoodcutterDamage;
        double orcWoodcutterAttackSector;
        double orcWoodcutterAttackRange;
        int fetishBlowdartActionCooldownTicks;
        double fetishBlowdartAttackRange;
        double fetishBlowdartAttackSector;
        double bonusRadius;
        int bonusAppearanceIntervalTicks;
        int bonusScoreAmount;
        double dartRadius;
        double dartSpeed;
        int dartDirectDamage;
        double magicMissileRadius;
        double magicMissileSpeed;
        int magicMissileDirectDamage;
        double frostBoltRadius;
        double frostBoltSpeed;
        int frostBoltDirectDamage;
        double fireballRadius;
        double fireballSpeed;
        double fireballExplosionMaxDamageRange;
        double fireballExplosionMinDamageRange;
        int fireballExplosionMaxDamage;
        int fireballExplosionMinDamage;
        double guardianTowerRadius;
        double guardianTowerVisionRange;
        double guardianTowerLife;
        double guardianTowerAttackRange;
        int guardianTowerDamage;
        int guardianTowerCooldownTicks;
        double factionBaseRadius;
        double factionBaseVisionRange;
        double factionBaseLife;
        double factionBaseAttackRange;
        int factionBaseDamage;
        int factionBaseCooldownTicks;
        int burningDurationTicks;
        int burningSummaryDamage;
        int empoweredDurationTicks;
        double empoweredDamageFactor;
        int frozenDurationTicks;
        int hastenedDurationTicks;
        double hastenedBonusDurationFactor;
        double hastenedMovementBonusFactor;
        double hastenedRotationBonusFactor;
        int shieldedDurationTicks;
        double shieldedBonusDurationFactor;
        double shieldedDirectDamageAbsorptionFactor;
        double auraSkillRange;
        double rangeBonusPerSkillLevel;
        int magicalDamageBonusPerSkillLevel;
        int staffDamageBonusPerSkillLevel;
        double movementBonusFactorPerSkillLevel;
        int magicalDamageAbsorptionPerSkillLevel;
    public:
        Game();
        Game(long long randomSeed, int tickCount, double mapSize, bool skillsEnabled, bool rawMessagesEnabled,
                double friendlyFireDamageFactor, double buildingDamageScoreFactor,
                double buildingEliminationScoreFactor, double minionDamageScoreFactor,
                double minionEliminationScoreFactor, double wizardDamageScoreFactor,
                double wizardEliminationScoreFactor, double teamWorkingScoreFactor, int victoryScore,
                double scoreGainRange, int rawMessageMaxLength, double rawMessageTransmissionSpeed, double wizardRadius,
                double wizardCastRange, double wizardVisionRange, double wizardForwardSpeed, double wizardBackwardSpeed,
                double wizardStrafeSpeed, int wizardBaseLife, int wizardLifeGrowthPerLevel, int wizardBaseMana,
                int wizardManaGrowthPerLevel, double wizardBaseLifeRegeneration,
                double wizardLifeRegenerationGrowthPerLevel, double wizardBaseManaRegeneration,
                double wizardManaRegenerationGrowthPerLevel, double wizardMaxTurnAngle,
                int wizardMaxResurrectionDelayTicks, int wizardMinResurrectionDelayTicks, int wizardActionCooldownTicks,
                int staffCooldownTicks, int magicMissileCooldownTicks, int frostBoltCooldownTicks,
                int fireballCooldownTicks, int hasteCooldownTicks, int shieldCooldownTicks, int magicMissileManacost,
                int frostBoltManacost, int fireballManacost, int hasteManacost, int shieldManacost, int staffDamage,
                double staffSector, double staffRange, const std::vector<int>& levelUpXpValues, double minionRadius,
                double minionVisionRange, double minionSpeed, double minionMaxTurnAngle, int minionLife,
                int factionMinionAppearanceIntervalTicks, int orcWoodcutterActionCooldownTicks, int orcWoodcutterDamage,
                double orcWoodcutterAttackSector, double orcWoodcutterAttackRange,
                int fetishBlowdartActionCooldownTicks, double fetishBlowdartAttackRange,
                double fetishBlowdartAttackSector, double bonusRadius, int bonusAppearanceIntervalTicks,
                int bonusScoreAmount, double dartRadius, double dartSpeed, int dartDirectDamage,
                double magicMissileRadius, double magicMissileSpeed, int magicMissileDirectDamage,
                double frostBoltRadius, double frostBoltSpeed, int frostBoltDirectDamage, double fireballRadius,
                double fireballSpeed, double fireballExplosionMaxDamageRange, double fireballExplosionMinDamageRange,
                int fireballExplosionMaxDamage, int fireballExplosionMinDamage, double guardianTowerRadius,
                double guardianTowerVisionRange, double guardianTowerLife, double guardianTowerAttackRange,
                int guardianTowerDamage, int guardianTowerCooldownTicks, double factionBaseRadius,
                double factionBaseVisionRange, double factionBaseLife, double factionBaseAttackRange,
                int factionBaseDamage, int factionBaseCooldownTicks, int burningDurationTicks, int burningSummaryDamage,
                int empoweredDurationTicks, double empoweredDamageFactor, int frozenDurationTicks,
                int hastenedDurationTicks, double hastenedBonusDurationFactor, double hastenedMovementBonusFactor,
                double hastenedRotationBonusFactor, int shieldedDurationTicks, double shieldedBonusDurationFactor,
                double shieldedDirectDamageAbsorptionFactor, double auraSkillRange, double rangeBonusPerSkillLevel,
                int magicalDamageBonusPerSkillLevel, int staffDamageBonusPerSkillLevel,
                double movementBonusFactorPerSkillLevel, int magicalDamageAbsorptionPerSkillLevel);

        long long getRandomSeed() const;
        int getTickCount() const;
        double getMapSize() const;
        bool isSkillsEnabled() const;
        bool isRawMessagesEnabled() const;
        double getFriendlyFireDamageFactor() const;
        double getBuildingDamageScoreFactor() const;
        double getBuildingEliminationScoreFactor() const;
        double getMinionDamageScoreFactor() const;
        double getMinionEliminationScoreFactor() const;
        double getWizardDamageScoreFactor() const;
        double getWizardEliminationScoreFactor() const;
        double getTeamWorkingScoreFactor() const;
        int getVictoryScore() const;
        double getScoreGainRange() const;
        int getRawMessageMaxLength() const;
        double getRawMessageTransmissionSpeed() const;
        double getWizardRadius() const;
        double getWizardCastRange() const;
        double getWizardVisionRange() const;
        double getWizardForwardSpeed() const;
        double getWizardBackwardSpeed() const;
        double getWizardStrafeSpeed() const;
        int getWizardBaseLife() const;
        int getWizardLifeGrowthPerLevel() const;
        int getWizardBaseMana() const;
        int getWizardManaGrowthPerLevel() const;
        double getWizardBaseLifeRegeneration() const;
        double getWizardLifeRegenerationGrowthPerLevel() const;
        double getWizardBaseManaRegeneration() const;
        double getWizardManaRegenerationGrowthPerLevel() const;
        double getWizardMaxTurnAngle() const;
        int getWizardMaxResurrectionDelayTicks() const;
        int getWizardMinResurrectionDelayTicks() const;
        int getWizardActionCooldownTicks() const;
        int getStaffCooldownTicks() const;
        int getMagicMissileCooldownTicks() const;
        int getFrostBoltCooldownTicks() const;
        int getFireballCooldownTicks() const;
        int getHasteCooldownTicks() const;
        int getShieldCooldownTicks() const;
        int getMagicMissileManacost() const;
        int getFrostBoltManacost() const;
        int getFireballManacost() const;
        int getHasteManacost() const;
        int getShieldManacost() const;
        int getStaffDamage() const;
        double getStaffSector() const;
        double getStaffRange() const;
        const std::vector<int>& getLevelUpXpValues() const;
        double getMinionRadius() const;
        double getMinionVisionRange() const;
        double getMinionSpeed() const;
        double getMinionMaxTurnAngle() const;
        int getMinionLife() const;
        int getFactionMinionAppearanceIntervalTicks() const;
        int getOrcWoodcutterActionCooldownTicks() const;
        int getOrcWoodcutterDamage() const;
        double getOrcWoodcutterAttackSector() const;
        double getOrcWoodcutterAttackRange() const;
        int getFetishBlowdartActionCooldownTicks() const;
        double getFetishBlowdartAttackRange() const;
        double getFetishBlowdartAttackSector() const;
        double getBonusRadius() const;
        int getBonusAppearanceIntervalTicks() const;
        int getBonusScoreAmount() const;
        double getDartRadius() const;
        double getDartSpeed() const;
        int getDartDirectDamage() const;
        double getMagicMissileRadius() const;
        double getMagicMissileSpeed() const;
        int getMagicMissileDirectDamage() const;
        double getFrostBoltRadius() const;
        double getFrostBoltSpeed() const;
        int getFrostBoltDirectDamage() const;
        double getFireballRadius() const;
        double getFireballSpeed() const;
        double getFireballExplosionMaxDamageRange() const;
        double getFireballExplosionMinDamageRange() const;
        int getFireballExplosionMaxDamage() const;
        int getFireballExplosionMinDamage() const;
        double getGuardianTowerRadius() const;
        double getGuardianTowerVisionRange() const;
        double getGuardianTowerLife() const;
        double getGuardianTowerAttackRange() const;
        int getGuardianTowerDamage() const;
        int getGuardianTowerCooldownTicks() const;
        double getFactionBaseRadius() const;
        double getFactionBaseVisionRange() const;
        double getFactionBaseLife() const;
        double getFactionBaseAttackRange() const;
        int getFactionBaseDamage() const;
        int getFactionBaseCooldownTicks() const;
        int getBurningDurationTicks() const;
        int getBurningSummaryDamage() const;
        int getEmpoweredDurationTicks() const;
        double getEmpoweredDamageFactor() const;
        int getFrozenDurationTicks() const;
        int getHastenedDurationTicks() const;
        double getHastenedBonusDurationFactor() const;
        double getHastenedMovementBonusFactor() const;
        double getHastenedRotationBonusFactor() const;
        int getShieldedDurationTicks() const;
        double getShieldedBonusDurationFactor() const;
        double getShieldedDirectDamageAbsorptionFactor() const;
        double getAuraSkillRange() const;
        double getRangeBonusPerSkillLevel() const;
        int getMagicalDamageBonusPerSkillLevel() const;
        int getStaffDamageBonusPerSkillLevel() const;
        double getMovementBonusFactorPerSkillLevel() const;
        int getMagicalDamageAbsorptionPerSkillLevel() const;
    };
}

#endif
