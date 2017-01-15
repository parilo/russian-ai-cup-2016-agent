//
//  Quadrocopter2DBrain.cpp
//  Quadrocopter2DBrain
//
//  Created by anton on 29/04/16.
//  Copyright © 2016 anton. All rights reserved.
//

#include "Quadrocopter2DBrain.hpp"

#include <iostream>
#include <math.h>
#include <stdio.h>
#include <vector>
#include <string>

#include "QuadrocopterBrain.hpp"
#include "ExpLambdaFilter.hpp"
#include "DDPG.hpp"

using namespace std;

namespace Quadrocopter2DBrain {

	const int numOfQuadrocopters = 16;

	QuadrocopterBrain quadrocopterBrain (std::shared_ptr<BrainAlgorithm> (new DDPG ()));
	
	vector<ExpLambdaFilter> experienceFilters; //one filter for each quadrocopter;
	vector<double> randomnessOfQuadrocopter;


	void quadrocopterBrainActCont(
		int quadrocopterId,
		std::shared_ptr<std::vector<float>> state,
		std::shared_ptr<std::vector<float>> action
	) {
		quadrocopterBrain.actCont (
			ObservationSeqLimited (state),
			*action,
			randomnessOfQuadrocopter [quadrocopterId]
		);
	}

	void storeQuadrocopterExperienceCont (
		int quadrocopterId,
		double reward,
		std::shared_ptr<std::vector <float>> action,
		std::shared_ptr<std::vector <float>> prevState,
		std::shared_ptr<std::vector <float>> nextState
	) {
		ExperienceItem expItem (
			ObservationSeqLimited (prevState),
			ObservationSeqLimited (nextState),
			reward,
			action
		);
		//expItem.rewardLambda = reward;
		
		//experienceFilters [quadrocopterId].storeExperience (expItem);
		quadrocopterBrain.storeExperience(expItem);
	}

	void initApiDiscreteDeepQ () {
//		currStateSeqs.resize(numOfQuadrocopters);
//		prevStateSeqs.resize(numOfQuadrocopters);
		experienceFilters.resize(numOfQuadrocopters);
		randomnessOfQuadrocopter.clear ();
//		Observation ob;
//		ob.setZeros(QuadrocopterBrain::observationSize);
//		Observation action;
//		action.setZeros(QuadrocopterBrain::contActionSize);
//		Observation reward;
//		reward.setZeros(1);

//		ObservationSeqLimited obs;
//		obs.setLimit(QuadrocopterBrain::observationsInSeq);
//		obs.initWith(ob);
		
		for (int i=0; i<numOfQuadrocopters; i++) {
		
			experienceFilters [i].setExperienceTarget(&quadrocopterBrain);
			
			if (i < 8) {
				randomnessOfQuadrocopter.push_back(20);
			} else
//			if (i < 10) {
			{
				randomnessOfQuadrocopter.push_back(20);
			}
//			else {
//				randomnessOfQuadrocopter.push_back(0.0);
//			}
		}
	}

	bool quadrocopterBrainTrain () {
		return quadrocopterBrain.train();
	}
	
	bool getBigErrorExp (
		std::vector <float>& state
	) {
		ExperienceItem expItem;
		bool got = quadrocopterBrain.getMaxErrorExp(expItem);
		if (got) {
			state = *(expItem.prevStates.getObservation(0).data);
			return true;
		}
		return false;
	}

}

