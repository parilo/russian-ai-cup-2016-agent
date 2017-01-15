//
//  Quadrocopter2DBrain.hpp
//  Quadrocopter2DBrain
//
//  Created by anton on 29/04/16.
//  Copyright © 2016 anton. All rights reserved.
//

#ifndef Quadrocopter2DBrain_hpp
#define Quadrocopter2DBrain_hpp

#include <vector>
#include <list>
#include <memory>

namespace Quadrocopter2DBrain {

	void initApiDiscreteDeepQ ();
	
	void quadrocopterBrainActCont(
		int quadrocopterId,
		std::shared_ptr<std::vector<float>> state,
		std::shared_ptr<std::vector<float>> action
	);
	
	bool quadrocopterBrainTrain ();

	void storeQuadrocopterExperienceCont (
		int quadrocopterId,
		double reward,
		std::shared_ptr<std::vector <float>> action,
		std::shared_ptr<std::vector <float>> prevState,
		std::shared_ptr<std::vector <float>> nextState
	);
	
	bool getBigErrorExp (
		std::vector <float>& state
	);

}

#endif /* Quadrocopter2DBrain_hpp */
