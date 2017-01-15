//
//  main.cpp
//  WizardTrainer
//
//  Created by anton on 12/11/2016.
//  Copyright © 2016 anton. All rights reserved.
//

#include <iostream>
#include <math.h>

#include "SimpleServer.hpp"
#include "SimpleClient.hpp"
#include "Quadrocopter2DBrain.hpp"
#include "QuadrocopterBrain.hpp"

using namespace WizardTrainer;
using namespace std;

double randDouble (double LO, double HI) {
    return LO + std::rand() /(RAND_MAX/(HI-LO));
}

std::mutex mtxListener;

int main(int argc, const char * argv[]) {

	Quadrocopter2DBrain::initApiDiscreteDeepQ ();

	SimpleServer s (QuadrocopterBrain::observationSize, QuadrocopterBrain::contActionSize);
	
//	std::vector<std::vector<double>> prevStates (16);
	
	s.setStateListener([](int id, std::shared_ptr<std::vector<float>> state, std::shared_ptr<std::vector<float>> action){

if (state->size () != QuadrocopterBrain::observationSize)
std::cout << "--- error state recived: " << id << " " << state->size() << std::endl;

//		prevStates [id] = state;

//std::cout << "--- act: " << id << " " << st.size() << " " << a.size () << std::endl;
		Quadrocopter2DBrain::quadrocopterBrainActCont(
			id,
			state,
			action
		);

//std::cout << "--- end act: " << id << " " << st.size() << " " << a.size () << std::endl;
//std::cout << "--- end act2: " << id << " " << state.size() << " " << action.size () << std::endl;
	});
	
	s.setRewardListener([](
	    int id,
	    double reward,
	    std::shared_ptr<std::vector<float>> prevState,
	    std::shared_ptr<std::vector<float>> nextState,
	    std::shared_ptr<std::vector<float>> action//,
//	    bool useLambda
	){
		std::lock_guard<std::mutex> lock (mtxListener);

//if (reward != 0)
//std::cout << "--- reward recived: " << id << " " << reward << " " << nextState.size() << " " << action.size () << std::endl;

/*//if (reward != 0)
std::cout << "--- reward recived: " << id << " " << reward << " " << nextState.size() << " " << action.size () << std::endl;
		for (int i=0; i< n.size(); i++) {
float d = n [i] - p [i];
if (fabs(d) > 1)
std::cout << n [i] - p [i] << " ";
		}
std::cout << std::endl;
*/
		Quadrocopter2DBrain::storeQuadrocopterExperienceCont (
			id,
			reward,
			action,
			prevState,
			nextState
		);

	});
	
	s.start();
	

//	c.setActionListener([&c](const std::vector<double>& action){
//std::cout << "--- action recived: " << action.size() << std::endl;
//		for (int i=0; i< action.size(); i++) {
//std::cout << action [i] << " ";
//		}
//std::cout << std::endl;
//		std::vector<double> r;
//		r.resize(action.size()+1);
//		r [0] = 123.123;
//		std::copy(action.begin(), action.end(), r.begin()+1);
//		c.sendReward(r);
//		c.sendState(action);
//	});

//	SimpleClient c1 (1013, 13);
//	SimpleClient c2 (1013, 13);
//
//	c1.connectToServer();
//	c2.connectToServer();
//
//	std::vector<double> buffer;
//	for (int i=0; i<1013; i++) {
//		buffer.push_back(i + 0.1);
//	}
//	c1.sendState (buffer);
//	c2.sendState (buffer);
	
	for (;;) {
		if (!Quadrocopter2DBrain::quadrocopterBrainTrain ()) {
			sleep (1);
		}
	}
	
	s.wait();

	std::cout << "Hello, World!\n";
    return 0;
}
