//
//  SimpleClient.hpp
//  WizardTrainer
//
//  Created by anton on 12/11/2016.
//  Copyright © 2016 anton. All rights reserved.
//

#ifndef SimpleClient_hpp
#define SimpleClient_hpp

#include <vector>
#include <thread>

namespace WizardTrainer {

	typedef std::function<void (const std::vector<float>& action)> AgentActionListener;

	class SimpleClient {
	public:
	
		SimpleClient (int stateSize, int actionSize);
	
		void connectToServer ();
		void disconnect ();
		
		void sendState (const std::vector<float>& state);
		void sendReward (
			float reward,
			const std::vector<float>& prevState,
			const std::vector<float>& nextState,
			std::vector<float>& action
		);
		void setActionListener (const AgentActionListener& listener);
	
	private:
	
		int stateSize;
		int actionSize;
	
		int sockfd;
		int portno = 12350;
		
//		const int readBufferSize = 16384;
//		std::vector<char> readBuffer;
		
		bool connected;
		std::thread readThread;
		
		AgentActionListener actionListener;

		void send (const std::vector<float>& data);
		bool recv (std::vector<float>& data, int size);
	
	};

}

#endif /* SimpleClient_hpp */
