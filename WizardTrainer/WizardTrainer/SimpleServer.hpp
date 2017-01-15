//
//  SimpleServer.hpp
//  WizardTrainer
//
//  Created by anton on 12/11/2016.
//  Copyright © 2016 anton. All rights reserved.
//

#ifndef SimpleServer_hpp
#define SimpleServer_hpp

#include <sys/socket.h>
#include <netinet/in.h>

#include <vector>
#include <thread>

namespace WizardTrainer {

	class SimpleServerClientData {
	public:
		SimpleServerClientData () {}
		int id;
		int sockfd = -1;
		struct sockaddr addr;
		bool needToReadState = true;
		bool needToReadReward = false;
	};

	typedef std::function<void (int id, std::shared_ptr<std::vector<float>> state, std::shared_ptr<std::vector<float>> action)> AgentStateListener;
	typedef std::function<void (
		int id,
		float reward,
		std::shared_ptr<std::vector<float>> prevState,
		std::shared_ptr<std::vector<float>> nextState,
		std::shared_ptr<std::vector<float>> action
	)> AgentRewardListener;

	class SimpleServerClientWorker {
	public:

		SimpleServerClientData cl;
	
		SimpleServerClientWorker () {}
		SimpleServerClientWorker (const SimpleServerClientWorker& w);
		SimpleServerClientWorker (const SimpleServerClientData& cl, int stateSize, int actionSize);
		
		SimpleServerClientWorker& operator= (const SimpleServerClientWorker& other);
		
		void run ();
		void stop ();
		bool isLive ();

		void setStateListener (const AgentStateListener& listener);
		void setRewardListener (const AgentRewardListener& listener);
	
	private:

		int stateSize;
		int actionSize;
		
		bool active = false;
		int numOfTimeout = 0;
		
//		const int readBufferSize = 16384;
//		std::vector<char> readBuffer;
		std::thread readThread;

		AgentStateListener stateListener;
		AgentRewardListener rewardListener;
		
	};

	class SimpleServer {
	public:
	
		SimpleServer (int stateSize, int actionSize);
	
		void start ();
		void wait ();
		void sendAction (int id, std::shared_ptr<std::vector<float>> action);
		
		void setStateListener (const AgentStateListener& listener);
		void setRewardListener (const AgentRewardListener& listener);
	
	private:

		int stateSize;
		int actionSize;

		int sockfd;
		const int portno = 12350;
		const int maxClients = 16;
		int newClientPos = 0;
		
		struct sockaddr_in addr;
		
		std::vector<SimpleServerClientWorker> connectedClients;
		std::thread listenThread;
		
		std::thread readThread;

		AgentStateListener stateListener;
		AgentRewardListener rewardListener;
	};

	void configSocket (int sock);
	bool readFromSocket (int sock, char* buffer, int size);

}


#endif /* SimpleServer_hpp */
