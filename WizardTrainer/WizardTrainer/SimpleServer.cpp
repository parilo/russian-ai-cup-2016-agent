//
//  SimpleServer.cpp
//  WizardTrainer
//
//  Created by anton on 12/11/2016.
//  Copyright © 2016 anton. All rights reserved.
//

#include "SimpleServer.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h> 
#include <netinet/tcp.h>

#include <iostream>

using namespace WizardTrainer;

SimpleServer::SimpleServer (int stateSize, int actionSize) :
	stateSize(stateSize),
	actionSize(actionSize)
{}

void SimpleServer::start ()
{
	connectedClients.resize(maxClients);

	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	//sockfd = socket(AF_INET, SOCK_DGRAM, 0);
	configSocket (sockfd);

	if (sockfd < 0) {
		std::cerr << "SimpleServer::start: ERROR opening socket" << std::endl;
	}
	bzero((char *) &addr, sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_addr.s_addr = INADDR_ANY;
	addr.sin_port = htons(portno);
	if (bind(sockfd, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
		std::cerr << "SimpleServer::start: ERROR on binding" << std::endl;
	}
	listen(sockfd, maxClients);
	
	listenThread = std::thread([this](){
		while (true) {
			SimpleServerClientData cl;
			socklen_t clilen;
			clilen = sizeof(cl.addr);
			int connSock = accept(sockfd, &cl.addr, &clilen);

			if (connSock < 0) {
				continue;
				//std::cerr << "SimpleServer::start: ERROR on accept" << std::endl;
			} else {
				std::cout << "--- connected client: " << newClientPos << " " << connSock << std::endl;
			}
			auto& clw = connectedClients [newClientPos];
			if (clw.isLive()) {
				clw.stop();
			}
			
			cl.id = newClientPos;
			cl.sockfd = connSock;

			clw = SimpleServerClientWorker(cl, stateSize, actionSize);
			clw.setStateListener(stateListener);
			clw.setRewardListener(rewardListener);
			clw.run ();
			
			newClientPos++;
			newClientPos %= maxClients;
		}
	});
}

void SimpleServer::sendAction (int id, std::shared_ptr<std::vector<float>> actionPtr) {
	auto& action = *actionPtr;
	const char* b = reinterpret_cast<const char*>(action.data());
	
	auto& cl = connectedClients [id];

//std::cout << "--- send action begin" << action.size () << std::endl;
    int n = write(cl.cl.sockfd, b, action.size() * sizeof(float));
//std::cout << "--- send action end" << std::endl;
    if (n < 0) {
		std::cerr << "SimpleServer: ERROR writing to socket" << std::endl;
	}
	
	cl.cl.needToReadReward = true;
}

void SimpleServer::wait () {
	listenThread.join();
	readThread.join();
}

void SimpleServer::setStateListener (const AgentStateListener& listener) {
	stateListener = listener;
}

void SimpleServer::setRewardListener (const AgentRewardListener& listener) {
	rewardListener = listener;
}



SimpleServerClientWorker::SimpleServerClientWorker (const SimpleServerClientData& cl, int stateSize, int actionSize) :
	stateSize(stateSize),
	actionSize(actionSize),
	cl (cl)
{}

SimpleServerClientWorker::SimpleServerClientWorker (const SimpleServerClientWorker& w) :
	SimpleServerClientWorker (w.cl, w.stateSize, w.actionSize)
{}

SimpleServerClientWorker& SimpleServerClientWorker::operator= (const SimpleServerClientWorker& other)
{
	stateSize = other.stateSize;
	actionSize = other.actionSize;
	cl = other.cl;
	numOfTimeout = other.numOfTimeout;
	return *this;
}


bool WizardTrainer::readFromSocket (int sock, char* buffer, int size) {
	
	int numOfTimeout = 0;
	int readedBytes = 0;
	for (;;) {
//std::cout << "start read: " << sock << " " << readedBytes << " " << size-readedBytes << std::endl;
	    int bytes = recv (sock, buffer+readedBytes, size-readedBytes, MSG_WAITALL);
//std::cout << "end read: " << sock << " " << bytes << std::endl;
	
	    if ( bytes<1 ) {
		numOfTimeout++;
		if (numOfTimeout>100) {
//std::cout << "--- read false: " << readedBytes << " " << sock << " " << size << std::endl;
		    return false;
		}
		continue;
	    } else {
		numOfTimeout = 0;
	    }

	    if (bytes> (size-readedBytes)) {
//std::cout << "--- server reading error: " << bytes << " != " << (size-readedBytes) << std::endl;
	    }

	    readedBytes += bytes;

	    if (readedBytes == size) {
		return true;
	    }
	}
}

void SimpleServerClientWorker::run () {
	active = true;
	
	readThread = std::thread([this](){
		while (active) {
			if (cl.needToReadState) {

				std::shared_ptr<std::vector<float>> statePtr (new std::vector<float> (stateSize));
				auto& state = *statePtr;

//std::cout << "--- server begin state reading: " << cl.id << " " << stateSize << std::endl;
				bool readed = readFromSocket (cl.sockfd, (char*)state.data(), stateSize * sizeof(float));
//std::cout << "--- server end state reading: " << readed << std::endl;

				if ( !readed ) {
				    stop ();
				    continue;
				}

				if (stateListener) {
					std::shared_ptr<std::vector<float>> actionPtr (new std::vector<float>(actionSize));
					auto& action = *actionPtr;
					stateListener (cl.id, statePtr, actionPtr);

//std::cout << "--- send action begin " << action.size () << std::endl;
					int n = write(cl.sockfd, action.data(), action.size() * sizeof(float));
//std::cout << "--- send action end" << std::endl;
					if (n < 0) {
					    std::cerr << "SimpleServer: ERROR writing to socket" << std::endl;
					}
	
					cl.needToReadReward = true;
				}
				cl.needToReadState = false;
				
			} else if (cl.needToReadReward) {

				float r;
				std::shared_ptr<std::vector<float>> prevStatePtr (new std::vector<float> (stateSize));
				std::shared_ptr<std::vector<float>> nextStatePtr (new std::vector<float> (stateSize));
				std::shared_ptr<std::vector<float>> actionPtr (new std::vector<float> (actionSize));
				auto& prevState = *prevStatePtr;
				auto& nextState = *nextStatePtr;
				auto& action = *actionPtr;

//std::cout << "--- server begin reward reading1: " << cl.id << std::endl;
				bool readed = readFromSocket (cl.sockfd, (char*)&r, sizeof(float));
//std::cout << "--- server end reward reading1: " << readed << std::endl;

				if ( !readed ) {
				    stop ();
				    continue;
				}

//std::cout << "--- server begin reward reading2: " << cl.id << std::endl;
				readed = readFromSocket (cl.sockfd, (char*)prevState.data(), stateSize * sizeof(float));
//std::cout << "--- server end reward reading2: " << readed << std::endl;

				if ( !readed ) {
				    stop ();
				    continue;
				}

//std::cout << "--- server begin reward reading2: " << cl.id << std::endl;
				readed = readFromSocket (cl.sockfd, (char*)nextState.data(), stateSize * sizeof(float));
//std::cout << "--- server end reward reading2: " << readed << std::endl;

				if ( !readed ) {
				    stop ();
				    continue;
				}

//std::cout << "--- server begin reward reading3: " << cl.id << std::endl;
				readed = readFromSocket (cl.sockfd, (char*)action.data(), actionSize * sizeof(float));
//std::cout << "--- server end reward reading3: " << readed << std::endl;

				if ( !readed ) {
				    stop ();
				    continue;
				}

				if (rewardListener) rewardListener (cl.id, r, prevStatePtr, nextStatePtr, actionPtr);
				cl.needToReadReward = false;
				cl.needToReadState = true;
			}
		}
	});
	readThread.detach ();
}

void SimpleServerClientWorker::stop () {
	active = false;
	if (cl.sockfd != -1) {
		close(cl.sockfd);
		cl.sockfd = -1;
	}
}

bool SimpleServerClientWorker::isLive () {
	return active;
}

void SimpleServerClientWorker::setStateListener (const AgentStateListener& listener) {
	stateListener = listener;
}

void SimpleServerClientWorker::setRewardListener (const AgentRewardListener& listener) {
	rewardListener = listener;
}

void WizardTrainer::configSocket (int sockfd) {

         int flag = 1;
         int result = setsockopt(sockfd,            /* socket affected */
                                 IPPROTO_TCP,     /* set option at TCP level */
                                 TCP_NODELAY,     /* name of option */
                                 (char *) &flag,  /* the cast is historical
                                                         cruft */
                                 sizeof(int));    /* length of option value */
         if (result < 0) {
		std::cerr << "config socket ERROR setting no delay" << std::endl;
	}

    struct timeval timeout;
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;

    if (setsockopt (sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout,
                sizeof(timeout)) < 0)
        std::cout << "config socket: setsockopt rcv timeout failed" << std::endl;

    if (setsockopt (sockfd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout,
                sizeof(timeout)) < 0)
        std::cout << "config socket: setsockopt snd timeout failed" << std::endl;
}
