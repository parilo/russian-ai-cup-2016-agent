//
//  SimpleClient.cpp
//  WizardTrainer
//
//  Created by anton on 12/11/2016.
//  Copyright © 2016 anton. All rights reserved.
//

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>

#include <iostream>

#include "SimpleClient.hpp"
#include "SimpleServer.hpp"

using namespace WizardTrainer;

SimpleClient::SimpleClient (int stateSize, int actionSize) :
	stateSize(stateSize),
	actionSize(actionSize)
{}

void SimpleClient::connectToServer() {

//	readBuffer.resize(readBufferSize);

    struct sockaddr_in serv_addr;

    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    //sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    configSocket (sockfd);

    if (sockfd < 0) {
		std::cerr << "SimpleClient: ERROR opening socket" << std::endl;
	}
    bzero((char *) &serv_addr, sizeof(serv_addr));
	inet_aton ("127.0.0.1", &serv_addr.sin_addr);
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) {
		std::cerr << "SimpleClient: ERROR connecting" << std::endl;
	}
	
	connected = true;
	readThread = std::thread ([this](){
		while (connected) {
			std::vector<float> data;
			bool res = recv (data, actionSize);
			if (res) {
				if (actionListener) actionListener (data);
			} else {
				std::cerr << "error reading action" << std::endl;
				return;
			}
		}
	});
	
}

void SimpleClient::sendState (const std::vector<float>& state) {
//std::cout << "--- write state begin: " << state.size() << std::endl;
	send (state);
//std::cout << "--- write state end" << std::endl;
}

void SimpleClient::sendReward (
    float reward,
    const std::vector<float>& prevState,
    const std::vector<float>& nextState,
    std::vector<float>& action
) {
//std::cout << "--- write reward begin: " << nextState.size() << std::endl;
	send ({reward});
	send (prevState);
	send (nextState);
	send (action);
//std::cout << "--- write reward end" << std::endl;
}

void SimpleClient::send (const std::vector<float>& data) {
	
	const char* b = reinterpret_cast<const char*>(data.data());
    int n = write(sockfd, b, data.size() * sizeof(float));
    if (n < 0) {
		std::cerr << "SimpleClient: ERROR writing to socket" << std::endl;
	}
}

bool SimpleClient::recv (std::vector<float>& data, int size) {
	data.resize (size);
//std::cout << "--- read action begin: " << size << std::endl;
	return readFromSocket (sockfd, (char*)data.data(), size * sizeof(float));
//std::cout << "--- read action end: " << readBytes << std::endl;
}

void SimpleClient::disconnect () {
	connected = false;
    close(sockfd);
	readThread.join();
}

void SimpleClient::setActionListener (const AgentActionListener& listener) {
	actionListener = listener;
}
