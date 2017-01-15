//
//  Observation.cpp
//  QuadrocopterBrain
//
//  Created by anton on 21/01/16.
//  Copyright © 2016 anton. All rights reserved.
//

#include "Observation.hpp"
#include <iostream>

Observation::Observation () {}

Observation::Observation (
	std::shared_ptr<std::vector<float>> data
) {
	this->data = data;
}

void Observation::set (
	std::shared_ptr<std::vector<float>> data
) {
	this->data = data;
}

void Observation::print () const {
	for (auto item : *data) {
		std::cerr << item << " ";
	}
	std::cerr << std::endl;
}

int Observation::getSize () const {
	return (int)data->size();
}

void Observation::setZeros (int len) {
	this->data.reset (new std::vector<float> (len, 0));
}
