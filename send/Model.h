#include <random>

#include "matrix_13.h"
#include "matrix_15.h"
#include "matrix_17.h"
#include "matrix_19.h"
#include "matrix_21.h"
#include "matrix_23.h"

#include "vec_14.h"
#include "vec_16.h"
#include "vec_18.h"
#include "vec_20.h"
#include "vec_22.h"
#include "vec_24.h"

std::default_random_engine randomGenerator;
std::normal_distribution<float> normalNoise (0, 0.05);

void matMul (const std::vector<float>& vec, const float matrix[], const int mSize[], std::vector<float>& result)
{
    int s0 = mSize [0];
    int s1 = mSize [1];
    
    result.resize (s1);
    for (int j=0; j<s1; j++) {
        float component = 0;
        for (int i=0; i<s0; i++) {
            component += vec [i] * matrix [i * s1 + j];
        }
        result [j] = component;
    }
}

void addVec (const float vec[], const int vSize, std::vector<float>& result) {
    for (int i=0; i<vSize; i++) {
        result [i] += vec [i];
    }
}

float relu (float v) {
    return v>0?v:0;
}

void reluVec (std::vector<float>& result) {
    for (auto& v : result) {
        v = relu (v);
    }
}

void tanhVec (std::vector<float>& result) {
    for (auto& v : result) {
        v = tanh (v);
    }
}

void modelAction (const std::vector<float>& state, std::vector<float>& action) {
    std::vector<float> in;

    matMul (state, matrix_13, matrix_13_size, action);
    addVec (vec_14, vec_14_size, action);
    reluVec (action);
    
    in = action;
    matMul (in, matrix_15, matrix_15_size, action);
    addVec (vec_16, vec_16_size, action);
    reluVec (action);
    
    in = action;
    matMul (in, matrix_17, matrix_17_size, action);
    addVec (vec_18, vec_18_size, action);
    reluVec (action);
    
    in = action;
    matMul (in, matrix_19, matrix_19_size, action);
    addVec (vec_20, vec_20_size, action);
    reluVec (action);
    
    in = action;
    matMul (in, matrix_21, matrix_21_size, action);
    addVec (vec_22, vec_22_size, action);
    tanhVec (action);
    
    in = action;
    matMul (in, matrix_23, matrix_23_size, action);
    addVec (vec_24, vec_24_size, action);
    
    for (auto& a : action) {
	a += normalNoise (randomGenerator);
    }
}
