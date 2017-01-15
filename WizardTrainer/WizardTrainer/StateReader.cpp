//
//  StateReader.cpp
//  WizardTrainer
//
//  Created by anton on 12/11/2016.
//  Copyright © 2016 anton. All rights reserved.
//

/*
source: actor/input_layer/W_0:0 : readVariable_13:0 : (714, 1024)
source: actor/input_layer/b:0 : readVariable_14:0 : (1024,)
source: actor/hidden_layer_0/W_0:0 : readVariable_15:0 : (1024, 1024)
source: actor/hidden_layer_0/b:0 : readVariable_16:0 : (1024,)
source: actor/hidden_layer_1/W_0:0 : readVariable_17:0 : (1024, 768)
source: actor/hidden_layer_1/b:0 : readVariable_18:0 : (768,)
source: actor/hidden_layer_2/W_0:0 : readVariable_19:0 : (768, 512)
source: actor/hidden_layer_2/b:0 : readVariable_20:0 : (512,)
source: actor/hidden_layer_3/W_0:0 : readVariable_21:0 : (512, 256)
source: actor/hidden_layer_3/b:0 : readVariable_22:0 : (256,)
source: actor/hidden_layer_4/W_0:0 : readVariable_23:0 : (256, 6)
source: actor/hidden_layer_4/b:0 : readVariable_24:0 : (6,)
*/


#ifdef __gnu_linux__
#include <unistd.h>
#endif

#include <fstream>
#include <iostream>
#include <math.h>

#include "Tensors.hpp"

#define QUOTE(name) #name
#define STR(macro) QUOTE(macro)
#define GRAPHDIR STR(TF_GRAPH_DIR)
#define GRAPHSAVEDIR STR(TF_GRAPH_SAVE_DIR)

using namespace std;
using namespace tensorflow;

bool readGraphState ();

int main(int argc, const char * argv[]) {

    readGraphState ();

    return 0;
}

/*
void writeVector (int index, const std::vector<float>& v) {
    string varName = "vec_"+to_string(index);
    ofstream vfile ("tensor_values/" + varName + ".h");
    if (vfile.is_open ())
    {
	vfile << "std::vector<float> " << varName << ";" << endl;
	vfile << "void init_" << varName << " () {" << endl;
	for (int i=0; i<v.size (); i++) {
	    vfile << varName << "[" << i << "] = " << v[i] << ";" << endl;
	}
	vfile << "};" << endl;
	vfile.close ();
    }
}

void writeMatrix (int index, std::vector<vector<float>>& v) {
    string varName = "matrix_"+to_string(index);
    ofstream vfile ("tensor_values/" + varName + ".h");
    if (vfile.is_open ())
    {
	vfile << "std::vector<std::vector<float>> " << varName << ";" << endl;
	vfile << "void init_" << varName << " () {" << endl;
	for (int i=0; i<v.size (); i++)
	for (int j=0; j<v[0].size (); j++)
	{
	    vfile << varName << "[" << i << "][" << j << "] = " << v[i][j] << ";" << endl;
	}
	vfile << "};" << endl;
	vfile.close ();
    }
}
*/

void writeVector (int index, const std::vector<float>& v) {
    string varName = "vec_"+to_string(index);
    ofstream vfile ("tensor_values/" + varName + ".h");
    if (vfile.is_open ())
    {
        vfile << "const int " << varName << "_size = " << v.size () << ";" << endl;
        vfile << "const float " + varName + "[] = {" << endl;
        for (int i=0; i<v.size (); i++) {
            vfile << v[i];
            if (i != v.size()-1) vfile << ",";
            vfile << endl;
        }
        vfile << "};" << endl;
        vfile.close ();
    }
}

void writeMatrix (int index, std::vector<vector<float>>& v) {
    string varName = "matrix_"+to_string(index);
    ofstream vfile ("tensor_values/" + varName + ".h");
    if (vfile.is_open ())
    {
        vfile << "const int " << varName << "_size [] = {"
            << v.size () << ", " << v[0].size() << "};" << endl;
        vfile << "const float " << varName << " [] = {" << endl;
        for (int i=0; i<v.size (); i++){
//            vfile << "{ ";
            for (int j=0; j<v[i].size (); j++)
            {
                vfile << v[i][j];
                if (j != v[i].size()-1) vfile << ", ";
            }
//            vfile << "}";
            if (i != v.size ()-1) vfile << ",";
            vfile << endl;
        }
        vfile << "};" << endl;
        vfile.close ();
    }
}

bool readGraphState () {

	std::string dir (GRAPHSAVEDIR);
	std::fstream input(dir + "/graph-state", std::ios::in | std::ios::binary);
	
	if (!input.good ()) return false;
	
	std::vector<std::pair<string, tensorflow::Tensor>> variablesValues;
	std::vector<string> restoreOps;
	
	int variableCount;
	input.read(reinterpret_cast<char *>(&variableCount), sizeof(int));
	
std::cerr << "--- stored variables: " << variableCount << std::endl;

	for (int i=0; i<variableCount; i++) {
		int serializedTensorSize;
		input.read(reinterpret_cast<char *>(&serializedTensorSize), sizeof(int));
		std::string pStr;
		
		pStr.resize(serializedTensorSize);
		char* begin = &*pStr.begin();
		input.read(begin, serializedTensorSize);
		
		TensorProto p;
		p.ParseFromString (pStr);
		
		std::string variableSuffix = (i==0?"":"_"+std::to_string(i));
		variablesValues.push_back ({"variableValue" + variableSuffix, Tensor ()});
		Tensor& t (variablesValues.back ().second);
		t.FromProto (p);

		if (i>=13 && i<=24) {
		    if (t.dims () == 1) {
			vector<float> tValues;
			getTensorVectorValues (t, tValues);
std::cout << "--- read tensor: " << t.TotalBytes() << " vec: " << tValues.size() << std::endl;
			writeVector (i, tValues);
		    } else if (t.dims() == 2) {
			vector<vector<float>> tValues;
			getTensorValues (t, tValues);
std::cout << "--- read tensor: " << t.TotalBytes() << " matrix: " << tValues.size() << " x " << tValues[0].size() << std::endl;
			writeMatrix (i, tValues);
		    } else {
std::cout << "--- read tensor: " << serializedTensorSize << " / " << t.TotalBytes() << " Warning: dims: " << t.dims () <<  std::endl;
		    }
		}
		
//		restoreOps.emplace_back ("resoreVariable" + variableSuffix);
	}
	
	input.close ();
	
//	std::vector<tensorflow::Tensor> out;
//	Status status = session->Run(variablesValues, {}, restoreOps, &out);
//	if (!status.ok()) {
//		std::cout << "tf error2: " << status.ToString() << "\n";
//	}

	return true;
};
