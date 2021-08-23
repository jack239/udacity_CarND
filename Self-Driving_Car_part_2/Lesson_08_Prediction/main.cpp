#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "classifier.h"

// Helper functions to load .txt files
std::vector<std::vector<double> > Load_State(std::string file_name);
std::vector<std::string> Load_Label(std::string file_name);

int main() {
    std::vector< std::vector<double> > X_train = Load_State(std::string(DATA_FOLDER) + "/train_states.txt");
    std::vector< std::vector<double> > X_test  = Load_State(std::string(DATA_FOLDER) + "/test_states.txt");
    std::vector< std::string > Y_train = Load_Label(std::string(DATA_FOLDER) + "/train_labels.txt");
    std::vector< std::string > Y_test  = Load_Label(std::string(DATA_FOLDER) + "/test_labels.txt");

    std::cout << "X_train number of elements " << X_train.size() << std::endl;
    std::cout << "X_train element size " << X_train[0].size() << std::endl;
    std::cout << "Y_train number of elements " << Y_train.size() << std::endl;

    GNB gnb = GNB();

    gnb.train(X_train, Y_train);

    std::cout << "X_test number of elements " << X_test.size() << std::endl;
    std::cout << "X_test element size " << X_test[0].size() << std::endl;
    std::cout << "Y_test number of elements " << Y_test.size() << std::endl;

    int score = 0;
    for (int i = 0; i < X_test.size(); ++i) {
        std::vector<double> coords = X_test[i];
        std::string predicted = gnb.predict(coords);
        if (predicted.compare(Y_test[i]) == 0) {
            score += 1;
        }
    }

    float fraction_correct = float(score) / Y_test.size();
    std::cout << "You got " << (100*fraction_correct) << " correct" << std::endl;

    return 0;
}

// Load state from .txt file
std::vector<std::vector<double> > Load_State(std::string file_name) {
    std::ifstream in_state_(file_name.c_str(), std::ifstream::in);
    std::vector< std::vector<double >> state_out;
    std::string line;

    while (getline(in_state_, line)) {
        std::istringstream iss(line);
        std::vector<double> x_coord;

        std::string token;
        while (getline(iss,token,',')) {
            x_coord.push_back(stod(token));
        }
        state_out.push_back(x_coord);
    }

    return state_out;
}

// Load labels from .txt file
std::vector<std::string> Load_Label(std::string file_name) {
    std::ifstream in_label_(file_name.c_str(), std::ifstream::in);
    std::vector< std::string > label_out;
    std::string line;
    while (getline(in_label_, line)) {
        std::istringstream iss(line);
        std::string label;
        iss >> label;

        label_out.push_back(label);
    }

    return label_out;
}