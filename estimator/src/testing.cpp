// #include "nlohmann/json.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <map>
using std::cout;
using std::endl;
// using json = nlohmann::json;

// int main() {
//     std::ifstream file("../data/mapper.json");
//     json jf = json::parse(file);
//     std::vector<std::string> keys;
//     for (auto it = jf.begin(); it != jf.end(); it++) {
//         keys.push_back(it.key());
//     }
//     for (const auto& key : keys) {
//         cout << "Key: " << key << endl;
//     }

//     std::string address = jf["1"]["address"];
//     cout << "Address 1: " << address << endl;
//     double pos[2];
//     pos[0] = jf["1"]["pos"][0];
//     cout << "pos[0]: " << pos[0] << endl;

// }

// int main() {
//     std::ifstream file("home/shashank/catkin_ws/src/estimator/src/mapper.json");
//     json mapper = json::parse(file);
//     std::vector<std::string> activeRobots;
//     for (auto it = mapper.begin(); it != mapper.end(); it++) {
//         activeRobots.push_back(it.key());
//     }
//     cout << "Active robots" << activeRobots.size();
// }


int main() {
    std::map<int, double> timeVector;

    for (int i = 0; i < 100; i++) {
        double sum = 0.0;
        auto startTime = std::chrono::high_resolution_clock::now();
        for (int j = 0; j <= 1000000; j++) {
            sum += double(j);
        }
        auto endTime = std::chrono::high_resolution_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();
        // timeVector[i] = duration;
        std::chrono::duration<double> duration = endTime - startTime;
        timeVector[i] = duration.count();
    }
    std::ofstream file("profiling.csv");
    for (const auto& data: timeVector) {
        file << data.first << "," << data.second << "\n";
    }
}