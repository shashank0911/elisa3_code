#include "nodes.h"
#include <unistd.h>
#include <Eigen/Dense>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>
#include <fstream>
using std::cout;
using std::endl;

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "estimator_node");

    std::ifstream file("mapper.json");
    Json::Value mapper;
    Json::Reader reader;
    reader.parse(file, mapper);
    std::vector<std::string> activeRobots = mapper.getMemberNames();

    cout << "Start controller code" << endl;
    Nodes robots(activeRobots);
    int intensity[3] = {0, 10, 0};
    robots.setLeds(intensity);

    int j = 0;
    while (j < 3) {
        cout << " Waiting for odometry response" << endl;
        robots.move("still", 0.0, 0.0);
        robots.nodesReset("theor");
        cout << "Resetting odometer" << endl;
        double err = 0;
        for (const auto& tag : robots.nodes) {
            err += (tag.second->odomVals.head(2) - tag.second->curEst.head(2)).norm();
            cout << "Estimation: (" << tag.second->curEst(0) << ", " << tag.second->curEst(1) << endl;
        }
        if (err < 1e-6) {
            cout << "Reset done" << endl;
            break;
        }
        usleep(int(0.1 * 1000000.0));
        j++;
    }

    cout << " Start reset";
    while (robots.cameraMarker.measurementList.empty()) {
        usleep(int(0.05 * 1000000.0));
    }
    cout << "Awake from sleep" << endl;
    
    int lastSavedTime = 0;
    double stepSize = 1.0;
    double theta = 0.0;
    int t = 0;

    for (; t < 100; t++) {
        cout << "Loop number: " << t << endl;
        robots.storeData(t);
        robots.nodesLoopFn("move");
        if (t % 3 == 0) {
            robots.nodesReset("theor");
        }
    }

    robots.saveData(0);
        
    for (; t > 100 && t <= 110; t++) {
        robots.move("still", 0.0, 0.0);
        usleep(int(0.05 * 1000000.0));
    }

    cout << "Program has completed" << endl;

}