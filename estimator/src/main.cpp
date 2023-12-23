#include "nodes.h"
#include <unistd.h>
#include <Eigen/Dense>
#include "nlohmann/json.hpp"
#include <fstream>
using std::cout;
using std::endl;
using json = nlohmann::json;

ros::Publisher publisherCams;
ros::Publisher publisherAutoMove;
ros::Publisher publisherLeds;
ros::Publisher publisherReset;

ros::Subscriber listenerCameraList;
std::map<int, ros::Subscriber> listenerCamera;
std::map<int, ros::Subscriber> listenerCameraTimer;
std::map<std::string, ros::Subscriber> listenerRobotPose;
std::map<std::string, ros::Subscriber> listenerAccel;

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "estimator");
    ros::NodeHandle np("~");
    ros::NodeHandle n;

    std::ifstream file("src/estimator/src/mapper.json");
    if (file.is_open()) {
        json mapper = json::parse(file);
        std::vector<std::string> activeRobots;
        for (auto it = mapper.begin(); it != mapper.end(); it++) {
            activeRobots.push_back(it.key());
        }
        // cout << "Active robots: " << activeRobots.size() << endl;;

        cout << "Start estimator code" << endl;
        Nodes robots(activeRobots);

        // Initialize publishers
        publisherCams = np.advertise<std_msgs::Float64MultiArray>("elisa3_all_robots/cams", 1);
        publisherAutoMove = np.advertise<std_msgs::Float64MultiArray>("elisa3_all_robots/auto_motive", 1);
        publisherLeds = np.advertise<std_msgs::Float64MultiArray>("elisa3_all_robots/leds", 1);
        publisherReset = np.advertise<std_msgs::Float64MultiArray>("elisa3_all_robots/reset", 1);

        // Initialize subscribers
        listenerCameraList = n.subscribe("Bebop1/makers", 10, &CameraMarker::listenOptitrackMarkersCallback, &robots.cameraMarker);
        for (const auto& tag : robots.nodes) {
            listenerRobotPose[tag.first] = n.subscribe("elisa3_robot_" + tag.first + "/odom", 10, &Node::listenRobotPoseCallback, tag.second);
            listenerAccel[tag.first] = n.subscribe("swarm/elisa3_robot_" + tag.first + "/accel", 10, &Node::listenAccelCallback, tag.second);
        }
        for (const auto& cam : robots.cameras.cameras) {
            listenerCamera[cam.first] = n.subscribe("Bebop" + std::to_string(cam.first + 1) + "/ground_pose", 10, &CameraNode::listenOptitrackCallback, cam.second);
            listenerCameraTimer[cam.first] = n.subscribe("Bebop" + std::to_string(cam.first + 1) + "/pose", 10, &CameraNode::listenOptitrackTimerCallback, cam.second);
        }

        usleep(3*1000000);
        // Loop to check if messages are arriving
        // int temp = 0;
        // while (ros::ok()) {
        //     if (temp > 2) {
        //         break;
        //     }
        //     usleep(int(1.0*1000000.0));
        //     temp += 1;
        //     ros::spinOnce();
        // }

        int intensity[3] = {0, 10, 0};
        robots.setLeds(intensity);
        publisherLeds.publish(robots.msgLeds);
        usleep(int(0.005 * 1000000.0));

        int j = 0;
        while (ros::ok()) {
            if (j < 3) {
                cout << endl << "Loop " << j << endl;
                cout << "Waiting for odometry response" << endl;

                robots.move("still", 0.0, 0.0);
                publisherAutoMove.publish(robots.msgAutoMove);
                usleep(int(0.005 * 1000000.0));

                robots.nodesReset("theor");
                publisherReset.publish(robots.msgReset);
                // usleep(int(0.005 * 1000000.0));
                cout << "Resetting odometer" << endl;

                double err = 0;
                for (const auto& tag : robots.nodes) {
                    err += (tag.second->odomVals.head(2) - tag.second->curEst.head(2)).norm();
                    cout << "Estimation for robot " << tag.first << ": (" << tag.second->curEst(0) << ", " << tag.second->curEst(1) << ")" << endl;
                }
                if (err < 1e-6) {
                    cout << "Reset done" << endl;
                    break;
                }
                cout << "Sleep start" << endl;
                usleep(int(0.1 * 1000000.0));
                cout << "Sleep done" << endl;
                j += 1;
            } else {
                break;
            }

            ros::spinOnce();
        }
        // while (j < 3) {
        //     cout << endl << "Loop " << j << endl;
        //     cout << "Waiting for odometry response" << endl;

        //     robots.move("still", 0.0, 0.0);
        //     publisherAutoMove.publish(robots.msgAutoMove);

        //     robots.nodesReset("theor");
        //     publisherReset.publish(robots.msgReset);
        //     cout << "Resetting odometer" << endl;

        //     double err = 0;
        //     for (const auto& tag : robots.nodes) {
        //         err += (tag.second->odomVals.head(2) - tag.second->curEst.head(2)).norm();
        //         cout << "Estimation for robot " << tag.first << ": (" << tag.second->curEst(0) << ", " << tag.second->curEst(1) << ")" << endl;
        //     }
        //     if (err < 1e-6) {
        //         cout << "Reset done" << endl;
        //         break;
        //     }
        //     cout << "Sleep start" << endl;
        //     usleep(int(0.1 * 1000000.0));
        //     cout << "Sleep done" << endl;
        //     j += 1;
        // }

        cout << "Start reset" << endl;
        // cout << "Skipping empty camera marker loop" << endl;
        while (ros::ok()) {    
            if (robots.cameraMarker.measurementList.size() != 0) {
                break;
            }
            
            // publisherLeds.publish(robots.msgLeds);
            // publisherAutoMove.publish(robots.msgAutoMove);
            // publisherReset.publish(robots.msgReset);
            usleep(int(0.005 * 1000000.0));
            ros::spinOnce();
        }
        // while (robots.cameraMarker.measurementList.size() == 0) {
        //     usleep(int(0.05 * 1000000.0));
        //     // cout << "still searching \t";
        // }
        // cout << "Awake from sleep" << endl;
        
        // int lastSavedTime = 0;
        double stepSize = 1.0;
        double theta = 0.0;
        int t = 0;

        while (ros::ok()) {
            if (t < 200) {
                cout << "Main loop number: " << t << endl;
                
                robots.nodesLoopFn("move");
                publisherAutoMove.publish(robots.msgAutoMove);
                usleep(int(0.005 * 1000000.0));

                if (t % 3 == 0) {
                    robots.nodesReset("theor");
                    publisherReset.publish(robots.msgReset);
                    // usleep(int(0.005 * 1000000.0));
                    // robots.storeData(t);
                }

                t += 1;
            }

            // robots.saveData(0);
                
            if (t >= 200 && t < 250) {
                cout << "Sub loop number: " << t << endl;
                robots.move("still", 0.0, 0.0);
                publisherAutoMove.publish(robots.msgAutoMove);
                usleep(int(0.005 * 1000000.0));

                t += 1;
            }

            // cout << "Program has completed" << endl;
            ros::spinOnce();
        }
        

    } else {
        cout << "Mapper.json cannot be opened. Check file path" << endl;
    }
    cout << "Program exited" << endl;
}