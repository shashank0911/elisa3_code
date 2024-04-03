#include "nodes.h" 
#include "utils.h"
#include "Kalman.h"
#include <iostream>
#include <fstream>
#include "nlohmann/json.hpp"
#include <unistd.h>
#include <cmath>

using std::string;
using std::cout;
using std::endl;
using json = nlohmann::json;

const double SAMPLING_TIME = 0.06;
const double PI = 3.14159265358979323846;

const string MODE = "kalman";

const int SR_KALMAN = 0;
const int MR_KALMAN = 1;





CameraNode::CameraNode(int tag_ext) {
    camX = 0.0;
    camY = 0.0;
    camPhi = 0.0;
    tag = tag_ext;
    timer = 0.0;
}

void CameraNode::listenOptitrackCallback(const geometry_msgs::Pose2D::ConstPtr& optiMsg) {
    camX = optiMsg->x;
    camY = optiMsg->y;
    camPhi = optiMsg->theta;
}

void CameraNode::listenOptitrackTimerCallback(const geometry_msgs::PoseStamped::ConstPtr& optiMsg) {
    timer = optiMsg->header.stamp.toSec();
}

CameraNode::~CameraNode() {
    cout << "Individual camera nodes destroyed" << endl;
}



Cameras::Cameras(int N) {
    number = N;
    for (int tag = 0; tag < N; tag++) {
        cameras[tag] = new CameraNode(tag);
    }
    measurementList.resize(N, 4);
    msgCam.data.resize(N*3 + 1);
    
}

void Cameras::updateCamera() {
    int i = 0;
    for (const auto& camera: cameras) {
        measurementList(i,0) = camera.second->camX;
        measurementList(i,1) = camera.second->camY;
        measurementList(i,2) = camera.second->camPhi;
        measurementList(i,3) = camera.second->timer;
        i++;
    }
}

void Cameras::publishCams() {
    int count = 0;
    for (int i = 0; i < number; i++) {
        msgCam.data[i*3 + 1] = measurementList(i, 0);
        msgCam.data[i*3 + 2] = measurementList(i, 1);
        msgCam.data[i*3 + 3] = measurementList(i, 2);
        count++;
    }
    msgCam.data[0] = count;
}

Cameras::~Cameras() {
    for (const auto& camera : cameras) {
        delete camera.second;
    }
    cout << "Cameras group node deleted" << endl;
}






Node::Node(double releaseTime, std::string tagExt) {
    std::ifstream file("src/estimator/src/mapper.json");
    json mapper = json::parse(file);
    
    t = 0;

    tag = tagExt;
    this->releaseTime = releaseTime;

    address = mapper[tag]["address"];
    startPos[0] = mapper[tag]["pos"][0];
    startPos[1] = mapper[tag]["pos"][1];
    startOrien = mapper[tag]["orien"];

    inputV = 0.0;
    inputOmega = 0.0;

    xyphiVals << startPos[0], startPos[1], startOrien;

    odomVals = xyphiVals;
    odomTimer = 0.0;

    camVals = xyphiVals;
    camTimer = 0.0;

    accelVals.resize(5);
    accelVals << 0.0, 0.0, 0.0, xyphiVals(0), xyphiVals(1);

    camPrev = xyphiVals;
    minDistPrev = 0.0;

    theoPosn = xyphiVals;
    prevEst = xyphiVals;
    curEst = xyphiVals;

    threshold = 0.05;

    bufferSize = 5;

    odomErrorBuffer.resize(bufferSize);
    odomErrorBuffer.fill(0.1);
    camErrorBuffer.resize(bufferSize);
    camErrorBuffer.fill(0.1);
    accelErrorBuffer.resize(bufferSize);
    accelErrorBuffer.fill(0.1);

    OwaWeights.fill(0.0);

    goal << 0.0, 0.0;
    setup = {
        {"vMax", 0.5},
        {"gtgScaling", 0.0001},
        {"Kp", 0.01},
        {"ao_scaling", 0.00005}
    };

    posBuf.resize(2, bufferSize);
    for (int i = 0; i < bufferSize; i++) {
        posBuf.col(i) << startPos[0], startPos[1];
    }
    orienBuf.resize(1, bufferSize);
    orienBuf.fill(startOrien);

    updateLeds = false;
    for (int i = 0; i < 3; i++) {
        msgLeds[i] = 0;
    }

    updateReset = false;
    for (int i = 0; i < 4; i++) {
        msgReset[i] = 0.0;
        msgAutoMove[i] = 0.0;
    }
}

void Node::publishGreenLed(int intensity) {
    updateLeds = true;
    msgLeds[0] = intensity;
}

void Node::publishRedLed(int intensity) {
    updateLeds = true;
    msgLeds[1] = intensity;
}

void Node::publishBlueLed(int intensity) {
    updateLeds = true;
    msgLeds[2] = intensity;
}

void Node::ledOff() {
    publishGreenLed(0);
    publishRedLed(0);
    publishBlueLed(0);
}

void Node::listenRobotPoseCallback(const nav_msgs::Odometry::ConstPtr& odomMsg) {
    odomVals << odomMsg->pose.pose.position.x, 
                odomMsg->pose.pose.position.y,
                odomMsg->pose.pose.position.z;
    odomTimer = odomMsg->header.stamp.toSec();
    // cout << "Received odom msgs" << endl;

}

void Node::listenAccelCallback(const sensor_msgs::Imu::ConstPtr& accelMsg) {
    accelVals(0) = accelMsg->linear_acceleration.x;
    accelVals(1) = accelMsg->linear_acceleration.y;
    accelVals(2) = accelMsg->angular_velocity.x;
    // cout << "Received accel msgs" << endl;
}

void Node::nodePrintPositionMeasures() {
    std::stringstream msg;

    msg << "\nID: " << tag << "\n"
        << "Estimation - Position: (" << curEst(0) << ", " << curEst(1) << "), Orientation: " << curEst(2) << "\n"
        << "Odometry - Position: (" << odomVals(0) << ", " << odomVals(1) << "), Orientation: " << odomVals(2) << "\n"
        << "Camera - Position: (" << camVals(0) << ", " << camVals(1) << "), Orientation: " << camVals(2) << "\n"
        << "Accelerometer - Position: (" << accelVals(3) << ", " << accelVals(4) << "), Velocity: " << accelVals(0) << "\n";

    if (t % 4 == 0) {
        cout << msg.str() << endl;
    }
    
}

void Node::computeMove(double pol[2]) {
    nodePrintPositionMeasures();

    double phiOld = orienBuf(bufferSize - 1);
    if (phiOld < 0.0) {
        phiOld += 2*PI;
    }
    phiOld = fmod(phiOld, 2*PI);
    if (phiOld > PI) {
        phiOld -= 2*PI;
    }

    double phiNew = pol[1];

    Eigen::Vector2d polVec;
    polVec << pol[0], phiNew;
    Eigen::Vector2d propMoveCart = pol2cart(polVec) * SAMPLING_TIME / 100.0;
    cout << "Robot tag: " << address << endl;
    auto [calcPoint, obsAvoidMode] = obstacleAvoidanceNode.obstacleAvoidance(posBuf.col(bufferSize - 1), propMoveCart);
    Eigen::Vector2d newMove = calcPoint - posBuf.col(bufferSize - 1);
    

    double phiFin = std::atan2(newMove(1), newMove(0));

    if (phiFin >= 0.0) {
        // turn left
        msgAutoMove[0] = 1.0;
        msgAutoMove[1] = phiFin;
        msgAutoMove[2] = 0.0;
        msgAutoMove[3] = newMove.norm() * 100.0 / SAMPLING_TIME;
        cout << "Turn left with translational speed: " << msgAutoMove[3]*50.0/30.0 << ", \tangular speed: " << msgAutoMove[1]*10.0 << " rad, \t" << msgAutoMove[1]*57.3 << " deg" << endl;
    } else {
        // turn right
        msgAutoMove[0] = 0.0;
        msgAutoMove[1] = -phiFin;
        msgAutoMove[2] = 0.0;
        msgAutoMove[3] = newMove.norm() * 100.0 / SAMPLING_TIME;
        cout << "Turn right with translational speed: " << msgAutoMove[3]*50.0/30.0 << ", \tanglular speed: " << msgAutoMove[1]*10.0 << " rad, \t" << msgAutoMove[1]*57.3 << "deg" << endl;
    } 
}

void Node::nodeReset(const std::string type) {
    if (type == "odom") {
        if (odomVals.hasNaN()) {
            cout << "Robot " << tag << "'s odometry measurement outputs NaN" << endl;
        } else {
            posBuf.col(bufferSize - 1) << odomVals(0), odomVals(1);
            orienBuf(bufferSize - 1) == odomVals(2);
        }
        updateReset = false;
    } else if (type == "theor") {
        updateReset = true;
        msgReset[0] = 0;
        msgReset[1] = curEst(0);
        msgReset[2] = curEst(1);
        msgReset[3] = curEst(2);
        accelVals[3] = curEst(0);
        accelVals[4] = curEst(1);
    }

}

void Node::statesTransform() {
    curEst(0) += inputV * std::cos(curEst(2)) * SAMPLING_TIME / 100.0;
    curEst(1) += inputV * std::sin(curEst(2)) * SAMPLING_TIME / 100.0;
    curEst(2) += inputOmega;
}

std::pair<double, double> Node::goToGoal() {
    Eigen::Vector2d err;
    err << goal - curEst.head(2);
    double Kp1 = 50.0;    
    double v = Kp1*err.norm();
    double phi = std::atan2(err(1), err(0));
    double Kp2 = 0.5;
    double omega = Kp2 * std::atan2( std::sin(phi - curEst(2)), std::cos(phi - curEst(2)) );
    cout << "\nDistance from goal (" << address << "): " << err.norm() << "\tv: " << v << "\tomega: " << omega << endl;
    return std::make_pair(v, omega);
}

bool Node::terminate() {
    double maxLength = 0.015;
    double distance = (goal - curEst.head(2)).norm();
    if (distance < maxLength) {
        return 1;
    } else {
        return 0;
    }
}

Eigen::Vector3d Node::determineCamera(Cameras& cameras) {
    double minDist = 1e5;
    cameras.updateCamera();

    int idx_og = std::stoi(tag);
    camVals << cameras.measurementList(idx_og-1, 0), cameras.measurementList(idx_og-1, 1), cameras.measurementList(idx_og-1, 2);
    camTimer = cameras.measurementList(idx_og-1, 3);

    double dist_og = (curEst.head(2) - camVals.head(2)).norm();

    if (dist_og < 0.075) {
        return camVals;
    } 

    int i = 0;
    int idx = 0;
    minDist = dist_og;
    double dist = 0;
    for (int j = 0; j < cameras.measurementList.rows(); j++) {
        if (i == std::stoi(tag)-1) {
            i += 1;
            continue;
        }
        Eigen::Vector2d temp;
        temp << cameras.measurementList(j,0), cameras.measurementList(j,1);
        dist = (curEst.head(2) - temp).norm();
        // cout << "Cam msmt for " << j+1 << "th robot: (" << cameras.measurementList(j,0) << ", " << cameras.measurementList(j,1) << ")\n";
        // cout << "Dist from cam btw curEst and " << j+1 << "th robot cam msmt: " << dist << endl;

        if (dist < minDist) {
            minDist = dist;
            idx = i;
        }
        i += 1;
    }

    if (minDist == dist_og) {
        camVals << cameras.measurementList(idx_og-1, 0), cameras.measurementList(idx_og-1, 1),cameras.measurementList(idx_og-1, 2);
        camTimer = cameras.measurementList(idx_og-1, 3);
    } else {
        camVals << cameras.measurementList(idx, 0), cameras.measurementList(idx, 1), cameras.measurementList(idx, 2);
        camTimer = cameras.measurementList(idx, 3);
    }

    if (minDist > (curEst.head(2) - odomVals.head(2)).norm()) {
        return odomVals;
    } else {
        if (t < 10) {
            return camVals;
        } else if (minDist > 0.10) {
            return odomVals;
        } else {
            return camVals;
        }

    }
}

//returns odom, accel and cam values, correspondingly in each row
Eigen::Matrix3d Node::measurementUpdate(Cameras& cameras) {
    accelVals(3) += accelVals(0) * std::cos(curEst(2)) * SAMPLING_TIME * SAMPLING_TIME;
    accelVals(4) += accelVals(0) * std::sin(curEst(2)) * SAMPLING_TIME * SAMPLING_TIME;
    Eigen::Vector3d accelMsmt;
    accelMsmt << accelVals(3), accelVals(4), curEst(2);

    Eigen::Vector3d camMsmt = determineCamera(cameras);

    Eigen::Matrix3d returnVal;
    returnVal << odomVals.transpose(), accelMsmt.transpose(), camMsmt.transpose();
    return returnVal; 
}

void Node::measurementFusionOWA(Eigen::Matrix3d measurements) {
    if (MODE == "CAM") {
        curEst = measurements.row(2);
    }

    //fuse with odometry
    kalmanOdom.Rk << 1.0, 0.0, 0.0,
                     0.0, 1.0, 0.0,
                     0.0, 0.0, 1.0;
    kalmanOdom.Qk << 0.01, 0.0, 0.0,
                     0.0, 0.01, 0.0,
                     0.0, 0.0, 0.01;
    auto [optimalStateEstimateOdom, covarianceEstimateOdom] = kalmanOdom.srEKF(measurements.row(0), curEst, 1);
    Eigen::Vector3d odomEst = optimalStateEstimateOdom;
    kalmanOdom.Pk1 = covarianceEstimateOdom;

    //fuse with acceleroemeter
    kalmanAccel.Rk << 0.5, 0.0, 0.0,
                      0.0, 0.5, 0.0,
                      0.0, 0.0, 0.5;
    kalmanAccel.Qk << 0.01, 0.0, 0.0,
                      0.0, 0.01, 0.0,
                      0.0, 0.0, 0.01;
    auto [optimalStateEstimateAccel, covarianceEstimateAccel] = kalmanAccel.srEKF(measurements.row(1), curEst, 1);
    Eigen::Vector3d accelEst = optimalStateEstimateAccel;
    kalmanAccel.Pk1 = covarianceEstimateAccel;

    //fuse with camera
    double errOdom = (curEst.head(2) - odomVals.head(2)).norm();
    double errAccel = (curEst.head(2) - accelVals.tail(2)).norm();
    
    odomErrorBuffer.head(bufferSize - 1) = odomErrorBuffer.tail(bufferSize - 1);
    odomErrorBuffer(bufferSize - 1) = errOdom;
    accelErrorBuffer.head(bufferSize - 1) = accelErrorBuffer.tail(bufferSize - 1);
    accelErrorBuffer(bufferSize - 1) = errAccel;

    double offset = 0.0;
    double sumOdom = odomErrorBuffer.sum();
    double sumAccel = accelErrorBuffer.sum();
    double tempOdom = (sumOdom + sumAccel) / sumOdom;
    double tempAccel = (sumOdom + sumAccel) / sumAccel;
    double tempSum = tempOdom + tempAccel + offset;
    OwaWeights << tempOdom/tempSum, 0.0, tempAccel/tempSum;
    curEst = OwaWeights(0) * odomEst + OwaWeights(2) * accelEst;    

    if (t % 1 == 0) {
        if (SR_KALMAN && !MR_KALMAN) {
            auto[optimalStateEstimateCam, covarianceEstimateCam] = kalmanCam.srEKF(measurements.row(2), curEst, 1);
            curEst = optimalStateEstimateCam;
            kalmanCam.Pk1 = covarianceEstimateCam;
        } else if (MR_KALMAN && !SR_KALMAN) {
            auto[optimalStateEstimateCam, covarianceEstimateCam] = kalmanCam.mrEKF(measurements.row(2), curEst, 1);
            curEst = optimalStateEstimateCam;
            kalmanCam.Pk1 = covarianceEstimateCam;
        }

        Eigen::Vector3d camEst = curEst;
        double errCam = (curEst.head(2) - camVals.head(2)).norm();
        camErrorBuffer.head(bufferSize - 1) = camErrorBuffer.tail(bufferSize - 1);
        camErrorBuffer(bufferSize - 1) = errCam;

        double sumCam = camErrorBuffer.sum();

        offset = 0.9;
        double sum1 = sumCam + sumOdom + sumAccel;
        tempAccel = sum1/sumAccel;
        //change if necessary
        tempAccel = 0.0;
        tempOdom = sum1/sumOdom;
        double tempCam = sum1/sumCam;
        tempSum = tempOdom + tempAccel + tempCam + offset;
        OwaWeights << tempOdom/tempSum, (tempCam + offset)/tempSum, tempAccel/tempSum;
        curEst = OwaWeights(0) * odomEst + OwaWeights(1) * camEst + OwaWeights(2) * accelEst;
    }
    
}

void Node::nodeLoopFun(Cameras& cameras, const std::string moveType) {
    t += 1;

    Eigen::Matrix3d measurements = measurementUpdate(cameras);

    statesTransform();
    measurementFusionOWA(measurements);

    prevEst << curEst;
    posBuf = renewVec(posBuf);
    orienBuf = renewVec(orienBuf);
    posBuf.col(bufferSize - 1) << curEst(0), curEst(1);
    orienBuf(bufferSize - 1) = curEst(2); 

    if (!terminate()) {
        auto [step, omega] = goToGoal();
        inputV = step;
        inputOmega = omega;
    } else {
        Eigen::Vector2d err = goal - curEst.head(2);
        cout << "\nDistance from goal (" << address << "): " << err.norm() << endl;
        inputV = 0.0;
        inputOmega = 0.0;
    }
    double pol[2] = {inputV, inputOmega};
    computeMove(pol);

}

Node::~Node() {
    cout << "All individual nodes destroyed" << endl;
}








Nodes::Nodes(std::vector<std::string> activeRobots) : cameras(activeRobots.size()) {

    std::ifstream file("src/estimator/src/mapper.json");
    json mapper = json::parse(file);
    std::vector<std::string> keys;
    for (auto it = mapper.begin(); it != mapper.end(); it++) {
        keys.push_back(it.key());
    }

    this->activeRobots = activeRobots;
    N = activeRobots.size();
    double releaseTime = 0.0;
    for (const string& tag : activeRobots) {
        bool flag = false;
        if (!keys.empty()) {
            for (const string& key : keys) {
                if (key == tag) {
                    flag = true;
                    break;
                }
            }
            if (flag == true) {
                nodes[tag] = new Node(releaseTime, tag);
            } else {
                cout << "Robot with address " << tag << " not found in mapper.json" << endl; 
            }
        } else {
            cout << "No robots found in JSON list" << endl;
        }         
    }

    msgAutoMove.data.resize(N*5 + 1, 0.0);
    msgLeds.data.resize(N*4 + 1, 0.0);
    msgReset.data.resize(N*5 + 1, 0.0);

}

void Nodes::nodesLoopFn(const std::string moveType) {
    int i = 0;
    int count = 0;
    for (const auto& node: nodes) {
        nodes[node.first]->nodeLoopFun(cameras, moveType);
        msgAutoMove.data[i*5 + 1] = std::stod(node.second->address);
        msgAutoMove.data[i*5 + 2] = node.second->msgAutoMove[0];
        msgAutoMove.data[i*5 + 3] = node.second->msgAutoMove[1];
        msgAutoMove.data[i*5 + 4] = node.second->msgAutoMove[2];
        msgAutoMove.data[i*5 + 5] = node.second->msgAutoMove[3];
        i++;
        count++;
    }
    msgAutoMove.data[0] = double(count);
}

void Nodes::testCam() {
    cameras.updateCamera();
    for (int i = 0; i < N; i++) {
        cout << "Camera " << i + 1 << " data" << endl;
        cout << "CamY: " << cameras.measurementList(i,0) << "\t";
        cout << "CamX: " << cameras.measurementList(i,1) << "\t";
        cout << "CamPhi: " << cameras.measurementList(i,2) << "\t";
        cout << "CamTimer: " << cameras.measurementList(i,3) << "\n";
    }

    for (const auto& tag : nodes) {
        Eigen::Vector3d camMeasurement = nodes[tag.first]->determineCamera(cameras);
        cout << "Nearest camera for robot " << tag.first << endl;
        cout << "CamX: " << camMeasurement(0) << "\t";
        cout << "CamY: " << camMeasurement(1) << "\t";
        cout << "CamPhi: " << camMeasurement(2) << "\n";
    }
}

void Nodes::printFn() {
    for (const auto& tag : nodes) {
        nodes[tag.first]->nodePrintPositionMeasures();
    }
}

void Nodes::move(const std::string moveType, double stepSize, double theta) {
    double step = 0.0;
    double omega = 0.0;
    int i = 0;
    int count = 0;
    for (const auto& tag : nodes) {
        if (moveType == "move") {
            auto [step, omega] = nodes[tag.first]->goToGoal();
        } else {
            step = stepSize;
            omega = theta;
        }
        double pol[2] = {step, omega};
        nodes[tag.first]->computeMove(pol);

        msgAutoMove.data[i*5 + 1] = std::stod(tag.second->address);
        msgAutoMove.data[i*5 + 2] = tag.second->msgAutoMove[0];
        msgAutoMove.data[i*5 + 3] = tag.second->msgAutoMove[1];
        msgAutoMove.data[i*5 + 4] = tag.second->msgAutoMove[2];
        msgAutoMove.data[i*5 + 5] = tag.second->msgAutoMove[3];
        i++;
        count++;
    }
    msgAutoMove.data[0] = double(count);
}

void Nodes::updateLeds() {
    int count = 0;
    int i = 0;
    for (const auto& tag : nodes) {
        msgLeds.data[i*4 + 1] = std::stod(tag.second->address);
        msgLeds.data[i*4 + 2] = double(nodes[tag.first]->msgLeds[0]);
        msgLeds.data[i*4 + 3] = double(nodes[tag.first]->msgLeds[1]);
        msgLeds.data[i*4 + 4] = double(nodes[tag.first]->msgLeds[2]);
        i++;
        count++;
    }
    msgLeds.data[0] = double(count);
}

void Nodes::nodesReset(const std::string type) {
    int count = 0;
    int i = 0;
    for (const auto& tag : nodes) {
        nodes[tag.first]->nodeReset(type);
        if (nodes[tag.first]->updateReset) {
            msgReset.data[i*5 + 1] = std::stod(tag.second->address);
            msgReset.data[i*5 + 2] = nodes[tag.first]->msgReset[0];
            msgReset.data[i*5 + 3] = nodes[tag.first]->msgReset[1];
            msgReset.data[i*5 + 4] = nodes[tag.first]->msgReset[2];
            msgReset.data[i*5 + 5] = nodes[tag.first]->msgReset[3];
            i++;
            count++;
        }
    }

    if (count != 0) {
        cout << "Reset - theoretical" << endl;
        msgReset.data[0] = double(count);
    }
}

void Nodes::turnOffLeds() {
    for (const auto& tag : nodes) {
        nodes[tag.first]->ledOff();
    }
    updateLeds();
}

void Nodes::setLeds(const int ledIntensity[3]) {
    for (const auto& tag : nodes) {
        
        nodes[tag.first]->publishGreenLed(ledIntensity[0]);
        nodes[tag.first]->publishRedLed(ledIntensity[1]);
        nodes[tag.first]->publishBlueLed(ledIntensity[2]);
    }
    updateLeds();
}

void Nodes::nodesPrintPositionMeasures() {
    for (const auto& tag : nodes) {
        nodes[tag.first]->nodePrintPositionMeasures();
    }
}

void Nodes::storeData(int t) {
    std::vector<double> robotInst;
    for (const auto& node: nodes) {
        std::vector<double> robotInst { double(t),
            std::stod(node.first),
            node.second->odomVals(0),
            node.second->odomVals(1),
            node.second->odomVals(2),
            node.second->curEst(0),
            node.second->curEst(1),
            node.second->curEst(2),
            node.second->camVals(0),
            node.second->camVals(1),
            node.second->camVals(2),
            node.second->theoPosn(0),
            node.second->theoPosn(1),
            node.second->theoPosn(2),
            node.second->odomTimer,
            node.second->camTimer,
            node.second->OwaWeights(0),
            node.second->OwaWeights(1),
            node.second->OwaWeights(2),
            node.second->accelVals(0),
            node.second->accelVals(2),
            node.second->accelVals(3),
            node.second->accelVals(4) };
        savedData.push_back(robotInst);
    }
}

void Nodes::saveData(int t) {
    std::ofstream file("src/estimator/data/saved_data_for_test_1.bin", std::ios::binary);
    if (file.is_open()) {
        for (const auto& row: savedData) {
            size_t row_size = row.size();
            file.write(reinterpret_cast<const char*>(&row_size), sizeof(row_size));
            file.write(reinterpret_cast<const char*>(row.data()), row_size * sizeof(double));
        }
        file.close();
        cout<< "All data has been saved" << endl;
    } else {
        std::cerr << "Error while opening file" << endl;
    }
}


Nodes::~Nodes() {
    for (const auto& node : nodes) {
        delete node.second;
    }
    cout << "Group of nodes deleted" << endl;
}
