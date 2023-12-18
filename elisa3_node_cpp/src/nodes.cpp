#include "nodes.h" 
#include <iostream>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <jsoncpp/json/value.h>
#include <unistd.h>
#include <cmath>

using std::string;
using std::cout;
using std::endl;

const double WHEEL_DISTANCE = 0.041;
const double T_DELAY = 0.5;
const int STD_VLT_TRANS = 20;
const int STD_VLT_ROT = 10;
const double SAMPLING_TIME = 0.005;
const double PI = 3.14159265358979323846;

const string MODE = "kalman";

const int SR_KALMAN = 0;
const int MR_KALMAN = 1;

ObstacleAvoidance obstacleAvoidance;



//TODO - check if argument activeRobots is really necessary
CameraMarker::CameraMarker(int N) {
    number = N;
    currentNumber = N;
    listenerCameraList = n.subscribe("Bebop1/makers", 10, &CameraMarker::listenOptitrackMarkersCallback, this);
}

void CameraMarker::listenOptitrackMarkersCallback(const std_msgs::Float64MultiArray::ConstPtr& optiMsg) {
    if (optiMsg->data.empty()) {
        measurementList.resize(0);
        return;
    }
    int nRobots = int(optiMsg->data[0]);
    measurementList.resize(nRobots);
    for (int i = 0; i < nRobots; i++) {
        //TODO - check if it should be double or int
        measurementList[i] = double(optiMsg->data[i]);
    }
}

CameraMarker::~CameraMarker() {}




CameraNode::CameraNode(int tag_ext) {
    camX = 0.0;
    camY = 0.0;
    camPhi = 0.0;
    tag = tag_ext;
    timer = 0.0;
    listenerCamera = n.subscribe("Bebop" + std::to_string(tag + 1) + "/ground_pose", 10, &CameraNode::listenOptitrackCallback, this);
    listenerCameraTimer = n.subscribe("Bebop" + std::to_string(tag + 1) + "/pose", 10, &CameraNode::listenOptitrackTimerCallback, this);
}

void CameraNode::listenOptitrackCallback(const geometry_msgs::Pose2D::ConstPtr& optiMsg) {
    camX = optiMsg->x;
    camY = optiMsg->y;
    camPhi = optiMsg->theta;
}

void CameraNode::listenOptitrackTimerCallback(const geometry_msgs::PoseStamped::ConstPtr& optiMsg) {
    timer = optiMsg->header.stamp.toSec();
}

CameraNode::~CameraNode() {}



Cameras::Cameras(int N) {
    number = N;
    for (int tag = 0; tag < N; tag++) {
        cameras[tag] = new CameraNode(tag);
    }
    measurementList = Eigen::MatrixXd(N, 4);
    //TODO - check if below variable is necessary
    // measurementListPrev = Eigen::MatrixXd(N, 4);

    //TODO - work on the publish part later if necessary. involves msgcam, publishercams
    std::vector<double> msgCam(number*3 + 1);
}

void Cameras::updateCamera() {
    int i = 0;
    for (const auto& camera: cameras) {
        //TODO - what does this do??
        measurementList(i,0) = 2.0 + camera.second->camY;
        measurementList(i,1) = 1.0 + camera.second->camX;
        measurementList(i,2) = camera.second->camPhi;
        measurementList(i,3) = camera.second->timer;
        i++;
    }
}

//TODO
void Cameras::publishCams() {
    int count = 0;
    for (int i = 0; i < number; i++) {
        msgCam[i*3 + 1] = measurementList(i, 0);
        msgCam[i*3 + 2] = measurementList(i, 1);
        msgCam[i*3 + 3] = measurementList(i, 2);
        count++;
    }
    msgCam[0] = number;
}

Cameras::~Cameras() {}





Node::Node(double releaseTime, std::string tagExt) {
    std::ifstream file("mapper.json");
    Json::Value mapper;
    Json::Reader reader;
    reader.parse(file, mapper);
    
    t = 0;

    tag = tagExt;
    this->releaseTime = releaseTime;
    // Json::Value addrArr = mapper[tag]["address"];
    Json::Value posArr = mapper[tag]["pos"];
    // Json::Value orienArr = mapper[tag]["orien"];

    address = mapper[tag]["address"].asString();
    startPos[0] = posArr[0].asDouble();
    startPos[1] = posArr[1].asDouble();
    startOrien = mapper[tag]["orien"].asDouble();

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

    kalmanOdom = Kalman();
    kalmanCam = Kalman();
    kalmanAccel = Kalman();

    threshold = 0.02;

    bufferSize = 5;

    odomErrorBuffer.resize(bufferSize);
    odomErrorBuffer.fill(0.1);
    camErrorBuffer.resize(bufferSize);
    camErrorBuffer.fill(0.1);
    accelErrorBuffer.resize(bufferSize);
    accelErrorBuffer.fill(0.1);

    OwaWeights.fill(0.0);

    goalX << 0.0, 0.5;
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

    listenerRobotPose = n.subscribe("elisa3_robot_" + tag + "/odom", 10, &Node::listenRobotPoseCallback, this);
    listenerAccel = n.subscribe("swarm/elisa3_robot_" + tag + "/accel", 10, &Node::listenAccelCallback, this);

    updateLeds = false;
    for (int i = 0; i < 3; i++) {
        msgLeds[i] = 0;
    }
    updateReset = false;
    updateAutoMove = false;
    for (int i = 0; i < 4; i++) {
        msgReset[i] = 0.0;
        msgAutoMove[i] = 0.0;
    }
    triggerAutoMove = 103;

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
    //TODO - check if we need to use robot meas pose and robot meas orien
    odomVals << odomMsg->pose.pose.position.x, 
                odomMsg->pose.pose.position.y,
                odomMsg->pose.pose.position.z;
    odomTimer = odomMsg->header.stamp.toSec();

}

void Node::listenAccelCallback(const sensor_msgs::Imu::ConstPtr& accelMsg) {
    accelVals(0) = accelMsg->linear_acceleration.x;
    accelVals(1) = accelMsg->linear_acceleration.y;
    accelVals(2) = accelMsg->angular_velocity.x;
}

void Node::nodePrintPositionMeasures() {
    std::stringstream msg;

    msg << "\nID: " << tag << "\n"
        << "Estimation - Position: (" << curEst(0) << ", " << curEst(1) << "), Orientation: " << curEst(2) << "\n"
        << "Odometry - Position: (" << odomVals(0) << ", " << odomVals(1) << "), Orientation: " << odomVals(2) << "\n"
        << "Camera - Position: (" << camVals(0) << ", " << camVals(1) << "), Orientation: " << camVals(2) << "\n"
        << "Accelerometer - Position: (" << accelVals(3) << ", " << accelVals(4) << "), Velocity: " << accelVals(0) << "\n";

    cout << msg.str() << endl;
}

void Node::computeMove(double pol[2]) {
    nodePrintPositionMeasures();

    double orienCor = orienBuf(1, bufferSize - 1);
    if (orienCor < 0.0) {
        orienCor += 2*PI;
    }
    orienCor = fmod(orienCor, 2*PI);
    if (orienCor > PI) {
        orienCor -= 2*PI;
    }

    double phiCor = pol[1];
    if (phiCor < 0.0) {
        phiCor += 2*PI;
    }
    phiCor = fmod(phiCor, 2*PI);
    if (phiCor > PI) {
        phiCor -= 2*PI;
    }

    double deltaPhi = phiCor - orienCor;
    if (deltaPhi >= 0.0) {
        msgAutoMove[0] = 1.0;
        msgAutoMove[1] = deltaPhi;
        msgAutoMove[2] = 0.0;
        msgAutoMove[3] = pol[0];
    } else {
        msgAutoMove[0] = 0.0;
        msgAutoMove[1] = -deltaPhi;
        msgAutoMove[2] = 0.0;
        msgAutoMove[3] = pol[0];
    }

    updateAutoMove = true;
    if (triggerAutoMove == 103) {
        triggerAutoMove = 104;
    } else {
        triggerAutoMove = 103;
    }

    Eigen::Vector2d polVec;
    polVec << pol[0], phiCor;
    Eigen::Vector2d propMoveCart = pol2cart(polVec);
    auto [calcPoint, obsAvoidMode] = obstacleAvoidance.obstacleAvoidance(posBuf.col(bufferSize - 1), propMoveCart);

    posBuf = renewVec(posBuf);
    orienBuf = renewVec(orienBuf);

    posBuf.col(bufferSize - 1) << calcPoint;
    orienBuf(1, bufferSize - 1) += deltaPhi; 
}

void Node::nodeReset(const std::string type) {
    if (type == "odom") {
        if (odomVals.hasNaN()) {
            cout << "Robot " << tag << "'s odometry measurement outputs NaN" << endl;
        } else {
            posBuf.col(bufferSize - 1) << odomVals(0), odomVals(1);
            orienBuf(1, bufferSize - 1) == odomVals(2);
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
    curEst(0) += inputV * std::cos(curEst(2)) / 1000.0 * SAMPLING_TIME;
    curEst(1) += inputV * std::sin(curEst(2)) / 1000.0 * SAMPLING_TIME;
    curEst(2) += inputOmega;
}

std::pair<double, double> Node::goToGoal() {
    Eigen::Vector2d err;
    err << goalX - curEst.head(2);
    Eigen::Vector2d Kp(-50, -50);
    double v = (Kp.cwiseProduct(err)).norm();
    double phi = std::atan2(err(1), err(0));
    double omega = setup["Kp"] * std::atan2( std::sin(phi - curEst(2)), std::cos(phi - curEst(2)) );
    return std::make_pair(v, omega);
}

bool Node::terminate() {
    double maxLength = 0.001;
    double distance = (goalX - curEst.head(2)).norm();
    if (distance < maxLength) {
        return 0;
    } else {
        return 1;
    }
}

Eigen::Vector3d Node::determineCameraMarker(CameraMarker& cameraMarker) {
    double minDist = 1e5;
    int i = 0;
    Eigen::Vector2d camMarkerMeas;
    cout << "Camera marker number: " << cameraMarker.currentNumber << endl;
    for (int j = 0; j < cameraMarker.currentNumber; j++) {
        camMarkerMeas << cameraMarker.measurementList[i*3 + 2], cameraMarker.measurementList[i*3 + 4];
        double dist = (curEst.head(2) - camMarkerMeas).norm();
        if (dist < minDist) {
            minDist = dist;
            i = j;
        }
    }
    camVals << camMarkerMeas(0), camMarkerMeas(1), odomVals(2);

    if (cameraMarker.number != cameraMarker.currentNumber) {
        if (minDist > 0.06) {
            return Eigen::Vector3d(curEst(0), curEst(1), odomVals(2));
        } else {
            return Eigen::Vector3d(camVals(0), camVals(1), odomVals(2));
        }
    } else {
        return Eigen::Vector3d(camVals(0), camVals(1), odomVals(2));
    }
}

Eigen::Vector3d Node::determineCamera(Cameras& cameras) {
    double minDist = 1e5;
    cameras.updateCamera();

    int idx = std::stoi(tag);
    camVals = cameras.measurementList.row(idx).head(3);
    camTimer = cameras.measurementList(idx, 3);

    double dist = (curEst.head(2) - camVals.head(2)).norm();

    if (dist < 0.05) {
        return Eigen::Vector3d(camVals(0), camVals(1), odomVals(2));
    }

    cout << "Current estimation: (" << curEst(0) << ", " << curEst(1) << ")\n";
    int i = 0;
    idx = 0;

    for (int j = 0; j < cameras.measurementList.rows(); j++) {
        if (i == std::stoi(tag)) {
            i += 1;
            continue;
        }
        dist = (curEst.head(2) - cameras.measurementList.row(j).head(2)).norm();
        //TODO - figure out what this does
        cout << "Camera measurement: (" << cameras.measurementList(j,0) << ", " << cameras.measurementList(j,1) << ")\n";
        cout << "Distance from camera: " << dist << endl;

        if (dist < minDist) {
            minDist = dist;
            idx = i;
        }
        i += 1;
    }
    camVals << cameras.measurementList.row(idx).head(3);
    camTimer = cameras.measurementList(idx, 3);

    if (minDist > threshold) {
        //TODO - what is this?
        threshold = 0.05;
        return odomVals;
    } else {
        threshold = 0.05;
        return Eigen::Vector3d(camVals(0), camVals(1), odomVals(2));
    }
}

//returns odom, accel and cam values, correspondingly in each row
Eigen::Matrix3d Node::measurementUpdate(Cameras& cameras, CameraMarker& cameraMarker) {
    //TODO - doubtful
    accelVals(3) += accelVals(0) * std::cos(curEst(2)) * SAMPLING_TIME * SAMPLING_TIME;
    accelVals(4) += accelVals(0) * std::sin(curEst(2)) * SAMPLING_TIME * SAMPLING_TIME;
    Eigen::Vector3d accelMsmt;
    accelMsmt << accelVals(3), accelVals(4), curEst(2);

    Eigen::Vector3d camMsmt = determineCameraMarker(cameraMarker);

    Eigen::Matrix3d returnVal;
    returnVal << odomVals, accelMsmt, camMsmt;

    return returnVal; 
}

//TODO - not used, so type it later
// void Node::measurementFusion(Eigen::Matrix3d measurements) {
//     if (MODE == "CAM") {
//         curEst = cam
//     }
// }

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

    //TODO - check for datatype of t
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
        //TODO - why??
        // tempAccel = 0.0;
        tempOdom = sum1/sumOdom;
        double tempCam = sum1/sumCam;
        tempSum = tempOdom + tempAccel + tempCam + offset;
        OwaWeights << tempOdom/tempSum, (tempCam + offset)/tempSum, tempAccel/tempSum;
        curEst = OwaWeights(0) * odomEst + OwaWeights(1) * camEst + OwaWeights(2) * accelEst;
    }

}

void Node::nodeLoopFun(Cameras& cameras, CameraMarker& cameraMarker, const std::string moveType) {
    t += 1;

    //TODO - profiling
    //TODO - print time 
    cout << "Loop function starting for " << t << "th loop" << endl;

    Eigen::Matrix3d measurements = measurementUpdate(cameras, cameraMarker);
    //TODO - print time
    cout << "Measurement update done" << endl;

    statesTransform();
    measurementFusionOWA(measurements);
    //TODO - print time
    cout << "Measurement fusion done" << endl;

    double distX = curEst(0) - prevEst(0);
    double distY = curEst(1) - prevEst(1);
    curEst(2) = std::atan2(distY, distX);
    prevEst << curEst;

    //TODO - check
    inputV = 1.2;
    inputOmega = 0.0;
    double pol[2] = {inputV, inputOmega};
    computeMove(pol);

}

Node::~Node() {}








Nodes::Nodes(std::vector<std::string> activeRobots) : cameras(activeRobots.size()), cameraMarker(activeRobots.size()) {

    std::ifstream file("mapper.json");
    Json::Value mapper;
    Json::Reader reader;
    reader.parse(file, mapper);
    // Json::Value addressList = mapper["address"];
    std::vector<std::string> keys = mapper.getMemberNames();

    this->activeRobots = activeRobots;
    N = activeRobots.size();
    double releaseTime = 0.0;
    //TODO - check if we need publisher modules
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

    std::vector<int> msgLeds(N*4 + 1);
    std::vector<double> msgReset(N*5 + 1);
    std::vector<double> msgAutoMove(N*5 + 1);
    //TODO - figure out datatype/format of saved data
    // savedData = 
}

void Nodes::nodesLoopFn(const std::string moveType) {
    int i = 0;
    int count = 0;
    for (const auto& node: nodes) {
        nodes[node.first]->nodeLoopFun(cameras, cameraMarker, moveType);
        msgAutoMove[i*5 + 1] = std::stod(node.first);
        msgAutoMove[i*5 + 2] = node.second->msgAutoMove[0];
        msgAutoMove[i*5 + 3] = node.second->msgAutoMove[1];
        msgAutoMove[i*5 + 4] = node.second->msgAutoMove[2];
        msgAutoMove[i*5 + 5] = node.second->msgAutoMove[3];
        i++;
        count++;
    }
    msgReset[0] = count;
    //TODO - publisher; send msgAutoMove to elisa3 fn

    usleep(int(SAMPLING_TIME*1000000.0));
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

        msgAutoMove[i*5 + 1] = std::stod(tag.first);
        msgAutoMove[i*5 + 2] = tag.second->msgAutoMove[0];
        msgAutoMove[i*5 + 3] = tag.second->msgAutoMove[1];
        msgAutoMove[i*5 + 4] = tag.second->msgAutoMove[2];
        msgAutoMove[i*5 + 5] = tag.second->msgAutoMove[3];
        i++;
        count++;
    }
    msgReset[0] = count;
    //TODO - publisher for msg automotive

    usleep(int(SAMPLING_TIME*1000000.0));
}

void Nodes::updateLeds() {
    int count = 0;
    // auto bigIter = msgLeds.begin();
    // std::advance(bigIter, 1);
    int i = 0;
    for (const auto& tag : nodes) {
        // auto smallIter = nodes[tag.first]->msgLeds;
        msgLeds[i*4 + 1] = std::stoi(tag.first);
        msgLeds[i*4 + 2] = nodes[tag.first]->msgLeds[0];
        msgLeds[i*4 + 3] = nodes[tag.first]->msgLeds[1];
        msgLeds[i*4 + 4] = nodes[tag.first]->msgLeds[2];
        i++;
        count++;
    }
    msgLeds[0] = count;
    //TODO - publisher; call the elisa node fn to update robot dict
}

void Nodes::nodesReset(const std::string type) {
    int count = 0;
    int i = 0;
    for (const auto& tag : nodes) {
        nodes[tag.first]->nodeReset(type);
        if (nodes[tag.first]->updateReset) {
            msgReset[i*5 + 1] = std::stod(tag.first);
            msgReset[i*5 + 2] = nodes[tag.first]->msgReset[0];
            msgReset[i*5 + 3] = nodes[tag.first]->msgReset[1];
            msgReset[i*5 + 4] = nodes[tag.first]->msgReset[2];
            msgReset[i*5 + 5] = nodes[tag.first]->msgReset[3];
            i++;
            count++;
        }
    }

    if (count != 0) {
        //TODO - update printed message
        cout << "Reset - theoretical" << endl;
        msgReset[0] = count;
        //TODO - publisher; call the elisa node fn to update robot dict
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
    //TODO
    savedData[t] = std::map<std::string, std::map<std::string, multiType>>();
    for (const auto& node : nodes) {
        savedData[t][node.first] = {
            {"pos_x", node.second->odomVals(0)},
            {"pos_y", node.second->odomVals(1)},
            {"orien", node.second->odomVals(2)},
            {"estimation_x", node.second->curEst(0)},
            {"estimation_y", node.second->curEst(1)},
            {"estimation_phi", node.second->curEst(2)},
            {"cam_x", node.second->camVals(0)},
            {"cam_y", node.second->camVals(1)},
            {"cam_phi", node.second->camVals(2)},
            {"x", node.second->theoPosn(0)},
            {"y", node.second->theoPosn(1)},
            {"phi", node.second->theoPosn(2)},
            {"P_k_odo", node.second->kalmanOdom.Pk1},
            {"P_k_cam", node.second->kalmanCam.Pk1},
            {"odom_timer", node.second->odomTimer},
            {"cam_timer", node.second->camTimer},
            {"OWA_w1", node.second->OwaWeights(0)},
            {"OWA_w2", node.second->OwaWeights(1)},
            {"OWA_w3", node.second->OwaWeights(2)},
            {"accel_x", node.second->accelVals(0)},
            {"accel_x_lowpass", node.second->accelVals(2)},
            {"accel_x_pos", node.second->accelVals(3)},
            {"accel_y_pos", node.second->accelVals(4)},
            {"full_camera", cameraMarker.measurementList}
        };
    }
}

void Nodes::saveData(int t) {
    //TODO
    std::ofstream file("./data/saved_data_for_test_1.bin", std::ios::binary);
    if (file.is_open()) {
        for (const auto& dataEntry : savedData) {
            file.write(reinterpret_cast<const char*>(&dataEntry.second), sizeof(dataEntry.second));
        }
        file.close();
    } else {
        std::cerr << "Error while opening file" << endl;
    }

}


Nodes::~Nodes() {}
