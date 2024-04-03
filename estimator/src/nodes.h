#ifndef NODES_H
#define NODES_H

#include <vector>
#include <Eigen/Dense>
#include <map>
#include <string>
#include "utils.h"
#include "Kalman.h"
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

class CameraNode {

    private:

        int tag;

    public:

        double camX;
        double camY;
        double camPhi;
        double timer;

        CameraNode(int tag);
        ~CameraNode();
        void listenOptitrackCallback(const geometry_msgs::Pose2D::ConstPtr& optiMsg);
        void listenOptitrackTimerCallback(const geometry_msgs::PoseStamped::ConstPtr& optiMsg);
};

class Cameras {

    private:

        int number;
        std_msgs::Float64MultiArray msgCam;

    public:

        std::map<int, CameraNode*> cameras;
        //size (N, 4) N - from Cameras(), measurement for one camera is along the row
        Eigen::MatrixXd measurementList;

        Cameras(int N);
        ~Cameras();
        void updateCamera();
        void publishCams();
};

class Node {

    private:

        int t;

        std::string tag;
        double releaseTime;
        
        double startPos[2];
        double startOrien;

        Eigen::Vector3d robotMeasVals;
        double robotMeasTime;
        double robotInput;

        double inputV;
        double inputOmega;

        // x, y, phi values
        Eigen::Vector3d xyphiVals;

        Eigen::Vector3d camPrev;
        double minDistPrev;

        Eigen::Vector3d prevEst;

        double threshold;
        
        int bufferSize;

        //sizes for all three are bufferSize
        Eigen::VectorXd odomErrorBuffer;
        Eigen::VectorXd camErrorBuffer;
        Eigen::VectorXd accelErrorBuffer;

        std::map<std::string, double> setup;
        //size is (2, bufferSize)
        Eigen::MatrixXd posBuf;
        //size is (1, bufferSize)
        Eigen::MatrixXd orienBuf;

    public:

        std::string address;
        Eigen::Vector2d goal;
        bool updateLeds;
        int msgLeds[3];
        bool updateReset;
        double msgReset[4];
        double msgAutoMove[4];

        // odom x, y, phi, timer values
        Eigen::Vector3d odomVals;
        double odomTimer;

        // cam x, y, phi, timer values
        Eigen::Vector3d camVals;
        double camTimer;

        // accel x, y, x_lowpass, xpos, ypos values
        Eigen::VectorXd accelVals;

        Eigen::Vector3d curEst;
        Eigen::Vector3d theoPosn;

        Kalman kalmanOdom;
        Kalman kalmanCam;
        Kalman kalmanAccel;

        // owa_w1, owa_w2, owa_w3
        Eigen::Vector3d OwaWeights;

        ObstacleAvoidance obstacleAvoidanceNode;

        Node(double releaseTime, std::string tagExt);
        ~Node();
        void publishGreenLed(int intensity);
        void publishRedLed(int intensity);
        void publishBlueLed(int intensity);
        void ledOff();
        void listenRobotPoseCallback(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void listenAccelCallback(const sensor_msgs::Imu::ConstPtr& accelMsg);
        void nodePrintPositionMeasures();
        void computeMove(double pol[2]);
        void nodeReset(const std::string type = "odom");
        void statesTransform();
        std::pair<double, double> goToGoal();
        bool terminate();
        Eigen::Vector3d determineCamera(Cameras& cameras);
        Eigen::Matrix3d measurementUpdate(Cameras& cameras);
        void measurementFusion(Eigen::Matrix3d measurements);
        void measurementFusionOWA(Eigen::Matrix3d measurements);
        void nodeLoopFun(Cameras& cameras, const std::string moveType);

};

class Nodes {
    
    private:

        std::vector<std::string> activeRobots;
        int N;
        std::vector<std::vector<double>> savedData;
        

    public:

        std::map<std::string, Node*> nodes;
        Cameras cameras;
        std_msgs::Float64MultiArray msgAutoMove;
        std_msgs::Float64MultiArray msgLeds;
        std_msgs::Float64MultiArray msgReset;

        Nodes(std::vector<std::string> activeRobotsExt);
        ~Nodes();
        void nodesLoopFn(const std::string moveType = "move");
        void testCam();
        void printFn();
        void move(const std::string moveType = "move", double stepSize = 0.0, double theta = 0.0);
        void updateLeds();
        void nodesReset(const std::string type = "odom");
        void turnOffLeds();
        void setLeds(const int ledIntensity[3]);
        void nodesPrintPositionMeasures();
        void storeData(int t);
        void saveData(int t);

};

#endif