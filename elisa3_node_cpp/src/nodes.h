#ifndef NODES_H
#define NODES_H

#include <vector>
#include <Eigen/Dense>
#include <map>
#include <string>
#include <boost/variant.hpp>
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

class CameraMarker {

    private:

        ros::Subscriber listenerCameraList;
        //TODO check datatype of msmtList
        // Eigen::VectorXd measurementList;

    public:
    
        ros::NodeHandle n;
        int number;
        int currentNumber;
        std::vector<double> measurementList;

        CameraMarker(int N);
        //TODO - check datatype of optiMsg
        void listenOptitrackMarkersCallback(const std_msgs::Float64MultiArray::ConstPtr& optiMsg);
};

class CameraNode {

    private:

        int tag;
        ros::Subscriber listenerCamera;
        ros::Subscriber listenerCameraTimer;

    public:

        double camX;
        double camY;
        double camPhi;
        double timer;
        ros::NodeHandle n;

        CameraNode(int tag);
        //TODO - check datatype of optiMsg
        void listenOptitrackCallback(const geometry_msgs::Pose2D::ConstPtr& optiMsg);
        void listenOptitrackTimerCallback(const geometry_msgs::PoseStamped::ConstPtr& optiMsg);
};

class Cameras {

    private:

        int number;
        std::map<int, CameraNode*> cameras;
        
        //TODO - check if this publisher is necessary
        // ros::Publisher publisherCams;
        // std_msgs::Float64MultiArray msgCam;

    public:

        // ros::NodeHandle n;
        //size (N, 4) N - from Cameras()
        //measurement for one item is along the row
        Eigen::MatrixXd measurementList;
        // Eigen::MatrixXd measurementListPrev;
        std::vector<double> msgCam;

        Cameras(int N);
        void updateCamera();
        void publishCams();
};

class Node {

    private:

        int t;

        std::string tag;
        double releaseTime;
        std::string address;
        double startPos[2];
        double startOrien;

        // robot meas pose and robot meas orien values are found in odomVals
        // Eigen::Vector2d robot_meas_pose;
        // double robot_meas_orien;
        // double robotMeasTime;
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

        Eigen::Vector2d goalX;
        std::map<std::string, double> setup;
        //size is (2, bufferSize)
        Eigen::MatrixXd posBuf;
        //size is (1, bufferSize)
        Eigen::MatrixXd orienBuf;

        ros::Subscriber listenerRobotPose;
        ros::Subscriber listenerAccel;

    public:

        ros::NodeHandle n;
        bool updateLeds;
        int msgLeds[3];
        bool updateReset;
        double msgReset[4];
        bool updateAutoMove;
        double msgAutoMove[4];
        int triggerAutoMove;

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


        Node(double releaseTime, std::string tagExt);
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
        Eigen::Vector3d determineCameraMarker(CameraMarker& cameraMarker);
        Eigen::Vector3d determineCamera(Cameras& cameras);
        Eigen::Matrix3d measurementUpdate(Cameras& cameras, CameraMarker& cameraMarker);
        void measurementFusion(Eigen::Matrix3d measurements);
        void measurementFusionOWA(Eigen::Matrix3d measurements);
        void nodeLoopFun(Cameras& cameras, CameraMarker& cameraMarker, const std::string moveType);

};

class Nodes {
    
    private:

        std::vector<std::string> activeRobots;
        int N;

        //TODO - check if we need publishers
        // ros::Publisher publisherAutoMove;
        // ros::Publisher publisherLeds;
        // ros::Publisher publisherReset;
        // ros::Publisher publisherInput;
        // ros::Publisher publisherInputs;
        //TODO - check if we need these
        // geometry_msgs::Twist msgInput;
        // std_msgs::Float64MultiArray msgInputs;
        // std_msgs::Float64MultiArray msgInputs;
        // std_msgs::Float64MultiArray msgLeds;
        // std_msgs::Float64MultiArray msgReset;
        
        //TODO - figure out datatype of stored data
        using multiType = boost::variant<double, Eigen::Matrix3d, std::vector<double>>;
        std::map<int, std::map<std::string, std::map<std::string, multiType>>> savedData;
        

    public:

        std::map<std::string, Node*> nodes;
        Cameras cameras;
        CameraMarker cameraMarker;
        std::vector<int> msgLeds;
        std::vector<double> msgReset;
        std::vector<double> msgAutoMove;

        Nodes(std::vector<std::string> activeRobotsExt);
        void nodesLoopFn(const std::string moveType = "move");
        void testCam();
        void printFn();
        void move(const std::string moveType = "move", double stepSize = 0.0, double theta = 0.0);
        //TODO - changed argument, check later
        void updateLeds();
        //TODO - changed argument, check later
        void nodesReset(const std::string type = "odom");
        void turnOffLeds();
        void setLeds(const int ledIntensity[3]);
        void nodesPrintPositionMeasures();
        void storeData(int t);
        void saveData(int t);

};

#endif