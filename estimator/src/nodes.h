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

        // Robot tag (OptiTrack ID - 1)
        int tag;

    public:

        // Store camera measurements for a single camera
        double camX;
        double camY;
        double camPhi;
        double timer;

        // Constructor
        CameraNode(int tag);

        // Destructor
        ~CameraNode();

        // ROS callback functions to obtain camera measurements
        void listenOptitrackCallback(const geometry_msgs::Pose2D::ConstPtr& optiMsg);
        void listenOptitrackTimerCallback(const geometry_msgs::PoseStamped::ConstPtr& optiMsg);
};

class Cameras {

    private:

        // Number of robots
        int number;

        // Array to store camera measurement of all robots for publishing
        std_msgs::Float64MultiArray msgCam;

    public:

        // Stores a map of robot ID (optitrack ID) and corresponding CameraNode class
        std::map<int, CameraNode*> cameras;

        // Stores the camera information for all robots
        // Size (N, 4), measurement for one camera is along the row
        Eigen::MatrixXd measurementList;

        // Constructor
        Cameras(int N);

        // Destructor
        ~Cameras();

        // Function to update measurementList
        void updateCamera();

        // Function to update msgCam
        void publishCams();
};

class Node {

    private:

        // Discrete time or loop instance
        int t;

        // Robot tag (OptiTrack ID)
        std::string tag;
        double releaseTime;
        
        // Initial condition for a robot
        double startPos[2];
        double startOrien;

        // Control inputs (linear and angular velocity)
        double inputV;
        double inputOmega;

        // Robot pose (x, y, phi in order)
        Eigen::Vector3d xyphiVals;

        // Camera measurement at the previous instance
        Eigen::Vector3d camPrev;

        // Robot pose estimate at the previous instance
        Eigen::Vector3d prevEst;

        // Buffer size        
        int bufferSize;

        // Error buffer for odometry, accelerometer and camera measurement
        // Size is bufferSize
        Eigen::VectorXd odomErrorBuffer;
        Eigen::VectorXd camErrorBuffer;
        Eigen::VectorXd accelErrorBuffer;
        
        // Buffer containign most recent position estimates
        // Size is (2, bufferSize)
        Eigen::MatrixXd posBuf;

        // Buffer containing most recent oreintation estimates
        // Size is (1, bufferSize)
        Eigen::MatrixXd orienBuf;

    public:

        // Elisa3 address
        std::string address;

        // Goal or destination coordinates
        Eigen::Vector2d goal;

        // Message to update LEDs
        bool updateLeds;
        int msgLeds[3];

        // Message to reset sensors
        bool updateReset;
        double msgReset[4];

        // Message to update motors
        double msgAutoMove[4];

        // Odometry measurements (x, y, phi in order)
        Eigen::Vector3d odomVals;
        double odomTimer;

        // Camera measurements (x, y, phi in order)
        Eigen::Vector3d camVals;
        double camTimer;

        // Accelerometer measurements (accel_x, accel_y, x_lowpass, pos_x, pos_y in order)
        Eigen::VectorXd accelVals;

        // Current fused estimate (x, y, phi in order)
        Eigen::Vector3d curEst;

        // 
        Eigen::Vector3d theoPosn;

        // Kalman class objects for odometry estimate
        Kalman kalmanOdom;

        // Kalman class objects for camera estimate
        Kalman kalmanCam;

        // Kalman class objects for accelerometer estimate
        Kalman kalmanAccel;

        // OWA weights (odometry, camera, accelerometer in order)
        Eigen::Vector3d OwaWeights;

        // Instance of class
        ObstacleAvoidance obstacleAvoidanceNode;

        // Constructor
        Node(double releaseTime, std::string tagExt);
        
        // Destructor
        ~Node();

        // Function to set intensity of green LED
        void greenLed(int intensity);

        // Function to set intensity of red LED
        void redLed(int intensity);

        // Function to set intensity of Blue LED
        void blueLed(int intensity);

        // Function to set intensity of all LEDs to 0
        void ledOff();

        //ROS callback function for odometry measurement
        void listenRobotPoseCallback(const nav_msgs::Odometry::ConstPtr& odomMsg);

        // ROS callback function for accelerometer measurement
        void listenAccelCallback(const sensor_msgs::Imu::ConstPtr& accelMsg);

        // Print information about the robot
        void nodePrintPose();

        // Function to find the next desired position and control inputs
        void computeMove(double pol[2]);

        // Function to reset the sensors
        void nodeReset(const std::string type = "odom");

        // Function to update current estimate based on state equations
        void statesTransform();

        // Function to compute control inputs to reach destination point
        std::pair<double, double> goToGoal();

        // Function to help stop robot if pose error is within a certain limit
        bool terminate();

        // Function to determine closest camera measurement to the robot (in case of camera misalignment errors)
        Eigen::Vector3d determineCamera(Cameras& cameras);

        // Update all sensor measurements
        // Returns measurements in order of odometry, accelerometer and camera 
        Eigen::Matrix3d measurementUpdate(Cameras& cameras);

        // Function to find the Kalman estimate
        void measurementFusionOWA(Eigen::Matrix3d measurements);

        // Function containing order of functions performed in one loop
        void nodeLoopFun(Cameras& cameras, const std::string moveType);

};

class Nodes {
    
    private:

        // List of robot addresses (Elisa3)
        std::vector<std::string> activeRobots;

        // Number of robots
        int N;

        // Saving data for plotting
        std::vector<std::vector<double>> savedData;
        

    public:

        // Map of robot ID (optitrack ID) and a node class (for a single robot)
        std::map<std::string, Node*> nodes;

        // Instance of Cameras containing camera data of all robots
        Cameras cameras;

        // ROS message to update motors for all robots 
        std_msgs::Float64MultiArray msgAutoMove;

        // ROS message to update LEDs for all robots 
        std_msgs::Float64MultiArray msgLeds;

        // ROS message to update sensor reset for all robots 
        std_msgs::Float64MultiArray msgReset;

        // Constructor
        Nodes(std::vector<std::string> activeRobotsExt);

        // Destructor
        ~Nodes();

        // Function containing order of functions to execute in one loop
        void nodesLoopFn(const std::string moveType = "move");
        // void testCam();

        // Function to print information of all the robots
        void printFn();

        void move(const std::string moveType = "move", double stepSize = 0.0, double theta = 0.0);
        
        // Function to update msgLeds
        void updateLeds();

        // Function to update msgReset
        void nodesReset(const std::string type = "odom");
        
        // Function to set intensity of all robots' LEDs to 0
        void turnOffLeds();

        // Function to set the intensity of all robots' LEDs to an input
        void setLeds(const int ledIntensity[3]);

        // Function to update savedData
        void storeData(int t);

        // Function to store savedData in a file
        void saveData(int t);

};

#endif