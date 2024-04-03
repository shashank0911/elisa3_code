#ifndef KALMAN_H
#define KALMAN_H

#include <Eigen/Dense>
#include <tuple>

class Kalman {
    public:

        Eigen::Matrix3d Qk;
        Eigen::Matrix3d Rk;
        Eigen::Matrix3d Pk1;

        Kalman();

        std::pair<Eigen::Vector3d, Eigen::Matrix3d> srEKF(const Eigen::Vector3d& Zk, const Eigen::Vector3d& stateEstimateK, double dk = 1);
        std::pair<Eigen::Vector3d, Eigen::Matrix3d> mrEKF(const Eigen::Vector3d& Zkm, const Eigen::Vector3d& stateEstimateKm, double dk = 1);

    private:
    
        Eigen::Matrix3d Ak1;
        Eigen::Vector3d processNoiseVkMinus1;
        
        Eigen::Matrix3d Hk;
        
        Eigen::Vector3d sensorNoiseWk;
        

};

#endif