#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Dense>
#include <tuple>
#include <map>

const double stepThreshold = 1e-6;
// Eigen::MatrixXd getDomain();
const bool obstacles = false;

Eigen::Vector2d cart2pol(const Eigen::Vector2d& cart);
Eigen::Vector2d pol2cart (const Eigen::Vector2d& pol);
Eigen::MatrixXd renewVec(const Eigen::MatrixXd& oldVec);
double yawFromQuaternion(double x, double y, double z, double w);

class ObstacleAvoidance {
public:
    ObstacleAvoidance();
    
    static Eigen::Vector2d perpendicular(const Eigen::Vector2d& a);
    static double det(const Eigen::Matrix2d& mat);
    static bool checkDirectionVectors(const Eigen::Vector2d& vector1, const Eigen::Vector2d& vector2);
    bool checkInDomain(const Eigen::Vector2d& point);
    std::pair<int, Eigen::Vector2d> lineIntersection(const Eigen::Matrix2d& locations);
    std::pair<Eigen::Vector2d, bool> obstacleAvoidance(const Eigen::Vector2d& startPoint, const Eigen::Vector2d& move);

private:
    Eigen::MatrixXd refLinesDomain;
    // Eigen::VectorXd refLines;
};

#endif