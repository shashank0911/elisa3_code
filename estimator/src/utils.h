#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Dense>
#include <tuple>
#include <map>
#include <vector>

const double stepThreshold = 1e-6;
const bool obstacles = false;

// Converts cartesian coordinates to polar coordinates
Eigen::Vector2d cart2pol(const Eigen::Vector2d& cart);

// Converts polar coordinates to cartesian coordinates
Eigen::Vector2d pol2cart (const Eigen::Vector2d& pol);

// Used to update the posBuf and orienBuf buffers by shifting the elements
Eigen::MatrixXd renewVec(const Eigen::MatrixXd& oldVec);

double yawFromQuaternion(double x, double y, double z, double w);

class ObstacleAvoidance {

public:

    ObstacleAvoidance();

    // Contains the boundary of the environment
    std::vector<Eigen::Matrix2d> refLinesDomain;
    
    // Returns a perpendicular vector
    static Eigen::Vector2d perpendicular(const Eigen::Vector2d& a);

    // Returns the determinant of a 2x2 matrix
    static double det(const Eigen::Matrix2d& mat);

    // Used in the lineIntersection function
    static bool checkDirectionVectors(const Eigen::Vector2d& vector1, const Eigen::Vector2d& vector2);
    
    // Checks if the theoretical next position of the robot is within the boundary
    bool checkInDomain(const Eigen::Vector2d& point);

    // Finds the boundary line (if it exists) which intersects the line joined by the current and next position of the robot
    std::pair<int, Eigen::Vector2d> lineIntersection(const Eigen::Matrix2d& locations);
    
    // Calculates the next position of the robot by ensuring it stays within bounds
    std::pair<Eigen::Vector2d, bool> obstacleAvoidance(const Eigen::Vector2d& startPoint, const Eigen::Vector2d& move);
};

#endif