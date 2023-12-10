#ifndef UTILS_H
#define UTILS_H

#include <Eigen/Dense>
#include <tuple>
#include <map>

const double step_threshold = 1e-6;
// Eigen::MatrixXd get_domain();
const bool obstacles = false;

Eigen::Vector2d cart2pol(const Eigen::Vector2d& cart);
Eigen::Vector2d pol2cart (const Eigen::Vector2d& pol);
Eigen::VectorXd renew_vec(const Eigen::VectorXd& old_vec);
double yaw_from_quaternion(double x, double y, double z, double w);

class ObstacleAvoidance {
public:
    ObstacleAvoidance();
    
    static Eigen::Vector2d perpendicular(const Eigen::Vector2d& a);
    static double det(const Eigen::Matrix2d& mat);
    static bool check_direction_vectors(const Eigen::Vector2d& vector_1, const Eigen::Vector2d& vector_2);
    bool check_in_domain(const Eigen::Vector2d& point);
    std::pair<int, Eigen::Vector2d> line_intersection(const Eigen::Matrix2d& locations);
    std::pair<Eigen::Vector2d, bool> obstacle_avoidance(const Eigen::Vector2d& start_point, const Eigen::Vector2d& move);

private:
    Eigen::MatrixXd ref_lines_domain;
    // Eigen::VectorXd ref_lines;
};

#endif