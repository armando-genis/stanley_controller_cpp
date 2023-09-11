#include "Linear_Interpolation.h"

Linear_Interpolation::Linear_Interpolation(const std::vector<double>& x_vals, const std::vector<double>& y_vals, double interp_distance_res) : INTERP_DISTANCE_RES(interp_distance_res)
{
    std::cout << "Linear_Interpolation Constructor called" << std::endl;
    if (x_vals.size() != y_vals.size()) {
        throw std::runtime_error("Size mismatch between x and y vectors");
    }

    for (size_t i = 0; i < x_vals.size(); i++) {
        Eigen::VectorXd waypoint(2);
        waypoint[0] = x_vals[i];
        waypoint[1] = y_vals[i];
        waypoints.push_back(waypoint);
    }
}

Linear_Interpolation::~Linear_Interpolation()
{
    std::cout << "Linear_Interpolation Destructor called" << std::endl;
}   

void Linear_Interpolation::interpolateWaypoints(){
    for (size_t i = 1; i < waypoints.size(); i++) {
        Eigen::VectorXd diff = waypoints[i] - waypoints[i - 1];
        wp_distance.push_back(diff.norm());
    }
    wp_distance.push_back(0.0);

    int interp_counter = 0;

    for (size_t i = 0; i < waypoints.size() - 1; i++) {
        wp_interp.push_back(waypoints[i]);
        wp_interp_hash.push_back(interp_counter);
        interp_counter++;

        int num_pts_to_interp = static_cast<int>(std::floor(wp_distance[i] / INTERP_DISTANCE_RES)) - 1;
        Eigen::VectorXd wp_vector = waypoints[i + 1] - waypoints[i];
        Eigen::VectorXd wp_uvector = wp_vector.normalized();

        for (int j = 0; j < num_pts_to_interp; j++) {
            Eigen::VectorXd next_wp_vector = INTERP_DISTANCE_RES * static_cast<double>(j + 1) * wp_uvector;
            wp_interp.push_back(waypoints[i] + next_wp_vector);
            interp_counter++;
        }
    }
    wp_interp.push_back(waypoints.back());
    wp_interp_hash.push_back(interp_counter);
}

double Linear_Interpolation::getSize() const{
    return wp_interp.size();
}

std::vector<Eigen::VectorXd> Linear_Interpolation::getWaypoints() const{
    return waypoints;
}

std::vector<Eigen::VectorXd> Linear_Interpolation::getWp_interp() const{
    return wp_interp;
}

std::vector<int> Linear_Interpolation::getWp_interp_hash() const{
    return wp_interp_hash;
}

std::vector<double> Linear_Interpolation::getWp_distance() const{
    return wp_distance;
}