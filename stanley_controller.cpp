#include <iostream>
#include <vector>
#include <Eigen/Dense>
// ----- TO DISPLAY THE WAYPOINTS ------
#include "matplotlibcpp.h"  // Include the matplotlib-cpp header
#include "BicycleModel.h"
#include <chrono>
#include <thread>
namespace plt = matplotlibcpp;  // Use a namespace alias for convenience

class StanleyController {
private:
    std::vector<Eigen::VectorXd> waypoints;
    std::vector<Eigen::VectorXd> wp_interp;
    std::vector<int> wp_interp_hash;
    std::vector<double> wp_distance;
    double INTERP_DISTANCE_RES = 0.01;

public:
    StanleyController(const std::vector<double>& x_vals, const std::vector<double>& y_vals) {
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

    void display() const;
    void plot_waypoints() const;
    void interpolateWaypoints(); 
    void plot(const std::vector<double>& x_vals, const std::vector<double>& y_vals, const std::vector<double>& thetas);

};

void StanleyController::interpolateWaypoints(){

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

void StanleyController::display() const {

    std::cout << "Total number of waypoints: " << waypoints.size() << std::endl;
    std::cout << "Total number of waypoints interpolated: " << wp_interp.size() << std::endl;

}

void StanleyController::plot_waypoints() const {
    std::vector<double> x_vals, y_vals;
    for (const auto& waypoint : waypoints) {
        x_vals.push_back(waypoint[0]);
        y_vals.push_back(waypoint[1]);
    }

    plt::plot(x_vals, y_vals, "bo-");  // Plot blue circles connected by lines
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::title("Waypoints");
    plt::grid(true);
    plt::show();
}


void StanleyController::plot(const std::vector<double>& x_vals, const std::vector<double>& y_vals, const std::vector<double>& thetas){
    plt::clf();
    std::vector<double> wp_x, wp_y;
    for (const auto& waypoint : waypoints) {
        wp_x.push_back(waypoint[0]);
        wp_y.push_back(waypoint[1]);
    }

    plt::plot(wp_x, wp_y, "ro");
    plt::plot(x_vals, y_vals, "b--");


    plt::xlabel("X");
    plt::ylabel("Y");
    plt::title("Waypoints and Vehicle Path with Orientation");
    plt::grid(true);
    plt::show();
    try {
        plt::pause(0.1);
    } catch (const std::runtime_error& e) {
        std::cerr << "Runtime error: " << e.what() << std::endl;
    }
}


int main() {
    plt::ion();  
    std::vector<double> x = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 6.0,6.0,6.0,6.0,6.0,6.0,6.0, 5.0,4.0,3.0,2.0,1.0,0.0,-1.0,-2.0,-3.0,-4.0,-5.0,-6.0,-7.0,-7.0,-7.0,-7.0,-7.0,-7.0,-7.0,-7.0, -6.0,-5.0,-4.0,-3.0,-2.0};
    std::vector<double> y = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,2.0,3.0,4.0,5.0,6.0,7.0, 7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0, 6.0,5.0,4.0,3.0,2.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0};

    StanleyController controller(x, y);
    controller.interpolateWaypoints();
    controller.display();
    // controller.plot();


    BicycleModel vehicle(0.0, 0.0, 0.0, 1.0);
    std::vector<double> vehicle_x, vehicle_y, vehicle_theta;

    double dt = 0.1;
    for (int i = 0; i < 500; i++) {

        vehicle.update(-3, 0.2, 0.1); // Steering angle, acceleration, time step

        // Store vehicle data for plotting
        vehicle_x.push_back(vehicle.getX());
        vehicle_y.push_back(vehicle.getY());
        vehicle_theta.push_back(vehicle.getTheta());

        // Plot the vehicle's current path with orientation
        controller.plot(vehicle_x, vehicle_y, vehicle_theta);

        // Pause for a short time to simulate real-time plotting
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }


    // Keep the plot window open
    plt::show(true);


    return 0;
}
