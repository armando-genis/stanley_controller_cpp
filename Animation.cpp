#include <iostream>
#include <Eigen/Dense>
#include <chrono>
#include <thread>
#include <vector>


#include "Linear_Interpolation.h"
#include "matplotlibcpp.h"  // Include the matplotlib-cpp header
#include "BicycleModel.h"
#include "StanleyController.h"

using namespace std;
namespace plt = matplotlibcpp;



void plot_waypoints( const std::vector<Eigen::VectorXd> waypoints){
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

void plot_waypoints_interpolation( const std::vector<Eigen::VectorXd> waypoints){
    std::vector<double> x_vals, y_vals;
    for (const auto& waypoint : waypoints) {
        x_vals.push_back(waypoint[0]);
        y_vals.push_back(waypoint[1]);
    }

    plt::plot(x_vals, y_vals, "bo--");  // Plot blue circles connected by lines
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::title("Waypoints interpolated");
    plt::grid(true);
    plt::show();
}

void animation_car(const vector<Eigen::VectorXd> waypoints,const vector<double>& wp_distance, const vector<int>& wp_interp_hash, const vector<Eigen::VectorXd>& wp_interp){
    plt::ion();  
    BicycleModel vehicle(0.0, 0.0, 0.0, 1.0);
    StanleyController controller(waypoints);

    std::vector<double> vehicle_x, vehicle_y, vehicle_theta;
        double dt = 0.1;
    for (int i = 0; i < 500; i++) {

        vehicle.update(-3, 0.2, 0.1); // Steering angle, acceleration, time step
        std::cout << vehicle.getYaw() << std::endl;

        // Store vehicle data for plotting
        vehicle_x.push_back(vehicle.getX());
        vehicle_y.push_back(vehicle.getY());
        vehicle_theta.push_back(vehicle.getTheta());

        // Find the closest waypoint to the vehicle
        controller.findClosestWaypoint(vehicle.getX(), vehicle.getY(), wp_distance, wp_interp_hash, wp_interp);

        plt::clf();
        vector<double> wp_x, wp_y;
        for (const auto& waypoint : waypoints) {
            wp_x.push_back(waypoint[0]);
            wp_y.push_back(waypoint[1]);
        }
        vector<double> wp_x2, wp_y2;
        for (const auto& waypoint : controller.getNewWaypoints()) {
            wp_x2.push_back(waypoint[0]);
            wp_y2.push_back(waypoint[1]);
        }

        plt::plot(wp_x, wp_y, "ro");
        plt::plot(wp_x2, wp_y2, "go");
        plt::plot(vehicle_x, vehicle_y, "b--");


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


    // Keep the plot window open
    plt::show(true);
    
}



int main() {
    std::vector<double> x = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 6.0,6.0,6.0,6.0,6.0,6.0,6.0, 5.0,4.0,3.0,2.0,1.0,0.0,-1.0,-2.0,-3.0,-4.0,-5.0,-6.0,-7.0,-7.0,-7.0,-7.0,-7.0,-7.0,-7.0,-7.0, -6.0,-5.0,-4.0,-3.0,-2.0};
    std::vector<double> y = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,2.0,3.0,4.0,5.0,6.0,7.0, 7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0, 6.0,5.0,4.0,3.0,2.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0};
    
    Linear_Interpolation linear_interpolation(x, y, 0.01);
    linear_interpolation.interpolateWaypoints();
    std::vector<Eigen::VectorXd> wp_interp = linear_interpolation.getWp_interp();
    std::vector<int> wp_interp_hash = linear_interpolation.getWp_interp_hash();
    std::vector<double> wp_distance = linear_interpolation.getWp_distance();
    std::cout << "wp_interp size: " << wp_interp.size() << std::endl;
    std::cout << "wp_interp_hash size: " << wp_interp_hash.size() << std::endl;
    std::vector<Eigen::VectorXd> waypoints = linear_interpolation.getWaypoints();
    // plot_waypoints(waypoints);
    // plot_waypoints_interpolation(wp_interp);
    animation_car(waypoints, wp_distance, wp_interp_hash, wp_interp);
}