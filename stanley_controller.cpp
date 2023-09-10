#include <iostream>
#include <vector>
#include <Eigen/Dense>

// ----- TO DISPLAY THE WAYPOINTS ------
#include "matplotlibcpp.h"  // Include the matplotlib-cpp header
namespace plt = matplotlibcpp;  // Use a namespace alias for convenience


class StanleyController {
private:
    std::vector<Eigen::VectorXd> waypoints;

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

    void display() const {

        // for (const auto& waypoint : waypoints) {
        //     std::cout << "(" << waypoint[0] << ", " << waypoint[1] << ")" << std::endl;
        // }

        std::cout << "Total number of waypoints: " << waypoints.size() << std::endl;
    }

    void plot() const {
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

};

int main() {
    std::vector<double> x = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 6.0,6.0,6.0,6.0,6.0,6.0,6.0, 5.0,4.0,3.0,2.0,1.0,0.0,-1.0,-2.0,-3.0,-4.0,-5.0,-6.0,-7.0,-7.0,-7.0,-7.0,-7.0,-7.0,-7.0,-7.0, -6.0,-5.0,-4.0,-3.0,-2.0};
    std::vector<double> y = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,2.0,3.0,4.0,5.0,6.0,7.0, 7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0, 6.0,5.0,4.0,3.0,2.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0};

    StanleyController controller(x, y);
    controller.display();
    controller.plot();

    return 0;
}
