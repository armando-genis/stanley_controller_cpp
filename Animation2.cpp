#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <numeric>
#include "matplotlibcpp.h"
#include "CubicSpline1D.h"

using namespace std;
namespace plt = matplotlibcpp;

void plot_waypoints(const vector<double>& x_vals, const vector<double>& y_vals, const string& style = "bo-") {
    plt::plot(x_vals, y_vals, style);  
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::title("Waypoints");
    plt::grid(true);
}

int main() {
    // Given waypoints
    std::vector<double> x = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 6.0,6.0,6.0,6.0,6.0,6.0,6.0, 5.0,4.0,3.0,2.0,1.0,0.0,-1.0,-2.0,-3.0,-4.0,-5.0,-6.0,-7.0,-7.0,-7.0,-7.0,-7.0,-7.0,-7.0,-7.0, -6.0,-5.0,-4.0,-3.0,-2.0};
    std::vector<double> y = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,2.0,3.0,4.0,5.0,6.0,7.0, 7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0,7.0, 6.0,5.0,4.0,3.0,2.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0};

    // Generating a list of indices for the waypoints:
    std::vector<double> t_values(x.size());
    std::iota(t_values.begin(), t_values.end(), 0);  // fills t_values with increasing values starting from 0

    // Create the splines using these indices as the independent variable
    CubicSpline1D spline_x(t_values, x);
    CubicSpline1D spline_y(t_values, y);

    // Interpolate waypoints using the t_values range
    std::vector<double> x_new, y_new;
    for (double t = 0; t < t_values.back(); t += 0.01) {
        x_new.push_back(spline_x.calc_der0(t));
        y_new.push_back(spline_y.calc_der0(t));
    }

    // Plot original waypoints
    plot_waypoints(x, y, "bo-");

    // Plot interpolated waypoints
    plot_waypoints(x_new, y_new, "r-");

    plt::show();

    return 0;
}
