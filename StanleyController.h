#ifndef STANLEYCONTROLLER_H
#define STANLEYCONTROLLER_H
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
using namespace std;


class StanleyController
{
private:
    vector<Eigen::VectorXd> waypoints; 
    const double L = 0.0; // [m] Wheel base of vehicle
    const double K = 0.5;  // control gain
    const double Kp = 1.0; // speed proportional gain
    const double max_steer = GetAngleToRadians(30); // [rad] max steering angle
    size_t closest_index; // Closaest waypoint index
    vector<Eigen::VectorXd> new_waypoints; //subset of waypoints

public:
    StanleyController(vector<Eigen::VectorXd> waypoints);
    ~StanleyController();
    // function to compute the main functions of the controller
    double GetNormaliceAngle(double angle);
    double GetAngleToRadians(double angle_in_degrees);
    double computeDistance(double x1, double y1, double x2, double y2);
    // function to find the closest waypoint to the car
    void findClosestWaypoint(double current_x, double current_y,const vector<double>& wp_distance, const vector<int>& wp_interp_hash, const vector<Eigen::VectorXd>& wp_interp);
    // function to return data
    vector<Eigen::VectorXd> getNewWaypoints() const;
    size_t getClosestIndex() const;

    // void Controller();
};

#endif
