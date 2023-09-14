#include "StanleyController.h"

StanleyController::StanleyController(vector<Eigen::VectorXd> waypoints) : waypoints(waypoints)
{
}

StanleyController::~StanleyController()
{

}

vector<Eigen::VectorXd> StanleyController::getNewWaypoints() const{
    return new_waypoints;
}

size_t StanleyController::getClosestIndex() const{
    return closest_index;
}

double StanleyController::GetMaxSteer() const{
    return max_steer;
}

double StanleyController::GetTargetIdx() const{
    return target_idx;
}

double StanleyController::GetErrorFrontAxle() const{
    return error_front_axle;
}

double StanleyController::GetPid() const{
    return pid;
}

double StanleyController::GetDelta() const{
    return delta;
}

double StanleyController::GetNormaliceAngle(double angle){
    // ---------------------------------------------------------------------
    // Normalize an angle to [-pi, pi].
    // :param angle: (double)
    // :return: (double) double in radian in [-pi, pi]
    // ---------------------------------------------------------------------
    if (angle > M_PI) angle -= 2 * M_PI;
    if (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

double StanleyController::GetAngleToRadians(double angle_in_degrees){
    // ---------------------------------------------------------------------
    // Cover an angle in degrees to radians.
    // :param angle_in_degrees: (double) 
    // :return: (double) angle in radian
    // ---------------------------------------------------------------------
    double radians = angle_in_degrees * (M_PI / 180.0);
    return radians;
}

double StanleyController::computeDistance(double x1, double y1, double x2, double y2){
    // ---------------------------------------------------------------------
    // Compute the distance between two points.
    // :param x1: (double) x coordinate of point 1
    // :param y1: (double) y coordinate of point 1
    // :param x2: (double) x coordinate of point 2
    // :param y2: (double) y coordinate of point 2
    // :return: (double) distance between two points
    // ---------------------------------------------------------------------
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

// ---------------------------------------------------------------------
// To reduce the amount of waypoints sent to the controller,
// provide a subset of waypoints that are within some 
// lookahead distance from the closest point to the car.
// ---------------------------------------------------------------------

void StanleyController::findClosestWaypoint(double current_x, double current_y,const vector<double>& wp_distance, const vector<int>& wp_interp_hash, const vector<Eigen::VectorXd>& wp_interp)
{
    // ---------------------------------------------------------------------
    // Find the closest waypoint to the car and To reduce the amount of waypoints sent to the controller, provide a subset of waypoints that are within some lookahead distance from the closest point to the car.
    // :param current_x: (double) current x position of the car
    // :param current_y: (double) current y position of the car
    // :param wp_distance: (vector<double>) distance between waypoints
    // :param wp_interp_hash: (vector<int>) hash table of waypoints
    // :param wp_interp: (vector<Eigen::VectorXd>) interpolated waypoints
    // :return: (int) index of the closest waypoint
    // :return: (vector<Eigen::VectorXd>) subset of waypoints
    // ---------------------------------------------------------------------

    closest_index = 0;  
    double closest_distance = computeDistance(waypoints[closest_index][0], waypoints[closest_index][1], current_x, current_y);
    double new_distance = closest_distance;
    size_t new_index = closest_index;

    while (new_distance <= closest_distance) {
        closest_distance = new_distance;
        closest_index = new_index;
        new_index++;
        if (new_index >= waypoints.size()) {  
            break;
        }
        new_distance = computeDistance(waypoints[new_index][0], waypoints[new_index][1], current_x, current_y);
    }

    new_distance = closest_distance;
    new_index = closest_index;

    while (new_distance <= closest_distance) {
        closest_distance = new_distance;
        closest_index = new_index;
        if (new_index == 0) {  // if at the beginning of the waypoints
            break;
        }
        new_index--;
        new_distance = computeDistance(waypoints[new_index][0], waypoints[new_index][1], current_x, current_y);
    }
    
    // Once the closest index is found, return the path that has 1
    // waypoint behind and X waypoints ahead, where X is the index
    // that has a lookahead distance specified by 
    // INTERP_LOOKAHEAD_DISTANCE = 20

    size_t waypoint_subset_first_index = closest_index - 1;
    if (waypoint_subset_first_index < 1) {
        waypoint_subset_first_index = 0;
    }

    size_t waypoint_subset_last_index = closest_index;
    double total_distance_ahead = 0.0;

    while (total_distance_ahead < 2) {
        if (waypoint_subset_last_index >= waypoints.size() || waypoint_subset_last_index >= wp_distance.size()) {
            waypoint_subset_last_index = std::min(waypoints.size(), wp_distance.size()) - 1;
            break;
        }
        total_distance_ahead += wp_distance[waypoint_subset_last_index];
        waypoint_subset_last_index++;
    }

    vector<Eigen::VectorXd> new_waypoints_data( 
        wp_interp.begin() + wp_interp_hash[waypoint_subset_first_index],
        wp_interp.begin() + wp_interp_hash[waypoint_subset_last_index] + 1
    );

    new_waypoints = new_waypoints_data;
}


void StanleyController::computeCrossTrackError(double current_x, double current_y, double current_yaw){
    double fx = current_x + L * cos(current_yaw);
    double fy = current_y + L * sin(current_yaw);
    // Search nearest point index

    // Search nearest point index
    target_idx = 0;
    double min_dist = std::numeric_limits<double>::max();

    for (int i = 0; i < new_waypoints.size(); i++) {
        double dx = fx - new_waypoints[i](0);  // Access x-coordinate of Eigen::VectorXd
        double dy = fy - new_waypoints[i](1);  // Access y-coordinate of Eigen::VectorXd
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < min_dist) {
            min_dist = dist;
            target_idx = i;
        }
    }

    double front_axle_vec[2] = {-std::cos(current_yaw + M_PI / 2), -std::sin(current_yaw + M_PI / 2)};
    error_front_axle = (fx - new_waypoints[target_idx](0)) * front_axle_vec[0] + (fy - new_waypoints[target_idx](1)) * front_axle_vec[1];

}


void StanleyController::computePID(double target, double current){
    pid = Kp * (target - current);
}


void StanleyController::computeSteeringAngle(double current_yaw, double v){

    double yaw_path = std::atan2(new_waypoints.back()[1]-new_waypoints.front()[1], new_waypoints.back()[0]-new_waypoints.front()[0]);
    double theta_e = GetNormaliceAngle(yaw_path - current_yaw);
    // theta_e corrects the heading error
    double theta_d = atan2(K * error_front_axle, v);
    delta = theta_e - theta_d;
}


// void StanleyController::Controller_Stanley(const double yaw,const std::vector<Eigen::VectorXd> waypoints, const double x, const double y, const double v, const double a, const double steer, const double dt){
//     // Calc front axle position
//     double fx = x + L * cos(yaw);
//     double fy = y + L * sin(yaw);
//     // Search nearest point index
//     double min_dist = 1e9;
//     int min_index = 0;

//     for (int i = 0; i < waypoints.size(); i++) {
//         double dx = fx - waypoints[i][0];
//         double dy = fy - waypoints[i][1];
//         double dist = sqrt(pow(dx, 2) + pow(dy, 2));
//         if (dist < min_dist) {
//             min_dist = dist;
//             min_index = i;
//         }
//     }
//     return min_index;

// }





    // double alpha = atan2(fy - waypoints[0][1], fx - waypoints[0][0]) - waypoints[0][2];
    // double crosstrack_error = sin(alpha) * sqrt(pow(fx - waypoints[0][0], 2) + pow(fy - waypoints[0][1], 2));
    // double theta_e = GetNormaliceAngle(yaw - waypoints[0][2]);
    // double theta_d = atan2(K * crosstrack_error, v);
    // double steer_angle = theta_e + theta_d;
    // steer_angle = GetNormaliceAngle(steer_angle);
    // steer_angle = std::max(std::min(steer_angle, max_steer), -max_steer);
    // double delta = steer_angle - steer;
    // delta = GetNormaliceAngle(delta);
    // double a = Kp * (v - waypoints[0][3]);
    // return delta, a; 



        // // # Calc crosstrack error
    // double alpha = atan2(fy - waypoints[min_index][1], fx - waypoints[min_index][0]) - waypoints[min_index][2];
    // double crosstrack_error = sin(alpha) * sqrt(pow(fx - waypoints[min_index][0], 2) + pow(fy - waypoints[min_index][1], 2));
    // // # Calc alongtrack error
    // double alongtrack_error = cos(alpha) * sqrt(pow(fx - waypoints[min_index][0], 2) + pow(fy - waypoints[min_index][1], 2));
    // // # Calc theta_e
    // double theta_e = GetNormaliceAngle(yaw - waypoints[min_index][2]);
    // // # Calc theta_d
    // double theta_d = atan2(K * crosstrack_error, v);
    // // # Calc steer angle
    // double steer_angle = theta_e + theta_d;
    // steer_angle = GetNormaliceAngle(steer_angle);
    // steer_angle = std::max(std::min(steer_angle, max_steer), -max_steer);
    // double delta = steer_angle - steer;
    // delta = GetNormaliceAngle(delta);
    // double a = Kp * (v - waypoints[min_index][3]);
    // return delta, a;