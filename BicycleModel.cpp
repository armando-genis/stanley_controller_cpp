#include "BicycleModel.h"


BicycleModel::BicycleModel(double x0, double y0, double theta0, double v0) : x(x0), y(y0), theta(theta0), v(v0) {}

void BicycleModel::update(double delta, double a, double dt, double max_steer) {
    delta = std::fmin(std::fmax(delta, -max_steer), max_steer);
    double beta = std::atan(0.5 * std::tan(delta));
    x += v * std::cos(yaw + beta) * dt;
    y += v * std::sin(yaw + beta) * dt;
    yaw += (v * std::cos(beta) * std::tan(delta) / L) * dt;
    yaw = GetNormaliceAngle(yaw);
    v += a * dt;

}

double BicycleModel::getX() const { return x; }
double BicycleModel::getY() const { return y; }
double BicycleModel::getTheta() const { return theta; }
double BicycleModel::getV() const { return v; }
double BicycleModel::getYaw() const { return yaw; }
double BicycleModel::GetNormaliceAngle(double angle){
    // ---------------------------------------------------------------------
    // Normalize an angle to [-pi, pi].
    // :param angle: (double)
    // :return: (double) double in radian in [-pi, pi]
    // ---------------------------------------------------------------------
    if (angle > M_PI) angle -= 2 * M_PI;
    if (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}
