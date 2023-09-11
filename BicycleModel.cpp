#include "BicycleModel.h"
#include <cmath>

BicycleModel::BicycleModel(double x0, double y0, double theta0, double v0) : x(x0), y(y0), theta(theta0), v(v0) {}

void BicycleModel::update(double delta, double a, double dt) {
    double beta = atan(0.5 * tan(delta));
    x += v * cos(theta + beta) * dt;
    y += v * sin(theta + beta) * dt;
    theta += v * cos(beta) * tan(delta) / L * dt;
    v += a * dt;
}

double BicycleModel::getX() const { return x; }
double BicycleModel::getY() const { return y; }
double BicycleModel::getTheta() const { return theta; }
double BicycleModel::getV() const { return v; }
