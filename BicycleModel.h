#ifndef BICYCLEMODEL_H
#define BICYCLEMODEL_H
#include <cmath>
class BicycleModel {
private:
    double x, y, theta, v, yaw;
    const double L = 0.40;

public:
    BicycleModel(double x0, double y0, double theta0, double v0);
    void update(double delta, double a, double dt, double max_steer);
    double getX() const;
    double getY() const;
    double getTheta() const;
    double getV() const;
    double getYaw() const;
    double GetNormaliceAngle(double angle);
};

#endif
