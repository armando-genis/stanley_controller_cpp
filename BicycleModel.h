#ifndef BICYCLEMODEL_H
#define BICYCLEMODEL_H

class BicycleModel {
private:
    double x, y, theta, v;
    const double L = 2.0;

public:
    BicycleModel(double x0, double y0, double theta0, double v0);
    void update(double delta, double a, double dt);
    double getX() const;
    double getY() const;
    double getTheta() const;
    double getV() const;
};

#endif
