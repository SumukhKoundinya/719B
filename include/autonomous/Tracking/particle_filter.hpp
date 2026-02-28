#pragma once

#include <vector>
#include <cmath>
#include <random>

struct Particle {
    double x;
    double y;
    double theta;
    double weight;
};

class ParticleFilter {
private:
    int N;
    std::vector<Particle> particles;

    double estX;
    double estY;
    double estTheta;

    double sigmaMotion;
    double sigmaTurn;
    double sigmaSensor;

    std::default_random_engine generator;
    double gaussian(double mean, double stddev);
    double raycastToWall(double x, double y, double angle);

public:
    ParticleFilter(int numParticles = 300);

    void init(double startX, double startY, double startTheta);

    void update(double deltaS, double deltaTheta,
                double frontReading, double backReading,
                double leftReading,  double rightReading);

    double getX() const;
    double getY() const;
    double getTheta() const;
    double getPose() const;

    void prediction(double deltaS, double deltaTheta);
    void measurement(double frontReading, double backReading,
                     double leftReading,  double rightReading);
    void normalizeWeights();
    void computeEstimate();
    void resample();
};