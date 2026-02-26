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

public:
    ParticleFilter(int numParticles = 300);

    void init(double startX, double startY, double startTheta);

    void update(double deltaS, double deltaTheta, double horizReading, double vertReading);
    
    double getX() const;
    double getY() const;
    double getTheta() const;

    void prediction(double deltaS, double deltaTheta);
    void measurement(double horizReading, double vertReading);
    void normalizeWeights();
    void computeEstimate();
    void resample();
};
