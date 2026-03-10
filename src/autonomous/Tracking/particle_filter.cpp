#include "main.h"
#include <algorithm>
#include <numeric>
#include <random>

ParticleFilter::ParticleFilter(int numParticles) {
    N = numParticles;
    sigmaMotion = 0.1;
    sigmaTurn = 0.005;
    sigmaSensor = 3.0;
}

double ParticleFilter::gaussian(double mean, double stddev) {
    std::normal_distribution<double> dist(mean, stddev);
    return dist(generator);
}

void ParticleFilter::init(double startX, double startY, double startTheta) {
    particles.clear();
    for (int i = 0; i < N; i++) {
        Particle p;
        p.x = startX + gaussian(0, 0.5);
        p.y = startY + gaussian(0, 0.5);
        p.theta = startTheta + gaussian(0, 0.02);
        p.weight = 1.0 / N;
        particles.push_back(p);
    }
    estX = startX;
    estY = startY;
    estTheta = startTheta;
}

void ParticleFilter::update(double deltaS, double deltaTheta,
                             double frontReading, double backReading,
                             double leftReading,  double rightReading) {
    prediction(deltaS, deltaTheta);
    measurement(frontReading, backReading, leftReading, rightReading);
    normalizeWeights();
    resample();
    computeEstimate();
}

void ParticleFilter::prediction(double deltaS, double deltaTheta) {
    for (auto &p : particles) {
        double noisyS     = deltaS     + gaussian(0, sigmaMotion);
        double noisyTheta = deltaTheta + gaussian(0, sigmaTurn);
        p.theta += noisyTheta;
        p.x += noisyS * cos(p.theta);
        p.y += noisyS * sin(p.theta);
    }
}

double ParticleFilter::raycastToWall(double x, double y, double angle) {
    double dx = cos(angle);
    double dy = sin(angle);
    double tMin = 1e9;

    if (fabs(dx) > 1e-6) {
        double t1 = -x / dx;
        double t2 = (144.0 - x) / dx;
        if (t1 > 0) tMin = std::min(tMin, t1);
        if (t2 > 0) tMin = std::min(tMin, t2);
    }

    if (fabs(dy) > 1e-6) {
        double t1 = -y / dy;
        double t2 = (144.0 - y) / dy;
        if (t1 > 0) tMin = std::min(tMin, t1);
        if (t2 > 0) tMin = std::min(tMin, t2);
    }

    return tMin;
}

void ParticleFilter::measurement(double frontReading, double backReading,
                                  double leftReading,  double rightReading) {
    for (auto &p : particles) {
        if (p.x < 0 || p.x > 144 || p.y < 0 || p.y > 144) {
            p.weight = 0;
            continue;
        }

        double expFront = raycastToWall(p.x, p.y, p.theta);
        double expBack  = raycastToWall(p.x, p.y, p.theta + M_PI);
        double expLeft  = raycastToWall(p.x, p.y, p.theta + M_PI / 2.0);
        double expRight = raycastToWall(p.x, p.y, p.theta - M_PI / 2.0);

        auto w = [&](double measured, double expected) {
            double e = measured - expected;
            return exp(-(e * e) / (2.0 * sigmaSensor * sigmaSensor));
        };

        p.weight = w(frontReading, expFront)
                 * w(backReading,  expBack)
                 * w(leftReading,  expLeft)
                 * w(rightReading, expRight);
    }
}

void ParticleFilter::normalizeWeights() {
    double sumW = 0;
    for (auto &p : particles) sumW += p.weight;

    if (sumW < 1e-10) {
        for (auto &p : particles) p.weight = 1.0 / N;
        return;
    }

    for (auto &p : particles) p.weight /= sumW;
}

void ParticleFilter::resample() {
    std::vector<Particle> newParticles;
    std::vector<double> cumulative;
    double sum = 0;

    for (auto &p : particles) {
        sum += p.weight;
        cumulative.push_back(sum);
    }

    std::uniform_real_distribution<double> dist(0, 1.0);
    for (int i = 0; i < N; i++) {
        double r = dist(generator);
        auto it = std::lower_bound(cumulative.begin(), cumulative.end(), r);
        int index = std::distance(cumulative.begin(), it);
        newParticles.push_back(particles[index]);
    }

    particles = newParticles;
}

void ParticleFilter::computeEstimate() {
    double newX = 0, newY = 0, newTheta = 0;

    for (auto &p : particles) {
        newX     += p.x     * p.weight;
        newY     += p.y     * p.weight;
        newTheta += p.theta * p.weight;
    }

    double alpha = 0.18;
    estX     = alpha * newX     + (1 - alpha) * estX;
    estY     = alpha * newY     + (1 - alpha) * estY;
    estTheta = alpha * newTheta + (1 - alpha) * estTheta;
}

double ParticleFilter::getX()     const { return estX; }
double ParticleFilter::getY()     const { return estY; }
double ParticleFilter::getTheta() const { return estTheta; }
double ParticleFilter::getPose()  const { return estX, estY, estTheta; }
