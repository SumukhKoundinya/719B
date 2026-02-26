#include "main.h"
#include <algorithm>
#include <numeric>
#include <random>

ParticleFilter::ParticleFilter(int numParticles) {
    N = numParticles;
    sigmaMotion = 0.1;
    sigmaTurn = 0.005;
    sigmaSensor = 3.0; // old 0.2
}

double ParticleFilter::gaussian(double mean, double stddev) {
    std::normal_distribution<double> dist(mean, stddev);
    return dist(generator);
}

void ParticleFilter::init(double startX, double startY, double startTheta) {
    particles.clear();
    for (int i =0; i < N; i++) {
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

void ParticleFilter::update(double deltaS, double deltaTheta, double horizReading, double vertReading) {
    /*prediction(deltaS, deltaTheta);
    computeEstimate();*/

    prediction(deltaS, deltaTheta);
    measurement(horizReading, vertReading);
    normalizeWeights();
    resample();
    computeEstimate();
}

void ParticleFilter::prediction(double deltaS, double deltaTheta) {
    for (auto &p : particles) {
        double noisyS = deltaS + gaussian(0, sigmaMotion);
        double noisyTheta = deltaTheta + gaussian(0, sigmaTurn);
        p.theta += noisyTheta;
        p.x += noisyS * cos(p.theta);
        p.y += noisyS * sin(p.theta);
    }
}

void ParticleFilter::measurement(double horizReading, double vertReading) {
    for (auto &p : particles) {
        double expectedX = 144 - p.x;
        double expectedY = 144 - p.y;

        double errorX = horizReading - expectedX;
        double errorY = vertReading - expectedY;

        double wX = exp(-(errorX*errorX)/(2*sigmaSensor*sigmaSensor));
        double wY = exp(-(errorY*errorY)/(2*sigmaSensor*sigmaSensor));

        p.weight = wX * wY;
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
        auto it = std::lower_bound(cumulative.begin() , cumulative.end(), r);
        int index = std::distance(cumulative.begin(), it);
        newParticles.push_back(particles[index]);
    }

    particles = newParticles;
}

void ParticleFilter::computeEstimate() {
    double newX = 0, newY = 0, newTheta = 0;

    for (auto &p : particles) {
        newX += p.x * p.weight;
        newY += p.y * p.weight;
        newTheta += p.theta * p.weight;
    }

    double alpha = 0.18; // 0 = never update, 1 = no smoothing
    estX = alpha * newX + (1 - alpha) * estX;
    estY = alpha * newY + (1 - alpha) * estY;
    estTheta = alpha * newTheta + (1 - alpha) * estTheta;
}

double ParticleFilter::getX() const { return estX; }
double ParticleFilter::getY() const { return estY; }
double ParticleFilter::getTheta() const { return estTheta; }
