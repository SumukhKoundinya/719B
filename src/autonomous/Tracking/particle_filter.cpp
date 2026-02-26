#include "main.h"
#include <algorithm>
#include <numeric>
#include <random>

ParticleFilter::ParticleFilter(int numParticles) {
    N = numParticles;
    sigmaMotion = 0.1;
    sigmaTurn = 0.005;
    sigmaSensor = 0.2;
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

void ParticleFilter::update(double deltaS, double deltaTheta) {
    prediction(deltaS, deltaTheta);
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
    estX = 0;
    estY = 0;
    estTheta = 0;

    for (auto &p : particles) {
        estX += p.x * p.weight;
        estY += p.y * p.weight;
        estTheta += p.theta * p.weight;
    }
}

double ParticleFilter::getX() const { return estX; }
double ParticleFilter::getY() const { return estY; }
double ParticleFilter::getTheta() const { return estTheta; }
