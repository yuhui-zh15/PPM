#ifndef HITPOINT_H
#define HITPOINT_H

#include "Vec3.h"
#include "Object.h"

struct Photon {
    Vec3 origin, position, direction, color;
    Object *object;
    Photon(Vec3 origin, Vec3 direction, Vec3 color): origin(origin), direction(direction), color(color), object(NULL) {}
};

struct HitPoint {
    Vec3 position, normal, weight, totalFlux;
    int x, y;
    double radius2, maxRadius2, N, M;
    Object *object;
    HitPoint() {}
    HitPoint(int x, int y, Vec3 initWeight) : x(x), y(y), N(0), M(0), totalFlux(Vec3(0.0, 0.0, 0.0)), weight(initWeight) {}
    void update(double alpha) {
        if (N < EPSI && M < EPSI) return;
        double frac = (N + alpha * M) / (N + M);
        radius2 *= frac; totalFlux *= frac;
        N += alpha * M; M = 0;
    }
    friend std::ostream &operator<< (std::ostream &os, const HitPoint &hp) {
        os  << "x = " << hp.x << ", y = " << hp.y
            << ", N = " << hp.N << ", M = " << hp.M << ", totalFlux = " << hp.totalFlux << ", radius2 = " << hp.radius2 
            << ", weight = " << hp.weight << ", position = " << hp.position << ", normal = " << hp.normal << std::endl;
        return os;
    }
};

#endif