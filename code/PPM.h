#ifndef PPM_H
#define PPM_H

#include "Vec3.h"
#include "KdTree.h"
#include "Object.h"
#include "Scene.h"
#include <vector>
#include <string>

#define MAX_DEPTH 8
#define MAX_PPM_ITER 10000
#define MAX_PHOTON_NUM 5000000
#define INIT_RADIUS 1.2
#define ALPHA 0.7

class PPM {
private:
	Scene *_scene;
	std::vector<std::vector<Vec3>> _image;
	std::vector<HitPoint> _hpList;
	std::vector<HitPoint> _envList;
	KdTree _kdTree;

public:
	void run(Scene *scene);
	Vec3 traceRay(HitPoint hp, const Vec3 &origin, const Vec3 &direction, int depth);
	void tracePhoton(Photon &photon, int depth);
	void saveImage(const std::string &filename);
	void updateKdTree();
	void renderImage(int iterNum);
};

#endif