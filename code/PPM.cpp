#include "PPM.h"
#include "Scene.h"
#include "Object.h"
#include "Light.h"
#include "Texture.h"
#include <ctime>
#include <cstdlib>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>
using namespace std;

void PPM::run(Scene *scene)
{
	_scene = scene;
	int height = _scene->getCamera()->getHeight();
	int width = _scene->getCamera()->getWidth();
	_image.resize(height);
	for (int i = 0; i < height; i++)
		_image[i].resize(width);

	int startTime = clock();
	// 光线跟踪
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			HitPoint hp(i, j, Vec3(1, 1, 1));
			Vec3 origin = _scene->getCamera()->getPosition();

			if (_scene->getCamera()->getAperture() > EPSI) {
				int aperIter = _scene->getCamera()->getAperIter();
				hp.weight = hp.weight / aperIter;
				_image[i][j] = 0;
				for (int s = 0; s < aperIter; ++s) {
					Vec3 ori, dir;
					_scene->getCamera()->getRay(i, j, ori, dir);
					_image[i][j] += traceRay(hp, ori, dir, 0);
				}
				_image[i][j] /= aperIter;
			} 
			else {
				Vec3 direction = _scene->getCamera()->getRay(i, j);
				_image[i][j] = traceRay(hp, origin, direction, 0);
			}
		}
	}
	cout << "Time used: " << clock() - startTime << endl;
	_kdTree.load(_hpList.size(), _hpList.data());
	_kdTree.build();

	// 光子追踪
	for (int i = 0; i < MAX_PPM_ITER; i++)
	{
		double totalPower = 0.0;
		for (Light *light : _scene->getLightList()) {
			totalPower += light->getColor().power();
		}

		double photonPower = totalPower / MAX_PHOTON_NUM;

		for (Light *light : _scene->getLightList()) {
			int photonNum = light->getColor().power() / photonPower;
			Vec3 photonColor = light->getColor() / photonNum;
			
			srand(int(time(NULL)));
			for (int j = 0; j < photonNum; j++) {
				Vec3 origin = light->getRandomOrigin();
				Vec3 direction = Vec3::getRandomVector();
				Photon photon(origin, direction, photonColor);
				tracePhoton(photon, 0);
			}
		}
		cout << "Time used: " << clock() - startTime << endl;

		updateKdTree();
		renderImage(i + 1);
		saveImage("update.jpg");
	}
}

Vec3 PPM::traceRay(HitPoint hp, const Vec3 &origin, const Vec3 &direction, int depth)	// assert: |direction| == 1
{
	Vec3 backgroundColor = _scene->getBackgroundColor(); 
	if (depth > MAX_DEPTH) {
		_envList.push_back(hp);
		return backgroundColor;
	}
	
	int objectNum = _scene->getObjectNum();
	int lightNum = _scene->getLightNum();
	vector<Object*> &objectList = _scene->getObjectList();
	vector<Light*> &lightList = _scene->getLightList();

	double maxt = RAND_MAX;
	Object *object = NULL;
	int intersectedType = NO_INTERSECT, tmp = NO_INTERSECT;
	for (int i = 0; i < objectNum; ++i) {
		if ((tmp = objectList[i]->intersect(origin, direction, maxt))) {
			object = objectList[i], intersectedType = tmp;
		}
	}
	
	if (!object) {
		_envList.push_back(hp);
		return backgroundColor;
	}
	
	Vec3 P = origin + direction * maxt;
	Vec3 N = object->getNormal(P);
	Vec3 color = Vec3(0, 0, 0);
	if (object->getDiffuse() > EPSI || object->getSpecular() > EPSI)	{
		Vec3 objectCol = object->getColor() * object->getTextureColor(P);
		double diffuse = object->getDiffuse();
		hp.object = object;
		hp.position = P; hp.normal = N; 
		hp.weight *= objectCol * diffuse; 
		hp.radius2 = INIT_RADIUS * INIT_RADIUS;
		_hpList.push_back(hp);
			
		// Phong
		for (int i = 0; i < lightNum; ++i) {
			bool shade = false;
			Vec3 L = lightList[i]->getPosition() - P;
			double lightDist = L.length();

			for (int j = 0; j < objectNum; j++) {
				int type = NO_INTERSECT;
				if (objectList[j] != object && (type = objectList[j]->intersect(P, L.normalized(), lightDist))) { 
					shade = true; 
					break; 
				}
			}
			if (shade) continue;

			L = L.normalized();
			Vec3 diffuse = lightList[i]->evalDiffuse(N, L, objectCol, object->getDiffuse());
			Vec3 V = origin - P;
			V = V.normalized();
			Vec3 specular = lightList[i]->evalSpecular(N, L, V, object->getColor(), object->getShininess(), object->getSpecular());

			color += diffuse + specular;
		}
	}
	if (object->getReflectivity() > EPSI) {// reflection	
		Vec3 objectCol = object->getColor();
		double reflectivity = object->getReflectivity();
		hp.weight *= objectCol * reflectivity;
		Vec3 reflected = direction.reflected(N);
		Vec3 col = traceRay(hp, P, reflected, depth + 1);

		color += reflectivity * col * objectCol;
	}
	if (object->getRefractivity() > EPSI) {// refraction
		Vec3 objectCol = object->getColor();
		double refractivity = object->getRefractivity();
		hp.weight *= objectCol * refractivity;
		double ior = object->getIOR();
		double n = (intersectedType == INTERSECT_IN) ? ior : (1 / ior);
		Vec3 refracted = direction.refracted((intersectedType == INTERSECT_IN ? -N : N), n);
		Vec3 col = traceRay(hp, P, refracted, depth + 1);		

		color += refractivity * col * objectCol;
	}
	return color;
}

void PPM::tracePhoton(Photon &photon, int depth)
{
	if (depth > MAX_DEPTH) return;
	int objectNum = _scene->getObjectNum();
	vector<Object*> &objectList = _scene->getObjectList();

	double maxt = RAND_MAX;
	Object *object = NULL;
	int intersectedType = NO_INTERSECT, tmp = NO_INTERSECT;
	for (int i = 0; i < objectNum; ++i)
		if ((tmp = objectList[i]->intersect(photon.origin, photon.direction, maxt)))
			object = objectList[i], intersectedType = tmp;
	if (!object) return;

	photon.position = photon.origin + photon.direction * maxt;
	photon.object = object;

	if (object->getDiffuse() > EPSI)
		_kdTree.insert(photon);

	Photon newPhoton = photon;
	newPhoton.color *= object->getColor();
	newPhoton.origin = photon.position;

	double randomNum = rand01();
	double mark1 = object->getDiffuse() + object->getSpecular(), 
		   mark2 = mark1 + object->getReflectivity(),
		   mark3 = mark2 + object->getRefractivity();
	if (randomNum < mark1) {
		Vec3 N = object->getNormal(photon.position);
		newPhoton.direction = Vec3::getCosRandomVector(N);
		tracePhoton(newPhoton, depth + 1);
	} 
	else if (randomNum < mark2) {
		Vec3 N = object->getNormal(photon.position);
		newPhoton.direction = photon.direction.reflected(N);
		tracePhoton(newPhoton, depth + 1);
	} 
	else {
		Vec3 N = object->getNormal(photon.position);
		double ior = object->getIOR();
		double n = (intersectedType == INTERSECT_IN) ? ior : (1 / ior);
		newPhoton.direction = photon.direction.refracted((intersectedType == INTERSECT_IN ? -N : N), n);
		tracePhoton(newPhoton, depth + 1);
	}
}

void PPM::updateKdTree() {
	int hpNum = 0;
	HitPoint *hpList = _kdTree.getData(hpNum);

	for (int i = 0; i < hpNum; i++) {
		hpList[i].update(ALPHA);
	}		
	_kdTree.update();
}

void PPM::renderImage(int iterNum) {
	int height = _scene->getCamera()->getHeight();
	int width = _scene->getCamera()->getWidth();

	for (int i = 0; i < height; i++)
		for (int j = 0; j < width; j++)
			_image[i][j] = Vec3();

	int hpNum = 0;
	HitPoint *hpList = _kdTree.getData(hpNum);
	for (int i = 0; i < hpNum; i++) {
		HitPoint hp = hpList[i];
		Vec3 color = 5000.0 * hp.totalFlux / (hp.radius2 * iterNum);
		_image[hp.x][hp.y] += color * hp.weight;
	}
	Vec3 backgroundColor = _scene->getBackgroundColor();
	for (int i = 0; i < _envList.size(); ++i) {
		HitPoint hp = _envList[i];
		_image[hp.x][hp.y] += hp.weight * backgroundColor;
	}
}

void PPM::saveImage(const string &filename) {
	cv::Mat_<cv::Vec3b> img;
	int height = _scene->getCamera()->getHeight();
	int width = _scene->getCamera()->getWidth();
	img.create(height, width);

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			img(i, j)[0] = std::min(_image[i][j][2], 1.0) * 255;
			img(i, j)[1] = std::min(_image[i][j][1], 1.0) * 255;
			img(i, j)[2] = std::min(_image[i][j][0], 1.0) * 255;
		}
	}
	cv::imwrite(filename, img);
}