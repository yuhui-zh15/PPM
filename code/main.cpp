#include "Object.h"
#include "Light.h"
#include "Scene.h"
#include "PPM.h"
#include "Texture.h"
#include <iostream>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <eigen3/Eigen/Dense>
#include "teapotdata.h"
using namespace Eigen;
using namespace cv;
using namespace std;

int main()
{
	// Scene
	Scene *scene = new Scene;
	scene->setBackgroundColor(Vec3(0.25, 0.25, 0.25));

	// Textures
	Texture *porcelain = new Texture("porcelain.jpeg");
	Texture *floors = new Texture("floor.jpeg");
	Texture *grain = new Texture("grain.jpg");

	// Walls
	Plane *front = new Plane(Vec3(0, 0, 0), Vec3(0, 0, -1));
	front->setMaterial(Vec3(), 1, 0, 0, 0, 0, 1.4);
	front->setTexture(grain);
	scene->addObject(front);

	Plane *back = new Plane(Vec3(0, 0, -170), Vec3(0, 0, 1));
	back->setMaterial(Vec3(0.75, 0.75, 0.75), 1, 0, 0, 0, 0, 1.4);
	back->setTexture(grain);
	scene->addObject(back);

	Plane *left = new Plane(Vec3(- 50, 50, -50), Vec3(1, 0, 0));
	left->setMaterial(Vec3(0.882, 0.537, 0.537), 1, 0, 0, 0, 0, 1.4);
	left->setTexture(grain);
	scene->addObject(left);

	Plane *right = new Plane(Vec3(50, 50, -50), Vec3(-1, 0, 0));
	right->setMaterial(Vec3(0.537, 0.537, 0.882), 1, 0, 0, 0, 0, 1.4);
	right->setTexture(grain);
	scene->addObject(right);

	Plane *top = new Plane(Vec3(0, 100, -50), Vec3(0, -1, 0));
	top->setMaterial(Vec3(0.75, 0.75, 0.75), 1, 0, 0, 0, 0, 1.4);
	top->setTexture(grain);
	scene->addObject(top);

	Plane* bottom = new Plane(Vec3(0, 0, 0), Vec3(0, 1, 0));
	bottom->setMaterial(Vec3(0.75, 0.75, 0.75), 1, 0, 0, 0, 0, 1.4);
	bottom->setTexture(floors);
	scene->addObject(bottom);

	// Light
	DirectionalLight *light = new DirectionalLight(Vec3(), Vec3(0, 95, -100), Vec3(1, 1, 1) * 1);
	scene->addLight(light);

	// Objects
	Vector3d** teapot = createTeapot();
	Bezier** teapotBezier = new Bezier*[16];
	for (int i = 0; i < 32; i++) {
		teapotBezier[i] = new Bezier(Vec3(-25, 15, -140), teapot[i]);
		teapotBezier[i]->setMaterial(Vec3(1, 1, 1) * .999, 0, 0, 0, 1, 0, 1.4);
		scene->addObject(teapotBezier[i]);
	}

	Bezier* drop1 = new Bezier(Vec3(3.1, 13.9, -140), createDrop());
	drop1->setMaterial(Vec3(1, 1, 1) * 0.999, 0, 0, 0, 1, 1.4);
	scene->addObject(drop1);

	Bezier* drop2 = new Bezier(Vec3(3.1, 6.9, -140), createDrop());
	drop2->setMaterial(Vec3(1, 1, 1) * 0.999, 0, 0, 0, 1, 1.4);
	scene->addObject(drop2);

	Sphere *glass = new Sphere(Vec3(22, 16, -110), 16);
	glass->setMaterial(Vec3(1, 1, 1) * 0.999, .8, 0, 0, .2, 0, 1.4);
	glass->setTexture(porcelain);
	scene->addObject(glass);

	Sphere *ball1 = new Sphere(Vec3(7, 5, -90), 5);
	ball1->setMaterial(Vec3(0.75, 0.5, 0.5), 0, 0, 0, 0, 1, 1.4);
	scene->addObject(ball1);

	Sphere *ball2 = new Sphere(Vec3(0, 5, -100), 5);
	ball2->setMaterial(Vec3(0.5, 0.75, 0.5), 0, 0, 0, 0, 1, 1.4);
	scene->addObject(ball2);

	Sphere *ball3 = new Sphere(Vec3(2, 5, -110), 5);
	ball3->setMaterial(Vec3(0.75, 0.75, 0.5), 0, 0, 0, 0, 1, 1.4);
	scene->addObject(ball3);

	Sphere *ball4 = new Sphere(Vec3(16, 5, -87), 5);
	ball4->setMaterial(Vec3(0.75, 0.5, 0.75), 0, 0, 0, 0, 1, 1.4);
	scene->addObject(ball4);

	// PPM 
	PPM *ppm = new PPM;
	ppm->run(scene);
	ppm->saveImage("test.jpg");

	return 0;
}
