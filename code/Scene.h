#ifndef SCENE_H
#define SCENE_H

#include "Vec3.h"
#include "Camera.h"
#include "Object.h"
#include "Light.h"
#include <vector>

class Scene
{
private:
	std::vector<Object *> _objectList;
	int _objectNum;
	std::vector<Light *> _lightList;
	int _lightNum;
	Camera *_camera;
	Vec3 _bgColor;

public:
	Scene() : _objectNum(0), _lightNum(0), _camera(new Camera) {}
	void addObject(Object *object) { _objectList.push_back(object); _objectNum++; }
	void addLight(Light *light) { _lightList.push_back(light); _lightNum++; }
	void setBackgroundColor(Vec3 backgroundColor) { _bgColor = backgroundColor; }
	Vec3 getBackgroundColor() const { return _bgColor; }
	Camera *getCamera() const { return _camera; }
	std::vector<Object *> &getObjectList() { return _objectList; }
	std::vector<Light *> &getLightList() { return _lightList; }
	int getObjectNum() const { return _objectNum; }
	int getLightNum() const { return _lightNum; }
};

#endif