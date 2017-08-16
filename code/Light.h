#ifndef LIGHT_H
#define LIGHT_H

#include "Vec3.h"

class Light
{
protected:
	Vec3 _position, _color;

public:
	Light() {}
	Light(Vec3 position, Vec3 color)
		: _position(position), _color(color) {}
	void setColor(Vec3 color) { _color = color; }
	Vec3 getColor() const { return _color; }
	void setPosition(Vec3 position) { _position = position; }
	Vec3 getPosition() const { return _position; }
	virtual Vec3 getRandomOrigin() const = 0;
	virtual Vec3 evalAmbient(Vec3 color) const = 0;
	virtual Vec3 evalDiffuse(Vec3 N, Vec3 L, Vec3 color, double diffuse) const = 0;
	virtual Vec3 evalSpecular(Vec3 N, Vec3 L, Vec3 V, Vec3 material_ks, double shininess, double specular) const = 0;
};

class DirectionalLight : public Light
{
private:
	Vec3 _direction;

public:
	DirectionalLight() : Light() {}
	DirectionalLight(Vec3 direction, Vec3 position, Vec3 color)
		: Light(position, color), _direction(direction) {}
	Vec3 getDirection() { return _direction; }
	void setDirection(Vec3 direction) { _direction = direction; }
	virtual Vec3 getRandomOrigin() const { return _position; }
	virtual Vec3 evalAmbient(Vec3 color) const { return Vec3(_color[0] * color[0], _color[1] * color[1], _color[2] * color[2]); }
	virtual Vec3 evalDiffuse(Vec3 N, Vec3 L, Vec3 color, double diffuse) const {
		Vec3 tmp = Vec3(_color[0] * color[0], _color[1] * color[1], _color[2] * color[2]);
		double tmp2 = std::max(dot(N, L), 0.0);
		return diffuse * tmp2 * tmp;
	}
	virtual Vec3 evalSpecular(Vec3 N, Vec3 L, Vec3 V, Vec3 material_ks, double shininess, double specular) const { 
		Vec3 tmp = Vec3(_color[0] * material_ks[0], _color[1] * material_ks[1], _color[2] * material_ks[2]);
		Vec3 H = (L + V) / sqrt(dot(L + V, L + V));
		double tmp1 = std::max(dot(N, L), 0.0);
		double tmp2 = tmp1 <= 0 ? 0.0 : pow(std::max(dot(N, H), 0.0), shininess);
		return tmp * specular * tmp2;
	}
};

#endif