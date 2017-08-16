#ifndef OBJECT_H
#define OBJECT_H

#include "Vec3.h"
#include "object.h"
#include "eigen3/Eigen/Dense"
#include "Texture.h"
#include <iostream>
#include <cmath>
#include <queue>
#include <vector>
#include <string>
#include <cstdlib>

#define INTERSECT_OUT 1
#define INTERSECT_IN -1
#define NO_INTERSECT 0

class Object
{
protected:
	Vec3 _color;
	double _diffuse, _specular, _shininess, _reflectivity, _refractivity;
	double _ior;
	Texture *_texture;

public:
	Object() : _texture(NULL) {}
	Object(Vec3 color, double diffuse, double specular, double shininess, double reflectivity, double refractivity, double ior)
		: _color(color), _diffuse(diffuse), _specular(specular), _shininess(shininess), _reflectivity(reflectivity), _refractivity(refractivity), _ior(ior), _texture(NULL) {}
	void setColor(Vec3 color) { _color = color; }
	void setMaterial(Vec3 color = Vec3(), double diffuse = 0, double specular = 0, double shininess = 0, double reflectivity = 0, double refractivity = 0, double ior = 1) {
		_color = color; _diffuse = diffuse; _specular = specular; _shininess = shininess; _reflectivity = reflectivity; _refractivity = refractivity; _ior = ior;
	}
	void setShininess(double shininess) { _shininess = shininess; }
	void setReflectivity(double reflectivity) { _reflectivity = reflectivity; }
	void setRefractivity(double refractivity) { _refractivity = refractivity; }
	void setDiffuse(double diffuse) { _diffuse = diffuse; }
	void setIOR(double ior) { _ior = ior; }
	void setTexture(Texture *texture) { _texture = texture; }
	Vec3 getColor() const { return _color; }
	double getShininess() const { return _shininess; }
	double getReflectivity() const { return _reflectivity; }
	double getRefractivity() const { return _refractivity; }
	double getDiffuse() const { return _diffuse; }
	double getSpecular() const { return _specular; }
	double getIOR() const { return _ior; }
	Texture *getTexture() const { return _texture; }
	virtual Vec3 getTextureColor(Vec3 point) const = 0;
	virtual Vec3 getNormal(Vec3 point) const = 0;
	virtual int intersect(const Vec3 &origin, const Vec3 &direction, double &maxt) const = 0;
};

class Plane : public Object
{
	Vec3 _point;
	Vec3 _normal; 
	Vec3 _U, _V;

public:
	Plane() : Object() {}
	Plane(Vec3 point, Vec3 normal) 
		: Object(), _point(point), _normal(normal), _U(Vec3(250, 0, 0)), _V(Vec3(0, 0, 250)) { }
	Plane(Vec3 point, Vec3 normal, Vec3 color, double diffuse, double specular, double shininess, double reflectivity, double refractivity, double ior)
		: _point(point), _normal(normal), Object(color, diffuse, specular, shininess, reflectivity, refractivity, ior) { }
	Vec3 getPoint() { return _point; }
	virtual Vec3 getNormal(Vec3 point) const { return _normal; }
	virtual int intersect(const Vec3 &origin, const Vec3 &direction, double &maxt) const
	{
		double tmp1 = dot(_point - origin, _normal);
		double tmp2 = dot(direction, _normal);
		if (fabs(tmp2) < 1e-8) return NO_INTERSECT;
		double t = tmp1 / tmp2;
		if (t > 1e-3 && t < maxt)
		{
			maxt = t;
			return INTERSECT_OUT;
		}
		return NO_INTERSECT;
	}
	virtual Vec3 getTextureColor(Vec3 point) const;
};

class Sphere : public Object
{
	Vec3 _center;
	double _radius;
	Vec3 _U, _V;

public:
	Sphere(Vec3 center, double radius)
		: Object(), _center(center), _radius(radius), _U(Vec3(1, 0, -3).normalized()), _V(Vec3(0, 1, 0)) {}
	Sphere(Vec3 center, double radius, Vec3 color, double diffuse, double specular, double shininess, double reflectivity, double refractivity, double ior)
		: Object(color, diffuse, specular, shininess, reflectivity, refractivity, ior)
		, _center(center), _radius(radius), _U(Vec3(1, 0, 0)), _V(Vec3(0, 1, 0)) {}
	Vec3 getCenter() { return _center; }
	double getRadius() { return _radius; }
	void setCenter(Vec3 center) { _center = center; }
	void setRadius(double radius) { _radius = radius; }
	virtual Vec3 getNormal(Vec3 point) const { return (point - _center).normalized(); }
	virtual int intersect(const Vec3 &origin, const Vec3 &direction, double &maxt) const;
	virtual Vec3 getTextureColor(Vec3 point) const;
};

class AABB {
    Eigen::Vector3d maxp, minp;

public:
    void init(const Eigen::Vector3d* controlPoints);
    double norm();
    bool intersect(const Eigen::Vector3d& rayorig, const Eigen::Vector3d& raydir);
};

struct Node {
	Eigen::Vector3d controlPoints[16];
    AABB box;
    Node(const Eigen::Vector3d* controlPoints_) {
        for (int i = 0; i < 16; i++) controlPoints[i] = controlPoints_[i];
        box.init(controlPoints);
    }
};

class Bezier: public Object {
    Eigen::Vector3d controlPoints[16];
    Eigen::Vector3d _center;
    mutable Eigen::Vector3d _normal;
	mutable Vec3 _color;
public:
	// Beizer() : Object() {}
	Bezier(Vec3 center, Eigen::Vector3d *controlPoints_): Object() {
		_center = Eigen::Vector3d(center.x, center.y, center.z);
    	for (int i = 0; i < 16; i++) controlPoints[i] = controlPoints_[i].eval() + _center;
    }
	Bezier(Vec3 center, Eigen::Vector3d *controlPoints_, Vec3 color, double diffuse, double specular, double shininess, double reflectivity, double refractivity, double ior)
		: Object(color, diffuse, specular, shininess, reflectivity, refractivity, ior) {
		_center = Eigen::Vector3d(center.x, center.y, center.z);
		for (int i = 0; i < 16; i++) controlPoints[i] = controlPoints_[i].eval() + _center;
	}
	virtual int intersect(const Vec3 &origin, const Vec3 &direction, double &maxt) const;
	virtual Vec3 getNormal(Vec3 point) const;
	virtual Vec3 getTextureColor(Vec3 point) const { return (_texture == NULL)? Vec3(1, 1, 1): _color; }

    Eigen::Vector3d evalBezierCurve(const Eigen::Vector3d *P, const double &t) const;
    Eigen::Vector3d evalBezierPatch(const Eigen::Vector3d *controlPoints, const double &u, const double &v) const;
    Eigen::Vector3d derivBezier(const Eigen::Vector3d *P, const double &t) const;
    Eigen::Vector3d dUBezier(const Eigen::Vector3d *controlPoints, const double &u, const double &v) const;
    Eigen::Vector3d dVBezier(const Eigen::Vector3d *controlPoints, const double &u, const double &v) const;
    Eigen::Vector3d* SplitIPatch( const Eigen::Vector3d* p) const;   
    void createObj(const Eigen::Vector3d *controlPoints_);
};

#endif