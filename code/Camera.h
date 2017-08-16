#ifndef CAMERA_H
#define CAMERA_H

#include "Vec3.h"

#define DEFAULT_P Vec3(0, 30, -40)
#define DEFAULT_F Vec3(0, 0, -1440)
#define DEFAULT_H Vec3(0, -1080, 0)
#define DEFAULT_W Vec3(1440, 0, 0)
#define DEFAULT_APERTURE 0.4
#define DEFAULT_FOCAL 100
#define DEFAULT_ITER 10

class Camera {
private:
	Vec3 _position;
	Vec3 _F, _H, _W;
	int _height, _width;
	double _aperture, _focalLen;
	int _aperIter;
	
public:
	Camera(const Vec3 &position = DEFAULT_P,
		const Vec3 &F = DEFAULT_F,
		const Vec3 &H = DEFAULT_H,
		const Vec3 &W = DEFAULT_W,
		const double aperture = DEFAULT_APERTURE,
		const double focalLen = DEFAULT_FOCAL,
		const int aperIter = DEFAULT_ITER)
		: _position(position), _F(F), _H(H), _W(W), _aperture(aperture), _focalLen(focalLen), _aperIter(aperIter) {
		_height = H.length(), _width = W.length();
	}
	~Camera() {}

	Vec3 getPosition() const { return _position; }
	int getHeight() const { return _height; }
	int getWidth() const { return _width; }
	double getAperture() const { return _aperture; }
	int getAperIter() const { return _aperIter; }

	Vec3 getRay(double i, double j) const {
		return (_F + _H * (2 * i / _height - 1) + _W * (2 * j / _width - 1)).normalized();
	}

	void getRay(double i, double j, Vec3 &ori, Vec3 &dir)
	{
		Vec3 emit = _F + _H * (2 * i / _height - 1) + _W * (2 * j / _width - 1);
		emit = emit / (-emit.z);
		Vec3 fp = _position + emit * _focalLen;
		double x, y;
		do
		{
			x = rand01() * 2.0 - 1.0;
			y = rand01() * 2.0 - 1.0;
		} while (x * x + y * y >= 1);
		Vec3 dx = _H.normalized(), dy = _W.normalized();
		ori = _position + dx * _aperture * x + dy * _aperture * y;
		dir = (fp - ori).normalized();
	}
};

#endif