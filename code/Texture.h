#ifndef TEXTURE_H
#define TEXTURE_H

#include "Vec3.h"
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>  
#include <opencv2/highgui/highgui.hpp>

class Texture
{
public:
	Texture(const std::string &filename) {
        cv::Mat_<cv::Vec3b> img = cv::imread(filename, cv::IMREAD_COLOR);
        _height = img.rows, _width = img.cols;

        _img.resize(_height);
        for (int i = 0; i < _height; i++) {
            _img[i].resize(_width);
        }
        for (int i = 0; i < _height; i++) {
            for (int j = 0; j < _width; j++)
            {
                _img[i][j][0] = img(i, j)[2] / 255.0;
                _img[i][j][1] = img(i, j)[1] / 255.0;
                _img[i][j][2] = img(i, j)[0] / 255.0;
            }
        }
    }
	Vec3 getColor(double u, double v) const {
        double fracU = (u - floor(u)) * _height;
        double fracV = (v - floor(v)) * _width;
        int u1 = (int)floor(fracU + EPSI), u2 = u1 + 1;
        int v1 = (int)floor(fracV + EPSI), v2 = v1 + 1;
        double du = u2 - fracU, dv = v2 - fracV;
        if (u1 < 0) u1 = _height - 1; if (u2 == _height) u2 = 0;
        if (v1 < 0) v1 = _width - 1; if (v2 == _width) v2 = 0;
        return _img[u1][v1] * du * dv + _img[u1][v2] * du * (1 - dv) + _img[u2][v1] * (1 - du) * dv + _img[u2][v2] * (1 - du) * (1 - dv);
    }

private:
	int _height, _width;
	std::vector<std::vector<Vec3>> _img;
};

#endif