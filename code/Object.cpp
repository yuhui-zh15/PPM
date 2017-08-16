#include "Object.h"
#include "Texture.h"
#include <algorithm>
#include <fstream>
using namespace std;

int Sphere::intersect(const Vec3 &origin, const Vec3 &direction, double &maxt) const {
	Vec3 x = origin - _center;
	double a = dot(direction, direction);
	double b = 2 * dot(direction, x);
	double c = dot(x, x) - _radius * _radius;
	double det = b * b - 4 * a * c;
	if (det > 0) {
		det = sqrt(det);
		double t1 = (0 - b - det) / 2.0 / a;
		double t2 = (0 - b + det) / 2.0 / a;
		if (t1 < maxt) {
			if (t1 > 1e-3) {
				maxt = t1;
				return INTERSECT_OUT;
			}
			else if (t2 > 1e-3 && t2 < maxt) {
				maxt = t2;
				return INTERSECT_IN;
			}
			else return NO_INTERSECT;
		}
		else return NO_INTERSECT;
	}
	else return NO_INTERSECT;
}

Vec3 Sphere::getTextureColor(Vec3 point) const
{
	if (_texture == NULL) return Vec3(1, 1, 1);
	Vec3 normal = getNormal(point);
	double theta = acos(-dot(normal, _V));
	double phi = acos(std::min(std::max(dot(normal, _U) / sin(theta), -1.0), 1.0));
	double u = theta / PI, v = phi / (2 * PI);
	v = (dot(normal, cross(_U, _V)) < 0) ? (1 - v) : v;
	return _texture->getColor(u, v);
}

Vec3 Plane::getTextureColor(Vec3 point) const 
{
	if (_texture == NULL) return Vec3(1, 1, 1);
    const double eps = 1e-3;
    if (abs(_normal.y - 1) <= eps || abs(_normal.y + 1) <= eps) {
	   double v = 0.5 + (point.x - _point.x) / _U.length();
	   double u = 0.5 + (point.z - _point.z) / _V.length();
       return _texture->getColor(u, v);
    }
    else if (abs(_normal.z - 1) <= eps || abs(_normal.z + 1) <= eps ) {
        double v = 0.5 + (point.x - _point.x) / _U.length();
        double u = 0.5 + (point.y - _point.y) / _V.length();
        return _texture->getColor(u, v);
    }
    else if (abs(_normal.x - 1) <= eps || abs(_normal.x + 1) <= eps) {
        double u = 0.5 + (point.y - _point.y) / _U.length();
        double v = 0.5 + (point.z - _point.z) / _V.length();
        return _texture->getColor(u, v);
    }
    return Vec3(1, 1, 1);
}

void AABB::init(const Eigen::Vector3d* controlPoints) {
    maxp = Eigen::Vector3d(-RAND_MAX, -RAND_MAX, -RAND_MAX);
    minp = Eigen::Vector3d(RAND_MAX, RAND_MAX, RAND_MAX);
    for (int i = 0; i < 16; i++) {
        for (int j = 0; j < 3; j++) {
            maxp(j) = std::max(maxp(j), controlPoints[i](j));
            minp(j) = std::min(minp(j), controlPoints[i](j));
    	}
    }
}

double AABB::norm() {
    return (maxp - minp).lpNorm<Eigen::Infinity>();
}
    
bool AABB::intersect(const Eigen::Vector3d& rayorig, const Eigen::Vector3d& raydir) {
    const double eps = 1e-6;
    double ox = rayorig(0), oy = rayorig(1), oz = rayorig(2);
    double dx = raydir(0), dy = raydir(1), dz = raydir(2);
    double x0 = minp(0), y0 = minp(1), z0 = minp(2);
    double x1 = maxp(0), y1 = maxp(1), z1 = maxp(2);
    double tx_min = -RAND_MAX,ty_min = -RAND_MAX, tz_min = -RAND_MAX;
    double tx_max = RAND_MAX, ty_max = RAND_MAX, tz_max = RAND_MAX;
    //x0,y0,z0为包围体的最小顶点
    //x1,y1,z1为包围体的最大顶点
    if(fabs(dx) < eps) {
        //若射线方向矢量的x轴分量为0且原点不在盒体内
        if(ox < x0 || ox > x1) return false ;
    }
    else {
        if(dx>=0) {
            tx_min = (x0-ox)/dx;
            tx_max = (x1-ox)/dx;
        }
        else {
            tx_min = (x1-ox)/dx;
            tx_max = (x0-ox)/dx;
        }
    }
    if(fabs(dy) < eps) {
        //若射线方向矢量的x轴分量为0且原点不在盒体内
        if(oy < y0 || oy > y1) return false ;
    }
    else {
        if(dy>=0) {
            ty_min = (y0-oy)/dy;
            ty_max = (y1-oy)/dy;
        }
        else {
            ty_min = (y1-oy)/dy;
            ty_max = (y0-oy)/dy;
        }
    }
    if(fabs(dz) < eps) {
        //若射线方向矢量的x轴分量为0且原点不在盒体内
        if(oz < z0 || oz > z1) return false ;
    }
    else {
        if(dz>=0) {
            tz_min = (z0-oz)/dz;
            tz_max = (z1-oz)/dz;
        }
        else {
            tz_min = (z1-oz)/dz;
            tz_max = (z0-oz)/dz;
        }
    }
    double t0,t1;
    //光线进入平面处（最靠近的平面）的最大t值 
    t0=std::max(tz_min,std::max(tx_min,ty_min));
    //光线离开平面处（最远离的平面）的最小t值
    t1=std::min(tz_max,std::min(tx_max,ty_max));
    return t0<t1;
}

//P为4个控制点，Bezier曲线在t(0~1)点的值
Eigen::Vector3d Bezier::evalBezierCurve(const Eigen::Vector3d *P, const double &t) const {
    double b0 = (1 - t) * (1 - t) * (1 - t);
    double b1 = 3 * t * (1 - t) * (1 - t);
    double b2 = 3 * t * t * (1 - t);
    double b3 = t * t * t;
    return P[0] * b0 + P[1] * b1 + P[2] * b2 + P[3] * b3;
}

//controlPoints为16个控制点，Bezier曲面在(u(0~1), v(0~1))点的值
Eigen::Vector3d Bezier::evalBezierPatch(const Eigen::Vector3d *controlPoints, const double &u, const double &v) const {
    Eigen::Vector3d uCurve[4];
    for (int i = 0; i < 4; ++i)
        uCurve[i] = evalBezierCurve(controlPoints + 4 * i, u);
    return evalBezierCurve(uCurve, v);
}

//P为4个控制点，Bezier曲线在t(0~1)点的导数值
Eigen::Vector3d Bezier::derivBezier(const Eigen::Vector3d *P, const double &t) const {
    double b0 = -3 * (1 - t) * (1 - t);
    double b1 = 3 * (1 - t) * (1 - t) - 6 * t * (1 - t);
    double b2 = 6 * t * (1 - t) - 3 * t * t;
    double b3 = 3 * t * t;
    return P[0] * b0 + P[1] * b1 + P[2] * b2 + P[3] * b3;
}

//controlPoints为16个控制点，Bezier曲面在(u(0~1), v(0~1))点对u的偏导数值
Eigen::Vector3d Bezier::dUBezier(const Eigen::Vector3d *controlPoints, const double &u, const double &v) const {
    Eigen::Vector3d P[4];
    Eigen::Vector3d vCurve[4];
    for (int i = 0; i < 4; ++i) {
        P[0] = controlPoints[i];
        P[1] = controlPoints[4 + i];
        P[2] = controlPoints[8 + i];
        P[3] = controlPoints[12 + i];
        vCurve[i] = evalBezierCurve(P, v);
    }
    return derivBezier(vCurve, u);
}

//controlPoints为16个控制点，Bezier曲面在(u(0~1), v(0~1))点对v的偏导数值
Eigen::Vector3d Bezier::dVBezier(const Eigen::Vector3d *controlPoints, const double &u, const double &v) const {
    Eigen::Vector3d uCurve[4];
    for (int i = 0; i < 4; ++i) {
        uCurve[i] = evalBezierCurve(controlPoints + 4 * i, u);
    }
    return derivBezier(uCurve, v);
}
    
Eigen::Vector3d* Bezier::SplitIPatch( const Eigen::Vector3d* p ) const {
    Eigen::Vector3d* iPatches;
    iPatches = new Eigen::Vector3d[64];
    Eigen::Vector3d q[16];
    Eigen::Vector3d r[16];
    Eigen::Vector3d q1[16];
    Eigen::Vector3d r1[16];
    Eigen::Vector3d q2[16];
    Eigen::Vector3d r2[16];

    // HullSplitU( p, &q, &r );
    for( int iv = 0; iv < 4; iv++ ) {
        Eigen::Vector3d p0_ = p[iv] ;
        q[iv] = p0_;
        Eigen::Vector3d p1_ = p[4 + iv ] ;
        Eigen::Vector3d q1_ = ( ( p0_ + p1_ ) / 2 );
        q[4 + iv] = q1_;
        Eigen::Vector3d p2_ = p[8 + iv] ;
        Eigen::Vector3d q2_ = ( q1_  / 2 ) + ( ( p1_ + p2_ ) / 4 );
        q[8 + iv] = q2_;
        Eigen::Vector3d p3_ = p[12 + iv ] ;
        r[12 + iv] = p3_;
        Eigen::Vector3d r2_ = ( p2_ + p3_ ) / 2;
        r[8 + iv] = r2_;
        Eigen::Vector3d r1_ = ( r2_ / 2 ) + ( ( p1_ + p2_ ) / 4 );
        r[4 + iv] = r1_;
        Eigen::Vector3d q3_ = ( q2_ + r1_ ) / 2 ;
        q[12 + iv] = q3_;
        r[iv] = q3_;
    }
    
    // HullSplitV( q, &q1, &q2 );
    for( int iv = 0; iv < 4; iv++ ) {
        Eigen::Vector3d p0_ = q[4 * iv] ;
        q1[4 * iv] = p0_;
        Eigen::Vector3d p1_ = q[4 * iv + 1] ;
        Eigen::Vector3d q1_ = ( ( p0_ + p1_ ) / 2 );
        q1[4 * iv + 1] =  q1_;
        Eigen::Vector3d p2_ = q[4 * iv + 2] ;
        Eigen::Vector3d q2_ = ( q1_  / 2 ) + ( ( p1_ + p2_ ) / 4 );
        q1[4 * iv + 2] = q2_;
        Eigen::Vector3d p3_ = q[4 * iv + 3] ;
        q2[4 * iv + 3] = p3_;
        Eigen::Vector3d r2_ = ( p2_ + p3_ ) / 2;
        q2[4 * iv + 2] = r2_;
        Eigen::Vector3d r1_ = ( r2_ / 2 ) + ( ( p1_ + p2_ ) / 4 );
        q2[4 * iv + 1] =  r1_;
        Eigen::Vector3d q3_ = ( q2_ + r1_ ) / 2 ;
        q1[4 * iv + 3] =  q3_;
        q2[4 * iv] = q3_;
    }
    
    // HullSplitV( r, &r1, &r2 );
    for( int iv = 0; iv < 4; iv++ ) {
        Eigen::Vector3d p0_ = r[4 * iv] ;
        r1[4 * iv] = p0_;
        Eigen::Vector3d p1_ = r[4 * iv + 1] ;
        Eigen::Vector3d q1_ = ( ( p0_ + p1_ ) / 2 );
        r1[4 * iv + 1] =  q1_;
        Eigen::Vector3d p2_ = r[4 * iv + 2] ;
        Eigen::Vector3d q2_ = ( q1_ / 2 ) + ( ( p1_ + p2_ ) / 4 );
        r1[4 * iv + 2] = q2_;
        Eigen::Vector3d p3_ = r[4 * iv + 3] ;
        r2[4 * iv + 3] = p3_;
        Eigen::Vector3d r2_ = ( p2_ + p3_ ) / 2;
        r2[4 * iv + 2] = r2_;
        Eigen::Vector3d r1_ = ( r2_ / 2 ) + ( ( p1_ + p2_ ) / 4 );
        r2[4 * iv + 1] =  r1_;
        Eigen::Vector3d q3_ = ( q2_ + r1_ ) / 2 ;
        r1[4 * iv + 3] =  q3_;
        r2[4 * iv] = q3_;
    }
    
    for (int i = 0; i < 16; i++) {
	    iPatches[i] = q1[i];
	    iPatches[16 + i] = q2[i];
	    iPatches[32 + i] = r1[i];
	    iPatches[48 + i] = r2[i];
	}
    return iPatches;
}

void Bezier::createObj(const Eigen::Vector3d *controlPoints_) {
    uint32_t divs = 8;
    std::unique_ptr<Eigen::Vector3d []> P(new Eigen::Vector3d[(divs + 1) * (divs + 1)]);
    std::unique_ptr<uint32_t []> nvertices(new uint32_t[divs * divs]);
    std::unique_ptr<uint32_t []> vertices(new uint32_t[divs * divs * 4]);
    std::unique_ptr<Eigen::Vector3d []> N(new Eigen::Vector3d[(divs + 1) * (divs + 1)]);
    std::unique_ptr<Eigen::Vector2d []> st(new Eigen::Vector2d[(divs + 1) * (divs + 1)]);

    for (uint16_t j = 0, k = 0; j < divs; ++j) {
        for (uint16_t i = 0; i < divs; ++i, ++k) {
            nvertices[k] = 4;
            vertices[k * 4 + 0] = (divs + 1) * j + i + 1;
            vertices[k * 4 + 1] = (divs + 1) * j + i + 2;
            vertices[k * 4 + 2] = (divs + 1) * (j + 1) + i + 2;
            vertices[k * 4 + 3] = (divs + 1) * (j + 1) + i + 1;
        }
    }

    for (uint16_t j = 0, k = 0; j <= divs; ++j) {
        double v = j / (double)divs;
        for (uint16_t i = 0; i <= divs; ++i, ++k) {
            double u = i / (double)divs;
            P[k] = evalBezierPatch(controlPoints_, u, v);
            Eigen::Vector3d dU = dUBezier(controlPoints_, u, v);
            Eigen::Vector3d dV = dVBezier(controlPoints_, u, v);
            N[k] = dU.cross(dV);
            N[k].normalize();
            st[k](0) = u;
            st[k](1) = v;
        }
    }
    std::ofstream fout("model.obj");
    for (int i = 0; i < (divs + 1) * (divs + 1); i++) {
        fout << "v " << P[i][0] << " " << P[i][1] << " " << P[i][2] << std::endl;
    }
    for (int i = 0; i < divs * divs; i++) {
        fout << "f " << vertices[4 * i + 0] << " " << vertices[4 * i + 1] << " " << vertices[4 * i + 2] << " " << vertices[4 * i + 3] << std::endl;
    }
    fout.close();
}

Vec3 Bezier::getNormal(Vec3 point) const {
	Vec3 ret(_normal(0), _normal(1), _normal(2));
	return ret;
}

int Bezier::intersect(const Vec3 &origin, const Vec3 &direction, double &maxt) const {
    const double eps = 0.01;
    Eigen::Vector3d rayorig(origin.x, origin.y, origin.z);
    Eigen::Vector3d raydir(direction.x, direction.y, direction.z);
    raydir.normalize();
    std::queue<Node> que;
    Node root(controlPoints);
    que.push(root);
    while (!que.empty()) {
        Node front = que.front();
        if (front.box.norm() < eps) break;
        que.pop();
        Eigen::Vector3d *newpointslist = SplitIPatch(front.controlPoints);

        Node b00(newpointslist);
        if (b00.box.intersect(rayorig, raydir)) { que.push(b00); }

        Node b01(newpointslist + 16);
        if (b01.box.intersect(rayorig, raydir)) { que.push(b01); }

        Node b10(newpointslist + 32);
        if (b10.box.intersect(rayorig, raydir)) { que.push(b10); }

        Node b11(newpointslist + 48);
        if (b11.box.intersect(rayorig, raydir)) { que.push(b11); }
        delete []newpointslist;
    }

    const double eps1 = 1e-6;
    const double eps2 = 1e-6;
    Eigen::Vector3d xmin(-1, -1, -1);
    double tmin = RAND_MAX;
    Node nodemin(controlPoints);
    while (!que.empty()) {
        Node front = que.front();
        que.pop();
        //Newton
        Eigen::Vector3d x(0, 0, 0);
        Eigen::Vector3d x_last(-1, -1, -1);
        int cnt = 0;
        while ((x - x_last).lpNorm<Eigen::Infinity>() > eps1 && cnt < 50) {
            x_last = x;
            Eigen::Vector3d L = rayorig + x_last(0) * raydir;
            Eigen::Vector3d P = evalBezierPatch(front.controlPoints, x_last(1), x_last(2));
            Eigen::Vector3d F = L - P;
            Eigen::Vector3d dU = dUBezier(front.controlPoints, x_last(1), x_last(2));
            Eigen::Vector3d dV = dVBezier(front.controlPoints, x_last(1), x_last(2));
            Eigen::Matrix3d J;
            J(0) = raydir(0), J(1) = raydir(1), J(2) = raydir(2);
            J(3) = -dU(0), J(4) = -dU(1), J(5) = -dU(2);
            J(6) = -dV(0), J(7) = -dV(1), J(8) = -dV(2);
            x = x_last - J.inverse() * F;
            cnt++;
        }

        if (cnt < 50 && x(0) > eps2 && x(1) >= - eps2 && x(1) <= 1 + eps2 && x(2) >= - eps2 && x(2) <= 1 + eps2 && x(0) < tmin) {
            tmin = x(0);
            xmin = x;
            nodemin = front;
        }
    }

    if (xmin(0) < eps2 || xmin(0) > maxt || xmin(1) < - eps2 || xmin(1) > 1 + eps2 || xmin(2) < - eps2 || xmin(2) > 1 + eps2) {
        return NO_INTERSECT;
    }
    maxt = tmin;
    Eigen::Vector3d dU = dUBezier(nodemin.controlPoints, xmin(1), xmin(2));
    Eigen::Vector3d dV = dVBezier(nodemin.controlPoints, xmin(1), xmin(2));
    _normal = dU.cross(dV);
    _normal.normalize();
    if (_normal.dot(raydir) > 0) _normal = -_normal;
	
	// texture
	if (_texture != NULL)
		_color = _texture->getColor(xmin(1), xmin(2));

    return INTERSECT_OUT;
}